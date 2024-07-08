#include "esp32s2_task.h"
#include "esp32s2_wifi.h"
#include "esp32s2_https_protocol.h"
#include "esp32s2_i2c_protocol.h"
#include "esp32s2_gpio.h"
#include "esp32s2_24lc512.h"
#include "esp32s2_socket.h"
#include "esp32s2_adc.h"
#include "esp32s2_algorithm.h"
#include "esp32s2_data_log.h"
#include "esp32s2_main.h"
#include "esp32s2_ina226.h"
#include "esp32s2_pac1952.h"
#include "esp32s2_buzzer.h"
#include "esp32s2_parse_data.h"
#include "esp32s2_button.h"
#include "freertos/event_groups.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp32s2_fatal_err.h"
//#include "esp_clk.h"

#define TAG "esp32s2_tasks.c"
#define MAX_RETRY_FOR_TIMESYNC 15
#define MAX_RESTART_COUNT 3
#define DEVICE_RESTART_CNT_FILE "/spiffs/restart_count.txt" 

#define MAX_EEPROM_READ 5
#define DYNAMIC_OTA_URL_KEY "/api/getOtaVersion/"
#define DYNAMIC_OTA_URL_KEY_LEN     19

extern uint8_t server_cert_pem_start[2048];
extern esp_err_t  client_event_handler(esp_http_client_event_t *evt);
extern uint8_t ap_ssid_buff[AP_SSID_LEN], ap_psw_buff[AP_PSW_LEN];
extern uint8_t connection_stat_check, config_calib_mode;
extern uint8_t packet_length;
extern char dev_id[12];

xSemaphoreHandle serverReset;
xSemaphoreHandle factoryReset;
xSemaphoreHandle otaSemaphore;
xSemaphoreHandle logdataSemaphore;
xSemaphoreHandle connectionSemaphore;
xSemaphoreHandle switchpressSemaphore;
xSemaphoreHandle switchpressConfirmedSemaphore;
xSemaphoreHandle serverSemaphore;
xSemaphoreHandle testConnectionSemaphore;
xSemaphoreHandle pacAlertSemaphore;
xSemaphoreHandle i2cSemaphore;
xSemaphoreHandle ledTaskSemaphore;
xSemaphoreHandle overVoltageProtectionSemaphore;
xSemaphoreHandle stateMachineSemaphore;
xSemaphoreHandle dataLoggingSemaphore;

TaskHandle_t serverTask_handle = NULL;
TaskHandle_t clientTask_handle = NULL;
TaskHandle_t normalTask_handle = NULL;
TaskHandle_t leastTask_handle  = NULL;

EventGroupHandle_t task_watchdog;


uint8_t https_request, https_action=0;
https_request_buffer_body_t https_request_buffer_body = {0,0,0,{0,0,0,0,0,0,0,0,0,0}};

uint8_t exit_server_task, access_point;
uint8_t algorithm_count=0,rtc_sync=0;
bool wifi_connection_status = 0;

uint8_t server_connect_retry=0,server_connect_retry_cnt=0;
uint8_t turt_state = TURT_STAT_ON, switch_state = 0, device_state=ALGO_ON,user_device_off=0;
uint8_t wait_time_sync=0;
uint8_t connection_stat_check=0,connection_stat=0,connection_stat_check_cnt=0;
uint16_t configuration_timeout_cnt;

operations_parameters_t critical_operations_parameters;
operations_parameters_t normal_operations_parameters;
operations_parameters_t least_priority_operations_parameters;

uint8_t fw_update=0,stop_lptask=0,perform_ota_task=0;

uint16_t watchdog_timer_count=0, watchdog_timer_reset=12000, watchdog_timer_stop=1;

uint8_t stopBatteryOverVoltageProtectionOnCalibration=0;

bool ledBlinkOn=false;

void set_rtc_sync_status(uint8_t status)
{
    rtc_sync = status;
}

uint8_t get_rtc_sync_status()
{
    return rtc_sync;
}

void createSemaphores(void){
  serverReset = xSemaphoreCreateBinary();
  factoryReset = xSemaphoreCreateBinary();
  otaSemaphore = xSemaphoreCreateBinary();
  logdataSemaphore = xSemaphoreCreateBinary();
  connectionSemaphore = xSemaphoreCreateBinary();
  switchpressSemaphore = xSemaphoreCreateBinary();
  switchpressConfirmedSemaphore = xSemaphoreCreateBinary();
  serverSemaphore = xSemaphoreCreateBinary();
  testConnectionSemaphore = xSemaphoreCreateBinary();
  pacAlertSemaphore = xSemaphoreCreateBinary();
  ledTaskSemaphore = xSemaphoreCreateBinary();
  overVoltageProtectionSemaphore = xSemaphoreCreateBinary();
  stateMachineSemaphore = xSemaphoreCreateBinary();
  dataLoggingSemaphore = xSemaphoreCreateBinary();
  i2cSemaphore = xSemaphoreCreateBinary();
  task_watchdog = xEventGroupCreate();
}

void test_wifi_config(void *param){
  while(true){
    if(xSemaphoreTake(testConnectionSemaphore, 10000 / portTICK_RATE_MS)){
        ESP_LOGI("CONFIGUARTION","Testing server connectivity");
      blink(led_green_color,ONE_SECOND_BLINK);
      deleteConfigCalibTask();
      vTaskDelay(1000/portTICK_PERIOD_MS);
      connection_stat_check=1;
      connectSTA( (char *) ap_ssid_buff , (char *) ap_psw_buff );
    }
  }
}

void mainTimerTask( void * param){
  static uint8_t ledTaskCount=0,overVoltageTaskCount=0,stateMachineTaskCount=0;
  static uint16_t dataLoggingTaskCount=0;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1;
  xLastWakeTime = xTaskGetTickCount();
  while(1){
    if(++ledTaskCount>10){
      ledTaskCount=0; 
      xSemaphoreGive(ledTaskSemaphore);
    }
    if(++overVoltageTaskCount>30){
      overVoltageTaskCount=0; 
      xSemaphoreGive(overVoltageProtectionSemaphore);
    }
    if(++stateMachineTaskCount>100){
      stateMachineTaskCount=0;
      xSemaphoreGive(stateMachineSemaphore); 
    }
    if(++dataLoggingTaskCount>1000){
      dataLoggingTaskCount=0; 
      xSemaphoreGive(dataLoggingSemaphore);
    }        
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void overvoltageProtectionTask( void * param){
    while(1){
      xSemaphoreTake(overVoltageProtectionSemaphore,portMAX_DELAY);
      if(!test_error((ERR_NOT_CALIB | ERR_NOT_CONFIG))){
        if(ledBlinkOn){
          if(stopBatteryOverVoltageProtectionOnCalibration==0){
            check_battery_voltage(); 
          }
        }
      }
      xEventGroupSetBits(task_watchdog,overvoltageProtectionTaskEvent);
    }
}

void ledBlinkTask( void * param){
    while(1){
      xSemaphoreTake(ledTaskSemaphore,portMAX_DELAY);
      if(ledBlinkOn){
        led_blink();
      }
    }
}

void stateMachineTask( void * param){
    static uint8_t seconds_cnt = 0;
    static uint8_t minutes_cnt = 0;
    static uint8_t hour_cnt = 0;
    static uint16_t day_cnt = 0;
    static uint32_t year_cnt = 0;

    while(1){
      xSemaphoreTake(stateMachineSemaphore,portMAX_DELAY);

      if(test_common_flag(CF_RESET_DEV)){
        clear_common_flag(CF_CLR_ALL);
        xSemaphoreGive(serverReset);
      }
 
      server_connect_retry_cnt+=1;
 
      if(xPortGetFreeHeapSize() < 10240) {
          ESP_LOGI(TAG,"Heap value %d",xPortGetFreeHeapSize());
      }
      if(if_algo_stopped() == 0){
        if(!test_error((ERR_NOT_CALIB | ERR_NOT_CONFIG))){
          ESP_LOGI(TAG,"iCON LV 8.00 algorithm");
          turtle_algorithm();
          watchdog_timer_count=0;
        }
        else{
          watchdog_timer_count=0;
          if(test_error(ERR_NOT_CALIB)){
            ESP_LOGI(TAG,"Device not calibrated");
          }

          if(test_error(ERR_NOT_CONFIG)){
            ESP_LOGI(TAG,"Device not configured");
          }
        }
      }
#if ICUBE == 0
      #warning "Inverter failure buzzer enabled"
      if(test_system_flag(SF_INV_FAIL)){ toggle_buzzer(); };
#endif 
      xEventGroupSetBits(task_watchdog,stateMachineTaskEvent);
      seconds_cnt++;
      if(seconds_cnt == 60) {
          minutes_cnt++;
          seconds_cnt = 0;
      }
      if(minutes_cnt == 60) {
          hour_cnt++;
          minutes_cnt = 0;
      }
      if(hour_cnt == 24) {
          day_cnt++;
          hour_cnt = 0;
      }
      if(day_cnt == 365) {
          year_cnt++;
          day_cnt = 0;
      }
      if( (minutes_cnt == 30) && (seconds_cnt == 0) ) {
          ESP_LOGI("UP TIME", "%d Year %d Days %d Hour %d Minutes %d Seconds",
                  year_cnt,day_cnt,hour_cnt,minutes_cnt,seconds_cnt);
      }
    }
}

void OnConnected(void *para){
  while(true){
    if(xSemaphoreTake(logdataSemaphore, 10000 / portTICK_RATE_MS)){
        vTaskDelay(10/portTICK_RATE_MS);
      if(connection_stat_check){
        test_server_connection();
      }
      else{
        if(https_action==1){
          bidirectional_communication_commands(&https_request_buffer_body);
        }
        else{
          send_DATA_to_server();
        }
      }
      ESP_LOGI("inside on connected", "Done!");
    }
  }
}

void dataLoggingTask( void * param){
  while(1){
    xSemaphoreTake(dataLoggingSemaphore,portMAX_DELAY);
    xEventGroupSetBits(task_watchdog,dataLoggingTaskEvent);

    if(watchdog_timer_stop==0){
      ESP_LOGI(TAG,"Checking watchdogTimer");
      if(++watchdog_timer_count>2){
        ESP_LOGI(TAG,"Watchdog Timer Expired");
        switch_state_4_5();
        ESP_LOGI(TAG,"Restarting the system");
        fatal_err_handler(WATCHDOG,WD_EXPIRED);
      }
    }

    static uint8_t getInfoTime=0;
    if(++getInfoTime>30){
      getInfoTime=0;
      https_req_buff_write(&https_request_buffer_body,get_information);
    }

    if((if_algo_stopped() == 0)&&(stop_lptask==0)){
      ESP_LOGI(TAG,"10 second tick");
      if(!test_error((ERR_NOT_CALIB | ERR_NOT_CONFIG))){
        least_priority_operations();
      }
    }

    if(connection_stat_check){
      if(++connection_stat_check_cnt>6){
        connection_stat_check=0;
        set_algo_status(0);
        watchdog_timer_stop=0;
      }
    }

    if(if_algo_stopped() == 1) {
      if(++configuration_timeout_cnt>=30){
          ESP_LOGI("ALERT","CONFIGURATION TIMEOUT EXCEEDED, STOPPING THE CONFIGURATION MODE");
          configuration_timeout_cnt=0;
          set_algo_status(0);
          watchdog_timer_stop=0;
          deleteConfigCalibTask();
      }
    }
  }
}

void TrySyncTime(void){
  if(get_rtc_sync_status() == 0) {
    ESP_LOGI(TAG,"Trying to sync time");
    least_priority_operations_parameters.status=waiting_for_time_synced;
    esp_wifi_stop();
    if(access_point==primary){
      ESP_LOGI(TAG,"Connecting to primary access point");
      connectSTA((char *)ap_ssid_buff,(char *)ap_psw_buff);
    }
    else{
      ESP_LOGI(TAG,"Connecting to secondary access point");
      connectSTA("E24by7","Energy24by7");
    }
  } else {
    wait_time_sync=0;
    least_priority_operations_parameters.status=lpos_data_not_available;    
  }
}

void TimeoutSyncTime(void){
  if(get_rtc_sync_status() == 0 ){
    ESP_LOGI(TAG,"Waiting to sync time");
    if(++wait_time_sync > 2){
      wait_time_sync=0;
      least_priority_operations_parameters.status = time_not_synced;
      if(access_point==primary){
        access_point=secondary;
      }
      else{
        access_point=primary;
      }
      ESP_LOGI(TAG,"Couldn't sync time");
    }
  } else {
    wait_time_sync=0;
    least_priority_operations_parameters.status=lpos_data_not_available;
  }  
}

void checkDataAvailable(void){
    uint16_t log_available = 0;
    if(fw_update) {
        ESP_LOGI(TAG,"OTA update");
        fw_update=0;
        perform_ota_task=1;
        least_priority_operations_parameters.status=lpos_data_available;      
    } else {
        log_available = turtle_log_available();
        //if(log_available > packet_length){
        if(log_available > 10) { // Temp fix to not sending log request.
            ESP_LOGI(TAG,"Data available in EEPROM of %d numbers",log_available);
            least_priority_operations_parameters.status=lpos_data_available;
        } else {
            if(https_req_buff_available(&https_request_buffer_body)!=0){
                ESP_LOGI(TAG,"Request is %d",https_req_buff_peek(&https_request_buffer_body));
                https_action=1;
                least_priority_operations_parameters.status=lpos_data_available;
            } else{
                ESP_LOGI(TAG,"No operation to perform");
            }
        }    
    }
}

void connectingToAP(void){
  if(wifi_connection_status){
    ESP_LOGI(TAG,"Already connected to AP");
    least_priority_operations_parameters.status=lpos_link_connected;
  }
  else{
    ESP_LOGI(TAG,"Not connected to AP");
    least_priority_operations_parameters.status=lpos_link_not_connected;
  }
}

void trySwitchingAP(void){
  if(access_point==primary){
    ESP_LOGI(TAG,"Connecting to primary access point");
    connectSTA((char *)ap_ssid_buff,(char *)ap_psw_buff);
  }
  else{
    ESP_LOGI(TAG,"Connecting to secondary access point");
    connectSTA("E24by7","Energy24by7");
  }
  least_priority_operations_parameters.status=lpos_link_connected;
  least_priority_operations_parameters.wait_time = 0;
}

void waitConnectingToAP(void){
  if(wifi_connection_status==0){
    if(++least_priority_operations_parameters.wait_time>1){
      esp_wifi_stop();
      if(access_point==primary){
        access_point=secondary;
      }else{
        access_point=primary;
      }
      least_priority_operations_parameters.wait_time=0;
      least_priority_operations_parameters.status=lpos_link_not_connected;
    }
    else{
      if(access_point==primary){
        ESP_LOGI(TAG,"Wait to connect with primary access point");
      }
      else{
        ESP_LOGI(TAG,"Wait to connect with secondary access point");
      }
    }
  }
  else{
    if(access_point==primary){
      ESP_LOGI(TAG,"Connected to primary access point");
    }
    else{
      ESP_LOGI(TAG,"Connected to secondary access point");
    }
    if(perform_ota_task){
      stop_lptask=1;
      perform_ota_task = 0;
      fw_update=0;
      xSemaphoreGive(otaSemaphore);
    }
    else{
      xSemaphoreGive(logdataSemaphore);
    }
    least_priority_operations_parameters.wait_time=0;
    least_priority_operations_parameters.status=lpos_waiting_for_response;
  }  
}

void waitingForServerResponse(void){
  if(++least_priority_operations_parameters.wait_time > 12){
    least_priority_operations_parameters.status=lpos_communication_failed;
    least_priority_operations_parameters.wait_time=0;
    ESP_LOGI(TAG,"Data logging failed");
  }
  else{
    ESP_LOGI(TAG,"Waiting for response from server");
  }
}

void onResponseFromServer(void){
    ESP_LOGI(TAG,"Received response from server");
    least_priority_operations_parameters.wait_time=0;
    if(https_action==1){
        https_req_buff_flush(&https_request_buffer_body);
        https_action=0;
    } else {
        if(get_server_resp_status() == 1) {
            ESP_LOGI(TAG,"EEPROM tail updated");
            turtle_log_update_tail();
        } else {
            ESP_LOGI(TAG,"Server response is not OK.. EEPROM tail not updating");
        }
    }
    least_priority_operations_parameters.status=lpos_data_not_available;
    esp_wifi_stop();  //VINEETH
}

void onCommunicationFailed(void){
  least_priority_operations_parameters.status=lpos_data_not_available;
  esp_wifi_stop(); //VINEETH
}

static inline int get_restart_count()
{
    int restart_cnt = 0;
    FILE* fp = fopen(DEVICE_RESTART_CNT_FILE, "r");
    if(fp == NULL) {
        return 0; // Return device reset count as 0
    } else {
        fseek(fp, 0, SEEK_SET);
        fread(&restart_cnt, 1, sizeof(int), fp);
        fclose(fp);
    }
    return restart_cnt;
}

static inline int update_restart_count(int restart_cnt)
{
    int ret = -1;
    FILE* fp = fopen(DEVICE_RESTART_CNT_FILE, "w+");
    if(fp != NULL) {
        if( fwrite(&restart_cnt, 1, sizeof(int), fp) == sizeof(int)) {
            ESP_LOGI(TAG, "Writing value [%d] to [%s] SUCCESS",restart_cnt,DEVICE_RESTART_CNT_FILE);
            ret = 0;
        } else {
            ESP_LOGI(TAG, "Writing value [%d] to [%s] FAILS",restart_cnt,DEVICE_RESTART_CNT_FILE);
            ret = -1;
        }
        fclose(fp);
    } else {
        ESP_LOGI(TAG,"File [%s] open fails",DEVICE_RESTART_CNT_FILE);
        ret = -1;
    }
    return ret;
}


void least_priority_operations(void){
    static int timesync_failed_count = 0;
    int restart_cnt = 0;
    int readback_val = 0;
    static int timesync_updated = 0;
    switch(least_priority_operations_parameters.status){
        case time_not_synced:
            ESP_LOGI("LPOS","lpos_time_not_synced");
            /* 
             * Time get synced once device gets Wi-Fi connection,
             * Obtain ip, sync time with NTP. If device is in
             * time_not_synced means, there could be following 
             * reasons for it.
             * 1) Device do not connect to Wi-Fi
             * 2) Obtaining IP fails
             * 3) Internet is not available
             * 4) NTP is not accessible
             * If device is not able to time sync, after a certain
             * no of retries, restart the device. This total no
             * of retries can be limited to a max value for the
             * life time of the device.
             */
            TrySyncTime();
            timesync_failed_count++;
            if(timesync_failed_count >= MAX_RETRY_FOR_TIMESYNC) {
                ESP_LOGI("LPOS_TIME_SYNC_ERR","Maximum retry for timesync reached");
                timesync_failed_count = 0;
                /* Check the maximum restart reached */
                restart_cnt = get_restart_count();
                if(restart_cnt < MAX_RESTART_COUNT) {
                    ESP_LOGI("LPOS_TIME_SYNC_ERR","Device restart count: %d",restart_cnt);
                    /* Update restart count before restarting */
                    if (update_restart_count(restart_cnt+1) == 0) { // Updation success
                        // Double check to ensure that value is written properly
                        readback_val = get_restart_count();
                        if(readback_val == (restart_cnt+1)) {
                            vTaskDelay(1000/portTICK_PERIOD_MS);
                            ESP_LOGI("LPOS_TIME_SYNC_ERR",
                                    "Restarting the device as time sync fails");
                            fatal_err_handler(TIME_SYNC,TIME_SYNC_FAILS);
                        }
                    }
                } else {
                    ESP_LOGI("LPOS_TIME_SYNC_ERR",
                            "Device restart count: %d reached max restart count",restart_cnt);
                }
            }

            break;
        case waiting_for_time_synced:
            ESP_LOGI("LPOS","lpos_waiting_for_time_synced");
            TimeoutSyncTime();
            break;
        case lpos_data_not_available:
            ESP_LOGI("LPOS","lpos_data_not_available" );
            /* 
             * Once timesync is success, then restart feature
             * should not happen so set as maximum restart 
             * happened (faking)
             */
            if(timesync_updated == 0) {
                update_restart_count(MAX_RESTART_COUNT);
                // Double check to ensure that value is written properly
                readback_val = get_restart_count();
                if(readback_val >= MAX_RESTART_COUNT) {
                    timesync_updated = 1;
                    ESP_LOGI("LPOS","Succesfully updated as maximum restart count reached");
                } else {
                    ESP_LOGI("LPOS","Failed to update as maximum restart count reached");
                    timesync_updated = 0;
                }
            }
            checkDataAvailable();
            break;
        case lpos_data_available:
            ESP_LOGI("LPOS","lpos_data_available" );
            connectingToAP();
            break;
        case lpos_link_not_connected:
            ESP_LOGI("LPOS","lpos_link_not_connected" );
            trySwitchingAP();
            break;
        case lpos_link_connected:
            ESP_LOGI("LPOS","lpos_link_connected" );
            waitConnectingToAP();
            break;
        case lpos_waiting_for_response:
            ESP_LOGI("LPOS","lpos_waiting_for_response" );
            waitingForServerResponse();
            break;
        case lpos_response_received:
            ESP_LOGI("LPOS","lpos_response_received" );
            onResponseFromServer();
            break;
        case lpos_communication_failed:
            ESP_LOGI("LPOS","lpos_communication_failed" );
            onCommunicationFailed();
            break;
    }
}

static esp_err_t validate_image_header(esp_app_desc_t *new_app_info)
{
    if (new_app_info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_app_desc_t running_app_info;
    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
        ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
    }
     return ESP_OK;
}


static esp_err_t _http_client_init_cb(esp_http_client_handle_t http_client)
{
    esp_err_t err = ESP_OK;
    /* Uncomment to add custom headers to HTTP request */
    // err = esp_http_client_set_header(http_client, "Custom-Header", "Value");
    return err;
}

void perform_ota(void *params){
  extern uint8_t https_password[SERV_IP_LEN];
  char url[2048];
  char ota_api[2048];
  memset(&ota_api,0,sizeof(ota_api));
  memset(&url,0,sizeof(url));
  esp_err_t ota_finish_err = ESP_OK;
  while (true) {
    read_domain(url);
    xSemaphoreTake(otaSemaphore, portMAX_DELAY);
    read_spiffs((char*)ota_api,ota_url_update);
    if(strncmp(ota_api,DYNAMIC_OTA_URL_KEY,DYNAMIC_OTA_URL_KEY_LEN) == 0) {
        strcat(url,ota_api);
    } else {
        strcat(url,"/api/getOta/");
    }
    ESP_LOGI(TAG, "Invoking OTA");
    ESP_LOGI(TAG, "\n[%ld], OTA: URL: %s \n",time(NULL),url);
    esp_http_client_config_t clientConfig = {
      .url = url,
      .event_handler = client_event_handler,
      .cert_pem = (char *)server_cert_pem_start,
      .username = (char *)dev_id,
      .password = (char *) https_password,
      .auth_type = HTTP_AUTH_TYPE_BASIC,
      .timeout_ms = 60000,
      .keep_alive_enable = true,
    };

    esp_https_ota_config_t ota_config = {
        .http_config = &clientConfig,
        .http_client_init_cb = _http_client_init_cb, // Register a callback to be invoked after esp_http_client is initialized
    };

    esp_https_ota_handle_t https_ota_handle = NULL;
    esp_err_t err = esp_https_ota_begin(&ota_config, &https_ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "\n [%ld] OTA: ESP HTTPS OTA Begin failed: %s\n",time(NULL), esp_err_to_name(err));
        goto ota_end;
    }

    esp_app_desc_t app_desc;
    err = esp_https_ota_get_img_desc(https_ota_handle, &app_desc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "\n [%ld] OTA: esp_https_ota_read_img_desc failed: %s\n",time(NULL),esp_err_to_name(err));
        goto ota_end;
    }
    err = validate_image_header(&app_desc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "\n [%ld] OTA: image header verification failed: %s\n",time(NULL),esp_err_to_name(err));
        goto ota_end;
    }

    while (1) {
        err = esp_https_ota_perform(https_ota_handle);
        if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
            ESP_LOGI(TAG, "\n [%ld] OTA: esp_https_ota_perform: %s\n",time(NULL),esp_err_to_name(err));
            break;
        }
        // esp_https_ota_perform returns after every read operation which gives user the ability to
        // monitor the status of OTA upgrade by calling esp_https_ota_get_image_len_read, which gives length of image
        // data read so far.
        ESP_LOGD(TAG, "OTA: Image bytes read: %d", esp_https_ota_get_image_len_read(https_ota_handle));
    }

    if (esp_https_ota_is_complete_data_received(https_ota_handle) != true) {
        // the OTA image was not completely received and user can customise the response to this situation.
        ESP_LOGE(TAG, "[%ld] OTA: Complete data was not received.",time(NULL));
        goto ota_end;
    } else {
        ota_finish_err = esp_https_ota_finish(https_ota_handle);
        if ((err == ESP_OK) && (ota_finish_err == ESP_OK)) {
            ESP_LOGI(TAG, "[%ld] OTA: ESP_HTTPS_OTA upgrade successful. Rebooting ...",time(NULL));
            set_algo_status(1);
            watchdog_timer_stop=1;
            vTaskDelay(pdMS_TO_TICKS(1000));
            switch_state_4_5();
            //esp_clk_slowclk_cal_set(0); // HACK TO SOLVE RTC TIME JUMP ISSUE
            fatal_err_handler(OTA_UPDATE,OTA_UPDATE_DONE);
        } else {
            if (ota_finish_err == ESP_ERR_OTA_VALIDATE_FAILED) {
                ESP_LOGE(TAG, "[%ld] OTA: Image validation failed, image is corrupted\n",time(NULL));
            }
            ESP_LOGE(TAG, "[%ld] OTA: ESP_HTTPS_OTA upgrade failed 0x%x [%s]", time(NULL),
                        ota_finish_err,esp_err_to_name(ota_finish_err));
            goto ota_end;
        }
    }
    ota_end:
    esp_https_ota_abort(https_ota_handle);
    ESP_LOGE(TAG, "[%ld] OTA: ESP_HTTPS_OTA upgrade failed",time(NULL));
    stop_lptask = 0;
    fw_update = 1;
  } 
}

void perform_factory_reset(void *params){
  while(1){
    xSemaphoreTake(factoryReset, portMAX_DELAY);
    extern void fcn_factory_reset(void);
    fcn_factory_reset();
  }
}

void perform_system_reset(void *params){
  while(1){
    xSemaphoreTake(serverReset,portMAX_DELAY);
    i2c_eeprom_write_buff(0xFE7F,(uint8_t *) &common_flag,2);
    printf("restarting in 5 seconds\n");
    set_algo_status(1);
    watchdog_timer_stop=1;
    vTaskDelay(pdMS_TO_TICKS(5000));
    switch_state_4_5();
    fatal_err_handler(SERVER_RESET,RST_SERVER_RESET);
  }
}

void watchdogTask(void){
  while(1){

    if(watchdog_timer_stop==0){
      uint32_t result = xEventGroupWaitBits(task_watchdog, allEvents, pdTRUE, pdTRUE, 10000/portTICK_RATE_MS);
      if((result&allEvents)==allEvents){
        ESP_LOGI("Inside watchdogTask","System healthy");
      }
      else{

        ESP_LOGI(TAG,"Watchdog Timer Expired");
        switch_state_4_5();

        if(!(result&overvoltageProtectionTaskEvent)){
          ESP_LOGI("Inside watchdogTask","overvoltage protection task not responding");
        }

        if(!(result&stateMachineTaskEvent)){
          ESP_LOGI("Inside watchdogTask","state machine task not responding");
        }

        if(!(result&dataLoggingTaskEvent)){
          ESP_LOGI("Inside watchdogTask","data logging task not responding");
        }
        fatal_err_handler(WATCHDOG,WD_EXPIRED);
      }
    }
    vTaskDelay(1000/portTICK_RATE_MS);
  }
}

void createTasks(void){
  xTaskCreate(&buttonPressDetection_routine,"Button press",4096,NULL,5,NULL);
  xTaskCreate(&buttonPressTask,"Button press confirmed",4096,NULL,5,NULL);
  xTaskCreate(&dataLoggingTask,"dataLogging",4096,NULL,1,NULL);
  xTaskCreate(&ledBlinkTask,"ledBlink",1024,NULL,3,NULL);
  xTaskCreate(&overvoltageProtectionTask,"overvoltageProtection",4096,NULL,4,NULL);
  xTaskCreate(&stateMachineTask,"stateMachineAlgorithm",4096,NULL,2,NULL);
  xSemaphoreGive(i2cSemaphore);
  xTaskCreate(&mainTimerTask,"mainTimer",1024,NULL,5,NULL);

  if(test_error(ERR_NO_MEMORY)){
    ESP_LOGI(TAG,"EEPROM not detected");
    blink(led_red_color,LED_OFF);
  }
  else{
    least_priority_operations_parameters.status=time_not_synced;
    https_req_buff_write(&https_request_buffer_body,update_configuration);
    watchdog_timer_stop=0;
    xTaskCreate(&OnConnected, "data logging", 5*4096, NULL, 5, NULL);
    xTaskCreate(&test_wifi_config,"Test configuration",4096,NULL,5,NULL);
    xTaskCreate(&perform_ota, "perform_ota", 1024 * 8, NULL, 2, NULL);
    xTaskCreate(&perform_factory_reset, "perform_fac_reset", 4096, NULL, 2, NULL);
    xTaskCreate(&perform_system_reset, "perform_system_reset", 4096 , NULL, 2, NULL);
  }
}
