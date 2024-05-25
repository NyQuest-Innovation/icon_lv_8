#include "esp32s2_wifi.h"
#include "esp32s2_socket.h"
#include "esp32s2_algorithm.h"

uint8_t https_password[SERV_IP_LEN];
uint8_t ap_ssid_buff[AP_SSID_LEN];
uint8_t ap_psw_buff[AP_PSW_LEN];
uint16_t server_port_address;
extern uint8_t wait_time_sync;

#define SSID "test"
#define PASSWORD "01123581321"

char *TAG = "WiFI CONNECTION";

char *SNTP_SERVER = "pool.ntp.org";

time_t current_time = 0;

uint8_t test_connect_cnt=0,connection_status_obtained=0;

extern bool wifi_connection_status;

/*
 * @Info: Handler for WiFi; 
 *        Depending on the action by WiFi the following switch case statement will be executed
 * @Param: system_event handler and void pointer
 * @Return: None
 */
static esp_err_t event_handler(void *ctx, system_event_t *event){
  switch (event->event_id){
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect(); //Connect to WiFi AP and PSW when wifi is started
        ESP_LOGI(TAG, "connecting...\n");
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        ESP_LOGI(TAG, "connected\n"); //When ESP connected to AP and PSW 
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "got ip\n");    //When ESP is assigned a IP
        extern uint8_t connection_stat_check,connection_stat;
        if(connection_stat_check){
          connection_stat=1;
          xSemaphoreGive(logdataSemaphore); //The following code is used to test connectivity with server during mobile app interaction
        }
        initRTC(); //Initialize RTC when connected to a AP
        wifi_connection_status = true;
        xSemaphoreGive(connectionSemaphore); //This is to start the socket client when the device is set in calibration mode
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        ESP_LOGI(TAG, "Disconnected\n"); //When ESP disconnected to AP
        /* Retry connecting to hotspot in during test connectivity */
        wifi_connection_status = false;
        if(connection_stat_check) {
            test_connect_cnt++;
          if(test_connect_cnt > 6) {
            connection_stat_check=0;
            connection_stat=0;
            connection_status_obtained=1;
            blink(led_red_color,FULL_ON);
            vTaskDelay(5000/portTICK_PERIOD_MS);
            set_algo_status(0);
          } else if( test_connect_cnt < 3) {
              // Try 3 times to connect to the configured Wi-Fi
              esp_wifi_stop();
              connectSTA( (char *) ap_ssid_buff , (char *) ap_psw_buff );
          } else { // Try connecting to Energy24 Hotspot
            esp_wifi_stop();
            connectSTA((char *)"E24by7",(char *)"Energy24by7");
          }
        }
        break;
    case SYSTEM_EVENT_AP_STACONNECTED:
         ESP_LOGI(TAG,"Station connected to ESP32 AP");
         break;
    case SYSTEM_EVENT_AP_STAIPASSIGNED:
         /* To start socket server when device set in configuration mode */
         ESP_LOGI(TAG,"IP assigned to connected station");
         xSemaphoreGive(serverSemaphore);
         break;
    default:
        break;
  }
  return ESP_OK;
}

/*
 * @Info:  Connecting to WiFi AP
 * @Param: SSID and PASSWORD to connect 
 * @Return: None
 */
void connectSTA(char *ssid, char *pass){
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  wifi_config_t wifi_config;
  memset(&wifi_config, 0, sizeof(wifi_config));
  strcpy((char *)wifi_config.sta.ssid, ssid);
  strcpy((char *)wifi_config.sta.password, pass);
  ESP_LOGI(TAG,"AP SSID is %s",(char *)wifi_config.sta.ssid);
  ESP_LOGI(TAG,"AP PSW is %s",(char *)wifi_config.sta.password);
  esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
  ESP_ERROR_CHECK(esp_wifi_start());  
}

/*
 * @Info:   Create an AP with specified SSID and PASSWORD 
 * @Param:  None   
 * @Return: None
 */
void connectAP(){
    char ssid_str[32];
    char password[32];
    start_dhcp_server();
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    memset(&ssid_str,0,sizeof(ssid_str));
    memset(&password,0,sizeof(password));

#if ICUBE
    sprintf(ssid_str,"iCUBE-L%X",FW_VERSION);
    strcpy(password,"icube1234");
#else
    sprintf(ssid_str,"iCON-L%X",FW_VERSION);
    strcpy(password,"icon1234");
#endif
    wifi_config_t wifi_config;
    esp_wifi_get_config(ESP_IF_WIFI_AP,&wifi_config);
    memcpy(wifi_config.ap.ssid,ssid_str,sizeof(ssid_str));
    memcpy(wifi_config.ap.password,password,sizeof(password));
    wifi_config.ap.max_connection = 1;
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;

    esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config);
    ESP_ERROR_CHECK(esp_wifi_start());  
}

/*
 * @Info:   Initialize the WiFi module of ESP
 * @Param:  None
 * @Return: None
 */
void esp32s2_WiFi_Init(void){
  ESP_ERROR_CHECK(nvs_flash_init());
  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
  wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
}

/*
 * @Info:   Print log time
 * @Param:  Time and message
 * @Return: None
 */
void print_time(long time, const char *message){
  struct tm *timeinfo = localtime(&time);
  printf("Time is  %dH:%dM:%dS \n", timeinfo->tm_hour,timeinfo->tm_min,timeinfo->tm_sec);
}

/* 
 * @Info:   Callback for time_sync_notification
 * @Param:  time structure
 * @Return: None
 */
void SyncTime_callback(struct timeval *tv){

    set_rtc_sync_status(1);
    ESP_LOGI(TAG,"Time synced\n");
    wait_time_sync=0;
    extern operations_parameters_t least_priority_operations_parameters;
    least_priority_operations_parameters.status=lpos_data_not_available;

    if (SNTP_SYNC_STATUS_COMPLETED == sntp_get_sync_status()) {
        ESP_LOGI(TAG, "Time sync status: SNTP_SYNC_STATUS_COMPLETED\n");
    } else if (SNTP_SYNC_STATUS_RESET == sntp_get_sync_status()) {
        ESP_LOGI(TAG, "Time sync status: SNTP_SYNC_STATUS_RESET\n");
    } else if (SNTP_SYNC_STATUS_IN_PROGRESS == sntp_get_sync_status()) {
        ESP_LOGI(TAG, "Time sync status: SNTP_SYNC_STATUS_IN_PROGRESS\n");
    }

    setenv("TZ", "UTC-5:30", 1);
    tzset();

    // getRTCtime();
}

/*
 * @Info:   Get RTC time
 * @Param:  None
 * @Return: None
 */
void getRTCtime(){
    time(&current_time);
    ESP_LOGI("Time","Time is %ld",current_time);
    print_time(current_time,"time");
}

/*
 * @Info:   Initialize the RTC module 
 * @Param:  None
 * @Return: None
 */
void initRTC(void){

  ESP_LOGI(TAG, "Initializing RTC (%s)", SNTP_SERVER);

  //--Azad ES--//
  sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
  sntp_setservername(0, SNTP_SERVER); //--Line 18: char *SNTP_SERVER = "pool.ntp.org";
  sntp_set_time_sync_notification_cb(SyncTime_callback);
  sntp_init();
  //-- --//
}

/*
 * @Info:   Stop DHCP server
 * @Param:  None
 * @Return: None
 */
void stop_dhcp_server(){
  esp_err_t status;
  status = tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP);
  ESP_LOGI(TAG,"Error value is %d",status);
}

/*
 * @Info: Establish a DHCP server for the mobile app to connect
 */
void start_dhcp_server(){
  // stop DHCP server
  tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP);
  // assign a static IP to the network interface
  tcpip_adapter_ip_info_t info;
  memset(&info, 0, sizeof(info));
  IP4_ADDR(&info.ip, 192, 168, 4, 1);
  IP4_ADDR(&info.gw, 192, 168, 4, 1);//ESP acts as router, so gw addr will be its own addr
  IP4_ADDR(&info.netmask, 255, 255, 255, 0);
  ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &info));
  // start the DHCP server   
  ESP_ERROR_CHECK(tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP));
  printf("DHCP server started \n");
}
