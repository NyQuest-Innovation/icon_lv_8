/*
  ESP-IDF VERSION :- 4.3.1
 */
#include <stdio.h>
#include "esp32s2_task.h"
#include "esp32s2_socket.h"
#include "esp_spiffs.h"
#include "esp32s2_buzzer.h"
#include "esp32s2_parse_data.h"
#include "esp_ota_ops.h" 
#include "esp_partition.h"
#include "esp32s2_pac1952.h"
#include "esp32s2_fatal_err.h"

uint8_t server_cert_pem_start[2048];

#define TAG "inside main"

void fcn_factory_reset(void){
  esp_partition_iterator_t pi = esp_partition_find(ESP_PARTITION_TYPE_APP,ESP_PARTITION_SUBTYPE_APP_FACTORY ,"factory");
  if( pi != NULL ){
    const esp_partition_t * factory = esp_partition_get ( pi );
    esp_partition_iterator_release ( pi );
    if(esp_ota_set_boot_partition(factory)==ESP_OK) {
        fatal_err_handler(FACTORY,FACTORY_RESET);
    }
  }  
}

void init_spiffs(void){
  esp_vfs_spiffs_conf_t conf = {
    .base_path = "/spiffs",.partition_label = NULL,
    .max_files = 5,.format_if_mount_failed = true
  };
  esp_err_t ret = esp_vfs_spiffs_register(&conf);
  if (ret!=ESP_OK){
    if (ret == ESP_FAIL){ ESP_LOGE(TAG, "Failed to mount or format filesystem"); } 
    else if(ret == ESP_ERR_NOT_FOUND){ ESP_LOGE(TAG, "Failed to find SPIFFS partition"); }
    else{ ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret)); }
    return;
  }
  size_t total = 0, used = 0;
  ret = esp_spiffs_info(NULL, &total, &used);
  if(ret != ESP_OK){ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));} 
  else{ ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used); }
}

char urls[11][32] = { 
  "/spiffs/getReadWrite.txt", 
  "/spiffs/updateCalib.txt", 
  "/spiffs/updateConfig.txt",
  "/spiffs/updateThreshold.txt",
  "/spiffs/updateStatus.txt",
  "/spiffs/updateSoC.txt",
  "/spiffs/updateLog.txt",
  "/spiffs/getCert.txt",
  "/spiffs/google.cer",
  "/spiffs/updateOTA.txt",
  "/spiffs/domain.txt"
};

void read_domain(char *ptr){
  bzero(ptr,2048);
  ESP_LOGI(TAG, "reading from domain.txt");
  FILE* f = fopen("/spiffs/domain.txt", "r");
  fseek(f, 0, SEEK_END);
  long fsize = ftell(f);
  fseek(f, 0, SEEK_SET);  /* same as rewind(f); */
  fread(ptr, 1, fsize, f);
  fclose(f);
  ptr[fsize] = 0;
  ESP_LOGI(TAG,"Domain name is: [%s]",ptr);
  #warning "Delete following line after server fix"
  bzero(ptr,2048);
  memcpy(ptr,"https://log.nyquestapi.com",strlen("https://log.nyquestapi.com"));
  //memcpy(ptr,"http://logenergy24by7.optiologic.com/",strlen("http://logenergy24by7.optiologic.com/")); // Based on Ameer's mail on March 28, 2024 preparing a test version with new link
  ESP_LOGI(TAG,"Domain name is\n [%s]",ptr);
}

void read_spiffs(char *ptr, int type ){
  bzero(ptr,2048);
  ESP_LOGI(TAG, "reading from %s",urls[type]);
  FILE* f = fopen(urls[type], "r");
  fseek(f, 0, SEEK_END);
  long fsize = ftell(f);
  fseek(f, 0, SEEK_SET);  /* same as rewind(f); */
  fread(ptr, 1, fsize, f);
  fclose(f);
  ptr[fsize] = 0;
//  ESP_LOGI(TAG,"Data read is \n %s",ptr);
}

void update_spiffs( char *param, int type ){
  ESP_LOGI(TAG, "updating from %s",urls[type]);
  remove(urls[type]);
  FILE* f = fopen(urls[type], "w");
  if(f==NULL){ESP_LOGE(TAG, "Failed to open %s",urls[type]);return;}
  fprintf(f,"%s",param);
  fclose(f);
  if(type==update_certificate){ read_spiffs((char *)server_cert_pem_start,read_certificate); };
}

int get_reset_reason_code(uint8_t code,char *reason)
{
    int ret=0;
    switch(code)
    {
        case ESP_RST_UNKNOWN:strcpy(reason,"RST_REASON_UNKNOWN");
                        break;
        case ESP_RST_POWERON:strcpy(reason,"RST_REASON_POWERON");
                        break;
        case ESP_RST_EXT:strcpy(reason,"RST_REASON_EXT");
                        break;
        case ESP_RST_SW:strcpy(reason,"RST_REASON_SW");
                        break;
        case ESP_RST_PANIC:strcpy(reason,"RST_REASON_PANIC");
                        break;
        case ESP_RST_INT_WDT:strcpy(reason,"RST_REASON_INT_WDT");
                        break;
        case ESP_RST_TASK_WDT:strcpy(reason,"RST_REASON_TASK_WDT");
                        break;
        case ESP_RST_WDT:strcpy(reason,"RST_REASON_WDT");
                        break;
        case ESP_RST_DEEPSLEEP:strcpy(reason,"RST_REASON_DEEPSLEEP");
                        break;
        case ESP_RST_BROWNOUT:strcpy(reason,"RST_REASON_BROWNOUT");
                        break;
        case ESP_RST_SDIO:strcpy(reason,"RST_REASON_SDIO");
                        break;
        default:strcpy(reason,"RST_REASON_UNKNOWN");
                        break;
    }
    return ret;
}
void app_main(void){
    int ret = 0;
    char reboot_reason_buffer[64];
    char soft_reset_reason[20];
    init_spiffs();
    memset(&reboot_reason_buffer,0,sizeof(reboot_reason_buffer));
    memset(&soft_reset_reason,0,sizeof(soft_reset_reason));
    ESP_LOGI(TAG,"Hardware version : %x ; Software version : %x",HW_VERSION,FW_VERSION);
    esp_reset_reason_t reset_code = esp_reset_reason();
    get_reset_reason_code(reset_code,reboot_reason_buffer);
    ESP_LOGI("RESET REASON", "%s [%d]",reboot_reason_buffer,reset_code);
    if(reset_code == 3) { // Soft reboot
        ret = get_soft_reset_reason(soft_reset_reason);
        if(ret == 0) {
            ESP_LOGI("RESET REASON","SOFT REBOOT WAS CAUSED BY %s ",soft_reset_reason);
        }
    }
    set_timezone();
    peripheral_initializaton();
    watchdogTask();
}

void peripheral_initializaton(void){
  #if ICUBE
  #warning "Device configured as iCUBE"
  #else
  #warning "Device configured as iCON"
  #endif  
  createSemaphores();
  init_buzzer();
  init_led();         
  init_i2c();        
  init_switches();    
#if ICUBE
  S4_SW_init();
#endif 
  init_pushbutton();
  init_adc();         
  init_mains_sense(); 
  initializeBatteryOvervoltageSense();
  init_pac1952();   
  check_time();
  read_spiffs((char *)server_cert_pem_start,read_certificate);
  power_on_initialization();
  esp32s2_WiFi_Init();
  createTasks();
}

void init_data(void){
  algo_param.cur_bat_v           = 13.1;
  algo_param.cur_chg_i           = 5.0;
  algo_param.cur_dis_i           = 0.0;
  algo_param.cur_sol_v           = 13.1;
  algo_param.cur_sol_i           = 5.0;
  algo_param.cur_ac_load_i       = 3.0;
  algo_param.cur_temp            = 35;
  algo_param.bat_cur_cap_coul    = 93.0;
  algo_param.dev_algo_state      = 3;
  algo_param.sol_e               = 12;
  algo_param.sol_bat_e           = 12;
  algo_param.bat_dis_e           = 12;
  algo_param.bat_ac_chg_e        = 12;  
  algo_param.ac_load_e           = 12;
  algo_param.num_sum_samples     = 1800;
  algo_param.log_type            = 3;
  algo_param.state_change_reason = 0;
  algo_param.dev_stat            = 48;
  algo_param.algo_stat           = 14336;
  algo_param.cur_inv_v           = 14.2;
  algo_param.error               = 10;
  algo_param.daystartflag        = 1;
  algo_param.resetflag           = 0;
  algo_param.utility_savings     = 0.25;  
}
