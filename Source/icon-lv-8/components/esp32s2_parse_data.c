#include "esp32s2_parse_data.h"
#include "esp32s2_socket.h"
#include "esp32s2_24lc512.h"
#include "esp32s2_main.h"

#define TAG "parseJSON"

extern https_request_buffer_body_t https_request_buffer_body;
static bool writeInfo_status = false;
static bool writeInfo_req_rxd_from_server = false;

uint16_t write_info_address[98]={
  EE_SERV_IP_ADDR, EE_SERV_PORT_ADDR, EE_AP_SSID_ADDR, EE_AP_PSW_ADDR,    
  EE_DC_OVER_I_THR_ADDR, EE_AC_OVER_I_THR_ADDR,
  EE_BAT_TYPE_ADDR, EE_BAT_MAX_VOLT_ADDR,               		        
  EE_BAT_CAP_ADDR, EE_BAT_AGE_ADDR, 
  EE_BAT_FRC_EXIT_VOLT_ADDR, EE_FT_NUM_ADDR,    
  EE_SOL_CAP,                               
  EE_BAT_MAINS_CHG_IN_VOLT_ADDR, EE_BAT_MAINS_CHG_OUT_VOLT_ADDR,
  EE_MORN_CHG_INH_EN_ADDR, EE_EVEN_CHG_INH_EN_ADDR, EE_ABS_INTR_ADDR, EE_BAT_EQU_INTR_ADDR, EE_BAT_EQU_DUR_ADDR,
  EE_DEV_SER_NO_ADDR, EE_DEVICE_TYPE_ADDR,    
  EE_UC_BAT_V_CONST_ADDR, EE_PAC_BAT_V_CONST_ADDR, 
  EE_PAC_BAT_CHG_I_CONST_ADDR, EE_PAC_BAT_DIS_I_CONST_ADDR, 
  EE_PAC_BAT_CHG_I_OFF_ADDR, EE_PAC_BAT_DIS_I_OFF_ADDR,
  EE_PAC_BAT_P_CONST_ADDR, EE_UC_SOL_V_CONST_ADDR, 
  EE_PAC_SOL_V_CONST_ADDR, EE_PAC_SOL_I_CONST_ADDR, EE_PAC_SOL_I_OFF_ADDR, EE_PAC_SOL_P_CONST_ADDR,
  EE_UC_TEMP1_CONST_ADDR, EE_UC_TEMP2_CONST_ADDR, EE_CT_LOAD_I_OFST_ADDR, EE_CT_LOAD_I_CONST_ADDR,
  DEV_USEABLE_SOC_MULTIPLIER, DEV_EQU_HYS, 
  DEV_ABS_MF_PER, EQ_MF_PER, 
  VFT_EXIT_HIGH, VFT_EXIT_LOW,  
  DEV_SYS_FLAG, DAILY_LOAD_THR,   
  WEEKLY_LOAD_THR, NUM_DAYS,     
  USEABLE_SOC, RES_ENE_MUL, VFT_EXT_THR,            
  EVE_BAT_SOC_COR_LOW_LIM, EVE_BAT_SOC_COR_HIGH_LIM,                   
  SOL_MAX_CUR, USEABLE_SOC_STRT,                    
  FT_ENTRY_STRT_DAY, FT_EXIT_STRT_DAY,                 
  STATE_CHANGE_LOG_UNITS, STATE_CHANGE_LOG_UNITS_NO_WIFI,           
  SOL_E_CORR, SER_ASSERT_STATE, SER_STATE_ASSERTED_TIM_HR, SER_STATE_ASSERTED_TIM_MIN, 
  SER_STATE_ASSERTED_DUR, FT_EXIT_TIM, MORN_SOL_PRED, EVEN_SOL_PRED,
  DAY1_MORN_LOAD, DAY1_NGT_LOAD, 
  DAY2_MORN_LOAD, DAY2_NGT_LOAD,
  DAY3_MORN_LOAD, DAY3_NGT_LOAD, 
  DAY4_MORN_LOAD, DAY4_NGT_LOAD, 
  DAY5_MORN_LOAD, DAY5_NGT_LOAD,                       
  DAY6_MORN_LOAD, DAY6_NGT_LOAD, 
  DAY7_MORN_LOAD, DAY7_NGT_LOAD,
  COMMON_THR, DEV_SOC_VOL_TABLE,
  DAY1_MORN_SOL, DAY1_AFT_SOL,
  DAY2_MORN_SOL, DAY2_AFT_SOL,
  DAY3_MORN_SOL, DAY3_AFT_SOL
};

uint8_t write_info_length[98]={
  SERV_IP_LEN, SERV_PORT_LEN, AP_SSID_LEN, AP_PSW_LEN,
  DC_OVER_I_THR_LEN, AC_OVER_I_THR_LEN,
  EE_BAT_TYPE_LEN, BAT_MAX_VOLT_LEN, 
  EE_BAT_CAP_LEN, EE_BAT_AGE_LEN, 
  BAT_FRC_EXIT_VOLT, EE_FT_NUM_LEN,  
  EE_SOL_CAP_LEN, 
  BAT_MAINS_CHG_IN_VOLT_LEN, BAT_MAINS_CHG_OUT_VOLT_LEN,
  MORN_CHG_INH_EN_LEN, EVEN_CHG_INH_EN_LEN, EE_ABS_INTR_LEN, BAT_EQU_INTR_LEN, BAT_EQU_DUR_LEN,		
  DEV_SER_NO_LEN, DEVICE_TYPE_LEN,          
  UC_BAT_V_CONST_LEN, PAC_BAT_V_CONST_LEN,          
  PAC_BAT_CHG_I_CONST_LEN, PAC_BAT_DIS_I_CONST_LEN,  
  PAC_BAT_CHG_I_OFF_LEN, PAC_BAT_DIS_I_OFF_LEN,   
  PAC_BAT_P_CONST_LEN, UC_SOL_V_CONST_LEN,        
  PAC_SOL_V_CONST_LEN, PAC_SOL_I_CONST_LEN, PAC_SOL_I_OFF_LEN, PAC_SOL_P_CONST_LEN,                 
  UC_TEMP1_CONST_LEN, UC_TEMP2_CONST_LEN, CT_LOAD_I_OFST_LEN, CT_LOAD_I_CONST_LEN,                 
  DEV_USEABLE_SOC_MULTIPLIER_LEN, DEV_EQU_HYS_LEN,                                                     
  DEV_ABS_MF_PER_LEN, EQ_MF_PER_LEN,                         
  VFT_EXIT_HIGH_LEN, VFT_EXIT_LOW_LEN,                                 
  DEV_SYS_FLAG_LEN, DAILY_LOAD_THR_LEN,                                
  WEEKLY_LOAD_THR_LEN, NUM_DAYS_LEN, 
  USEABLE_SOC_LEN, RES_ENE_MUL_LEN, VFT_EXT_THR_LEN, 
  EVE_BAT_SOC_COR_LOW_LIM_LEN, EVE_BAT_SOC_COR_HIGH_LIM_LEN,  
  SOL_MAX_CUR_LEN, USEABLE_SOC_STRT_LEN,  
  FT_ENTRY_STRT_DAY_LEN, FT_EXIT_STRT_DAY_LEN,
  STATE_CHANGE_LOG_UNITS_LEN, STATE_CHANGE_LOG_UNITS_NO_WIFI_LEN, 
  SOL_E_CORR_LEN, SER_ASSERT_STATE_LEN, SER_STATE_ASSERTED_TIM_HR_LEN, SER_STATE_ASSERTED_TIM_MIN_LEN, 
  SER_STATE_ASSERTED_DUR_LEN, FT_EXIT_TIM_LEN, MORN_SOL_PRED_LEN, EVEN_SOL_PRED_LEN,
  DAY1_MORN_LOAD_LEN, DAY1_NGT_LOAD_LEN, 
  DAY2_MORN_LOAD_LEN, DAY2_NGT_LOAD_LEN,  
  DAY3_MORN_LOAD_LEN, DAY3_NGT_LOAD_LEN, 
  DAY4_MORN_LOAD_LEN, DAY4_NGT_LOAD_LEN, 
  DAY5_MORN_LOAD_LEN, DAY5_NGT_LOAD_LEN,       
  DAY6_MORN_LOAD_LEN, DAY6_NGT_LOAD_LEN,                                  
  DAY7_MORN_LOAD_LEN, DAY7_NGT_LOAD_LEN,  
  COMMON_THR_LEN, DEV_SOC_VOL_TABLE_LEN,                         
  DAY1_MORN_SOL_LEN, DAY1_AFT_SOL_LEN,                    
  DAY2_MORN_SOL_LEN, DAY2_AFT_SOL_LEN,                       
  DAY3_MORN_SOL_LEN, DAY3_AFT_SOL_LEN                  
};

void https_req_buff_write(https_request_buffer_body_t *param,uint8_t c){
  uint8_t i; 
  i = (param->https_request_buffer_head + 1) % HTTPS_REQ_BUFFER_SIZE;
  if (i == param->https_request_buffer_tail){
    param->https_request_buffer_tail = (param->https_request_buffer_tail + 1) % HTTPS_REQ_BUFFER_SIZE;
  }
  param->https_request_buffer[param->https_request_buffer_head] = c;
  param->https_request_buffer_head = i;
}

uint8_t https_req_buff_available(https_request_buffer_body_t *param){
	uint8_t buff_size = (HTTPS_REQ_BUFFER_SIZE + param->https_request_buffer_head - param->https_request_buffer_tail) % HTTPS_REQ_BUFFER_SIZE;
	ESP_LOGI(TAG,"Buffer size %d",buff_size);
	return buff_size;
}

uint8_t https_req_buff_peek(https_request_buffer_body_t *param){
	return(param->https_request_buffer[param->https_request_buffer_tail]);
}

void https_req_buff_flush(https_request_buffer_body_t *param){
	if(param->https_request_buffer_head != param->https_request_buffer_tail){
	   param->https_request_buffer_tail = (param->https_request_buffer_tail + 1) % HTTPS_REQ_BUFFER_SIZE;
	}
}

void detect_domain_msg(cJSON *payload){
  cJSON *readInfo = cJSON_GetObjectItem(payload, "domain");
  if(readInfo!=NULL){
    // ESP_LOGI(TAG,"Parse domain msg - %s",readInfo->valuestring);
    update_spiffs(readInfo->valuestring,domain_update);
  }
}

void detect_ota_url(cJSON *payload){
    cJSON *readInfo = cJSON_GetObjectItem(payload, "ota_url");
    if(readInfo!=NULL){
        ESP_LOGI(TAG,"Parse ota_url - %s",readInfo->valuestring);
        update_spiffs(readInfo->valuestring,ota_url_update);
    }
}

void httpsmsgparse(char *https_receive_buffer,https_request_buffer_body_t *param){  
  if( https_req_buff_peek(param) == update_certificate ){
    update_spiffs(https_receive_buffer,update_certificate);
  }
  else{
    cJSON *payload = cJSON_Parse(https_receive_buffer);
    ESP_LOGI(TAG,"Parsing data");
    if(detect_payload(payload)){
      ESP_LOGI(TAG,"Detected device");
      if(detect_msgType(payload)){
        ESP_LOGI(TAG,"Message type matched");
        detect_domain_msg(payload);
        detect_ota_url(payload);
        detect_write_msg(payload);
        detect_read_msg(payload);
      }
    }
    cJSON_Delete(payload);
  }
}

int detect_payload(cJSON *payload){
    cJSON *device_id = cJSON_GetObjectItem(payload, "dev_id");
    if(device_id != NULL){
        extern char dev_id[12];
        if(strcmp(device_id->valuestring,dev_id)==0) {
            return 1;
        } else {
            return 0;
        }
    } else {
        return 0;
    }
}

int detect_msgType(cJSON *payload){
  cJSON *msg_type = cJSON_GetObjectItem(payload,"messagetype");
  if(msg_type!=NULL){
    if(strcmp(msg_type->valuestring,"read write response")==0){return 1;}
    else{return 0;}
  }
  else{return 0;}
}

void detect_write_msg(cJSON *payload){
  cJSON *writeInfo = cJSON_GetObjectItem(payload, "writeInfo");
  if(writeInfo!=NULL){
    parse_write_msg(writeInfo);
  }
}

void detect_read_msg(cJSON *payload){
  cJSON *readInfo = cJSON_GetObjectItem(payload, "readInfo");
  if(readInfo!=NULL){
    parse_read_msg(readInfo);
  }
}


void set_writeInfo_status(bool status)
{
    writeInfo_status = status;
}

bool get_writeInfo_status()
{
    return writeInfo_status;
}

void set_write_req_rxd_from_server(bool status)
{
    writeInfo_req_rxd_from_server = status;
}

bool get_write_req_rxd_from_server()
{
    return writeInfo_req_rxd_from_server;
}

void parse_write_msg(cJSON *writeInfo)
{
    uint8_t updated_eeprom_content = 0;
    uint8_t read_back_val_8 = 0, tmp_uint8_val = 0;
    uint16_t read_back_val_16 = 0, tmp_uint16_val = 0;
    float read_back_val_float = 0.0, tmp_float_val = 0;
    char read_back_string[32] = {0x0,};
    int ret = -1;
    bool eeprom_updation_status = false;

    ESP_LOGI(TAG,"Parsing writeInfo content");
    char temp_addr[5];

    for(int i=0;i<sizeof(write_info_address)/sizeof(uint16_t);i++) {
        sprintf(temp_addr,"%x",write_info_address[i]);
        cJSON *readvalue = cJSON_GetObjectItem(writeInfo,temp_addr);
        if(readvalue!=NULL) {
            // Set a flag indicating a write request from server is received
            set_write_req_rxd_from_server(true);
            updated_eeprom_content = 0;
            if(write_info_length[i] == 1) {
                tmp_uint8_val = atoi(readvalue->valuestring);
                ESP_LOGI(TAG,"8bit int: Writing to address %s and the value is %d",temp_addr,atoi(readvalue->valuestring));
                ee24lc256_write_byte(write_info_address[i],tmp_uint8_val);
                vTaskDelay(100/portTICK_PERIOD_MS);
                updated_eeprom_content = 1;
                // Read back the value and verify
                read_back_val_8 = ee24lc256_read_byte(write_info_address[i]);
                ESP_LOGI(TAG,"8bit int: Reading from address : [%s], Read value : %d, Expected value: %d",temp_addr,read_back_val_8,tmp_uint8_val);
                if(read_back_val_8 == tmp_uint8_val) { // Matches
                    // Mark as eeprom updation was success
                    eeprom_updation_status = true;
                } else { // Doesn't match
                    // Mark as eeprom updation was failure
                    eeprom_updation_status = false;
                }
            } else if(write_info_length[i] == 2) {
                tmp_uint16_val = atoi(readvalue->valuestring);
                ESP_LOGI(TAG,"16bit int: Writing to address %s and the value is %d",temp_addr,tmp_uint16_val);
                ee24lc256_write_int16(write_info_address[i],tmp_uint16_val);
                vTaskDelay(100/portTICK_PERIOD_MS);
                updated_eeprom_content = 1;
                // Read back the value and verify
                read_back_val_16 = ee24lc256_read_int16(write_info_address[i]);
                ESP_LOGI(TAG,"16bit int: Reading from address : [%s], Read value : %d, Expected value: %d",temp_addr,read_back_val_16,tmp_uint16_val);
                if(read_back_val_16 == tmp_uint16_val) { // Matches
                    eeprom_updation_status = true;
                } else { // Doesn't match
                    eeprom_updation_status = false;
                }
            } else if(write_info_length[i] == 4) {
                tmp_float_val = atof(readvalue->valuestring);
                ESP_LOGI(TAG,"Float: Writing to address %s and the value is %f",temp_addr,tmp_float_val);
                ee24lc256_write_float(write_info_address[i],tmp_float_val);
                vTaskDelay(100/portTICK_PERIOD_MS);
                updated_eeprom_content = 1;
                // Read back the value and verify
                read_back_val_float = ee24lc256_read_float(write_info_address[i]);
                ESP_LOGI(TAG,"Float: Reading from address : [%s], Read value : %f, Expected value: %f",temp_addr,read_back_val_float,tmp_float_val);
                if(read_back_val_float == tmp_float_val) { // Matches
                    eeprom_updation_status = true;
                } else { // Doesn't match
                    eeprom_updation_status = false;
                }
            } else if(write_info_length[i] == 32) {
                ESP_LOGI(TAG,"String: Writing to address %s and the value is %s",temp_addr,readvalue->valuestring);
                ee24lc256_write_buff(write_info_address[i],(uint8_t*)readvalue->valuestring,write_info_length[i]);
                vTaskDelay(100/portTICK_PERIOD_MS);
                updated_eeprom_content = 1;
                // Read back the value and verify
                memset(read_back_string,0,sizeof(read_back_string));
                ret = ee24lc256_read_buff(write_info_address[i],(uint8_t*)read_back_string,sizeof(read_back_string));
                ESP_LOGI(TAG,"String: Reading from address : [%s], Read value : %s, Expected value: %s",temp_addr,read_back_string,readvalue->valuestring);
                if(ret == 0) { // Read is success
                    if(strcmp(read_back_string,readvalue->valuestring) == 0) { // Matches
                        eeprom_updation_status = true;
                    } else { // Doesn't match
                        eeprom_updation_status = false;
                    }
                } else { // Read fails, hence mark flag as EEPROM updation as failure
                    eeprom_updation_status = false;
                }

            } else { // Any address location of which length is not in the list comes, // No chance to reach here
                ESP_LOGI(TAG,"Support to handle EEPROM write at %04X with value %s of length %d is not available",write_info_address[i],readvalue->valuestring,write_info_length[i]);
                eeprom_updation_status = false;
            }


            if(updated_eeprom_content == 1) {
                if (write_info_address[i] >= EE_CONFIG_DAT_START_ADDR && write_info_address[i] < EE_CONFIG_DONE_ADDR) {
                    ESP_LOGI("SERVER UPDATE", "UPDATING CRC INTO EE_CONFIG_CRC_ADDR");
                    calc_update_eeprom_crc(EE_CONFIG_DAT_START_ADDR, CONFIG_PARAM_LEN, EE_CONFIG_CRC_ADDR, 1);
                    ESP_LOGI("SERVER UPDATE", "DONE UPDATING CONFIG CRC");
                }

                if (write_info_address[i] >= EE_CALIB_DAT_START_ADDR && write_info_address[i] < EE_CALIB_DONE_ADDR) {
                    ESP_LOGI("SERVER UPDATE", "UPDATING CRC INTO EE_CALIB_CRC_ADDR");
                    calc_update_eeprom_crc(EE_CALIB_DAT_START_ADDR, CALIB_CONST_LEN, EE_CALIB_CRC_ADDR, 1);
                    clear_error(ERR_NOT_CALIB);
                    ESP_LOGI("SERVER UPDATE", "DONE UPDATING CALIB CRC");
                }
            }
            // If EEPROM updation fail, break [Should break only after updating CRC]
            if(eeprom_updation_status == false) {
                break;
            }

        }
    }
    load_wifi_param();
    load_sensor_constants();

    if(get_write_req_rxd_from_server() == true) {
        if(eeprom_updation_status == true) {
            ESP_LOGI("SERVER UPDATE", "UPDATING EEPROM CONTENTS SUCCESS, SENDING FEED BACK AS SUCCESS");
        } else {
            ESP_LOGI("SERVER UPDATE", "UPDATING EEPROM CONTENTS FAILS, SENDING FEED BACK AS FAILURE");
        }
        // Set the flag indicating EEPROM updation was success or failure
        set_writeInfo_status(eeprom_updation_status);
        /* Q to send the response back to server, send the response only 
         * if writeInfo has contents in it with flag indicating
         * the EEPROM write is success or not
         */
        https_req_buff_write(&https_request_buffer_body,send_response);
    }
}

void parse_read_msg(cJSON *readInfo) {
    if(parse_response(readInfo->valueint,calibration_req)) { 
        ESP_LOGI(TAG,"parse read: calibration"); 
        https_req_buff_write(&https_request_buffer_body,update_calibration); 
    };

    if(parse_response(readInfo->valueint,configuration_req)) { 
        ESP_LOGI(TAG,"parse read: configuration"); 
        https_req_buff_write(&https_request_buffer_body,update_configuration); 
    };

    if(parse_response(readInfo->valueint,system_thr_req)) { 
        ESP_LOGI(TAG,"parse read: system threshold"); 
        https_req_buff_write(&https_request_buffer_body,update_system_threshold);  
    };

    if(parse_response(readInfo->valueint,useable_soc_req)) { 
        ESP_LOGI(TAG,"parse read: useable soc"); 
        https_req_buff_write(&https_request_buffer_body,update_useable_soc); 
    };

    if(parse_response(readInfo->valueint,factory_update)) { 
        ESP_LOGI(TAG,"parse read: firmware update"); 
        ESP_LOGI("DEBUG", "OTA UPDATE available indication received from server at [%ld]",time(NULL));
        extern uint8_t fw_update; 
        fw_update=1;
    };
    if(parse_response(readInfo->valueint,factory_reset)) {  
        ESP_LOGI(TAG,"parse read: factory reset"); 
        fcn_factory_reset(); 
    };

    if(parse_response(readInfo->valueint,reboot_req_from_server)) {  
        ESP_LOGI(TAG,"parse read: Reboot request from server received");
        post_common_flag(CF_RESET_DEV);
    };

}
