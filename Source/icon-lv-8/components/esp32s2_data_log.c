#include "esp32s2_data_log.h"
#include "esp32s2_algorithm.h"
#include "esp32s2_fatal_err.h"
#include "esp32s2_https_protocol.h"

#if ((LOG_END_ADDR > 60000))
#error "Memory overflow"
#endif
logging_struct_t log_struct;
uint8_t  flush_idx = 0;

#define TAG "inside data log"
#define MAX_EEPROM_READ_ERR 60
#define MAX_EEPROM_READ_ERR_2 30
#define MAX_EEPROM_WRITE_ERR 30

uint16_t get_current_log_tail()
{
    return (log_struct.log_tail);
}

/*
 * @Info:   Flush the data in the ring buffer
 * @Param:  None
 * @Return: None
 */
void turtle_log_flush(){
	log_struct.log_head = LOG_START_ADDR;
	log_struct.log_tail = log_struct.log_head;
	log_struct.log_available = 0;
}

/*
 * @Info: 	Restore the tail of EEPROM after reset
 * @Param:	None
 * @Return: None
 */
void 
turtle_log_idx_restore(){
    static uint8_t eeprom_read_err = 0;
    if (i2c_eeprom_read_int32(VALID_IDX_ADDR) == VALID_IDX_VALUE) {
        esp_err_t errorCode = i2c_eeprom_read_buff(EE_LOG_IDX_START_ADDR, (uint8_t *) & log_struct, sizeof (log_struct));
        if(errorCode != ESP_OK) {
            ESP_LOGE("ERROR","EEPROM READ ERROR : %s",esp_err_to_name(errorCode));
            eeprom_read_err++;
            if(eeprom_read_err >= MAX_EEPROM_READ_ERR_2) {
                ESP_LOGI("turtle_log_idx_restore","Calling fatal error handler,since EEPROM READ ERR");
                fatal_err_handler(I2C_MODULE,EEPROM_READ_ERR);
            }
        } else {
            eeprom_read_err = 0;
        }
        flush_idx = 1;
        if (((log_struct.log_head % SUMMARY_LEN) != 0) || ((log_struct.log_tail % SUMMARY_LEN) != 0)) {
            log_struct.log_head = LOG_START_ADDR;
            log_struct.log_tail = log_struct.log_head;
            log_struct.log_available = 0;
            flush_idx = 0;
        }
    } 
    else {
        log_struct.log_head = LOG_START_ADDR;
        log_struct.log_tail = log_struct.log_head;
        log_struct.log_available = 0;
        flush_idx = 0;
    }
}

/*
 * @Info:   Save log head and tail before reset
 * @Param:  None
 * @Return: None
 */
void turtle_log_idx_save(){
    i2c_eeprom_write_buff(EE_LOG_IDX_START_ADDR, (uint8_t *) & log_struct, sizeof (log_struct));
    vTaskDelay(5);
    i2c_eeprom_write_int32(VALID_IDX_ADDR, VALID_IDX_VALUE);
}

/*
 * @Info:   Flush saved log head and tail before reset
 * @Param:  None
 * @Return: None
 */
void turtle_log_idx_flush(){
    vTaskDelay(5);
    i2c_eeprom_write_int32(VALID_IDX_ADDR, 0);
    flush_idx = 0;
}

/*
 * @Info:   Write data into eeprom
 * @Param:  Data buffer
 * @Return: None
 */
void turtle_log_put(uint8_t *log_in_buff){
    static uint8_t eeprom_write_err = 0;
	uint16_t _log_head_offset = 0, _log_tail_offset = 0;
	_log_head_offset = (log_struct.log_head + SUMMARY_LEN) % LOG_END_ADDR;
	if(_log_head_offset == log_struct.log_tail){
		_log_tail_offset = (log_struct.log_tail + SUMMARY_LEN) % LOG_END_ADDR;
		log_struct.log_tail = _log_tail_offset;
	}
	else{
		log_struct.log_available++;
	}
	esp_err_t errorCode = i2c_eeprom_write_buff(log_struct.log_head, log_in_buff, SUMMARY_LEN);
    if(errorCode != ESP_OK) {
        ESP_LOGE("ERROR","EEPROM WRITE ERROR : %s",esp_err_to_name(errorCode));
        eeprom_write_err++;
        vTaskDelay(20/portTICK_PERIOD_MS);
        if(eeprom_write_err >= MAX_EEPROM_WRITE_ERR) {
            ESP_LOGI("turtle_log_put","Calling fatal error handler,since max EEPROM write error");
            fatal_err_handler(I2C_MODULE,EEPROM_WRITE_ERR);
        }
    } else {
        eeprom_write_err = 0;
        ESP_LOGI("turtle_log_update","Added [%d] number of entry",log_struct.log_available);
        log_struct.log_head = _log_head_offset;
    }
}

/*
 * @Info:   Retrive data from EEPROM
 * @Param:  Data and packet count
 * @Retrun: None
 */
void turtle_log_get(uint8_t *log_out_buff,uint8_t pos){
	uint16_t _log_tail_offset, record_count, record_idx = 0;
	_log_tail_offset = log_struct.log_tail+(pos*SUMMARY_LEN);

    uint16_t log_count = 0;
    uint8_t reset_count = 0;
    static uint8_t eeprom_read_err = 0;

    while(if_i2c_bus_is_busy() ) {
        vTaskDelay(10/portTICK_RATE_MS);
        log_count++;
        if(log_count >= 500) { // 5 seconds
            ESP_LOGI(TAG,"[%s] I2C bus busy",__func__);
            log_count = 0;
            reset_count++;
        }
        if(reset_count >= SYS_RESET_COUNT) {
            ESP_LOGI("turtle_log_get","Calling fatal error handler,since i2c bus is busy");
            fatal_err_handler(I2C_MODULE,I2C_BUS_IS_BUSY);
        }
    };

    mark_as_i2c_in_use();

	for(record_count = 0; record_count < RECORDS_PER_PACKET; record_count++){
		ESP_LOGI("turtle_log_get","Reading from EEPROM at address [%d] ",_log_tail_offset);
        esp_err_t errorCode = i2c_eeprom_read_buff(_log_tail_offset, &log_out_buff[record_idx], SUMMARY_LEN); 
        if(errorCode != ESP_OK) {
            ESP_LOGE("ERROR","EEPROM READ ERROR : %s",esp_err_to_name(errorCode));
            eeprom_read_err++;
            record_count--;
            vTaskDelay(100/portTICK_PERIOD_MS);
            if(eeprom_read_err >= MAX_EEPROM_READ_ERR) {
                ESP_LOGI("turtle_log_get","Calling fatal error handler,since EEPROM READ ERR");
                fatal_err_handler(I2C_MODULE,EEPROM_READ_ERR);
            }
        } else {
            eeprom_read_err = 0;
            _log_tail_offset = (_log_tail_offset + SUMMARY_LEN) % LOG_END_ADDR;
            record_idx += SUMMARY_LEN;
        }
	}
    mark_as_i2c_free();
}

/*
 * @@Info:  Update the tail of data which has been sent to server
 * @Param:  None
 * @Return: None
 */
void turtle_log_update_tail(){
	uint8_t packet_length = get_packet_length();
	uint16_t _log_tail_offset;
	if(log_struct.log_head != log_struct.log_tail){
		_log_tail_offset = (log_struct.log_tail + packet_length*(SUMMARY_LEN * RECORDS_PER_PACKET)) % LOG_END_ADDR;
		log_struct.log_tail = _log_tail_offset;
        
        if(log_struct.log_tail == 0) {
            ESP_LOGI("DEBUG_DATA_LOSS","UPDATING LOG TAIL BACK TO STARTING");
        }
		
        if(log_struct.log_available >= packet_length*RECORDS_PER_PACKET){
			log_struct.log_available -= packet_length*RECORDS_PER_PACKET;
		}
	}	
}

/*
 * @Info:   Returns the number of data packets available in the memory
 * @Param:  None
 * @Return: None
 */
uint16_t turtle_log_available(){
	return (log_struct.log_available);
}

/*
 * @Info:   Log the algorithm data to the EEPROM
 * @Param:  None
 * @Return: None 
 */
void log_to_eeprom(){
	check_time();
	//ESP_LOGI(TAG,"Inside log to eeprom function\n");
	algo_param.date_time[0] = date_time->tm_mday;   //Set day
	algo_param.date_time[1] = 1+(date_time->tm_mon);    //Set month
	algo_param.date_time[2] = date_time->tm_year;   //set year
	algo_param.date_time[3] = date_time->tm_hour;   //Set Hour
	algo_param.date_time[4] = date_time->tm_min;    //Set minute	
	algo_param.date_time[5] = date_time->tm_sec;
	//ESP_LOGI(TAG,"Updated time\n");

    uint16_t log_count = 0;
    uint8_t reset_count = 0;

    while(if_i2c_bus_is_busy() ) {
        vTaskDelay(10/portTICK_RATE_MS);
        log_count++;
        if(log_count >= 500) { // 5 seconds
            ESP_LOGI(TAG,"[%s] I2C bus busy",__func__);
            log_count = 0;
            reset_count++;
        }
        if(reset_count >= SYS_RESET_COUNT) {
            ESP_LOGI("log_to_eeprom","Calling fatal error handler,since i2c bus is busy");
            fatal_err_handler(I2C_MODULE,I2C_BUS_IS_BUSY);
        }
    };

    mark_as_i2c_in_use();
    ESP_LOGI(TAG,"Logging algo param into EEPROM");
	turtle_log_put((uint8_t *)&algo_param);
    mark_as_i2c_free();
}
