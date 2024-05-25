#include "esp32s2_adc.h"
#include "esp32s2_pac1952.h"
#include "esp32s2_fatal_err.h"

#define TAG "ADC"
#define MAX_DEBUG_LOG_COUNT 300
#define PAC_ERR_MAX_COUNT 20
#define PAC_BUSY_MAX_CNT 10

#define THRESHOLD_CHECK 1 // To clip off invalid range of values for adc and PAC 
uint8_t i2cFlag=0;
//const uint8_t adc_ch_array[6] = {ADC_INV_V, ADC_SOL_V, ADC_LOAD_I, ADC_TEMP_1, ADC_TEMP_2, ADC_TEMP_2};
const uint8_t adc_ch_array[6] = {ADC_INV_V, ADC_SOL_V, ADC_LOAD_I, ADC_TEMP_1, ADC_V_REF, ADC_TEMP_2}; //VIN

float inverter_vol=0.0,temperature1=0.0,temperature2=0.0,vol_ref=0.0;
uint8_t mains=0;

float get_temp1()
{
    return temperature1;
}

void init_adc(void){
   adc1_config_width(ADC_WIDTH_BIT_13);
   for(int i=0;i<6;i++){
	adc1_config_channel_atten(adc_ch_array[i], ADC_ATTEN_DB_11);   
   }
}

void get_uc_adc_all(uint8_t *adc_buff){
	uint8_t adc_count = 0, buff_idx = 0;
	uint16_t adc_val = 0;
	for(adc_count = 0, buff_idx = 0; adc_count < 6; adc_count++, buff_idx += 2){
		adc_val = adc1_get_raw(adc_ch_array[adc_count]);


		adc_buff[buff_idx] = ((adc_val >> 8) & 0xFF);
		adc_buff[buff_idx + 1] = ( adc_val & 0xFF);
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
}

float get_bat_vol(){
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
            ESP_LOGI("get_bat_vol","Calling fatal error handler,since i2c bus is busy");
            fatal_err_handler(I2C_MODULE,I2C_BUS_IS_BUSY);
        }
    };
	uint16_t batValue;
	float batVoltage;
    mark_as_i2c_in_use();
	PAC194X5X_getBatteryVoltage(pPACdevice,&batValue);
    mark_as_i2c_free();
	batVoltage = pac_bat_v_const * batValue;
	return batVoltage;
}

int8_t read_adc_all(uint8_t *adc_all_buff){
    /*0 to 7 PAC; 8 to 13 ADC*/
	int8_t pac_stat;
    uint16_t log_count = 0;
    uint8_t reset_count = 0;
    static uint8_t pac_busy_count = 0;

    while(if_i2c_bus_is_busy() ) {
        vTaskDelay(10/portTICK_RATE_MS);
        log_count++;
        if(log_count >= 500) { // 5 seconds
            ESP_LOGI(TAG,"[%s] I2C bus busy",__func__);
            log_count = 0;
            reset_count++;
        }
        if(reset_count >= SYS_RESET_COUNT) {
            ESP_LOGI("read_adc_all","Calling fatal error handler,since i2c bus is busy");
            fatal_err_handler(I2C_MODULE,I2C_BUS_IS_BUSY);
        }
    };

    mark_as_i2c_in_use();
    if( pac1952_busy() != 0) {
        ESP_LOGI(TAG,"----------------------------------------------------");
        ESP_LOGI(TAG,"Error reading PAC1953 sensor, since i2c bus is busy");
        ESP_LOGI(TAG,"----------------------------------------------------");
        pac_busy_count++;
        mark_as_i2c_free();
        if(pac_busy_count >= PAC_BUSY_MAX_CNT) {
            ESP_LOGI("read_adc_all","Calling fatal error handler,since PAC1952 is busy");
            fatal_err_handler(I2C_MODULE,I2C_BUS_IS_BUSY);
        }
        return 1;
    }
    pac_busy_count = 0;
 	
	pac_stat = PAC194x5x_GetVIP_digital(pPACdevice,adc_all_buff,16);
	if(pac_stat != 0){
        mark_as_i2c_free();
 		return 1; 
 	}	
    mark_as_i2c_free();
 	get_uc_adc_all(&adc_all_buff[16]);
 	return 1;
}

int8_t get_sensor_values(uint8_t is_flush, uint8_t is_tx){
	static uint32_t sol_v_sum = 0, load_i_sum = 0, inv_v_sum = 0, temp1_sum = 0, vref_sum = 0, temp2_sum = 0;
	uint16_t _sol_v = 0, _temp1 = 0, _temp2 = 0, _load_i = 0, _inv_v = 0, _v_ref = 0;
	uint16_t moving_avg = 0;
	uint8_t _adc_buff[16];
	int8_t pac_stat = 0;
	//uint32_t uint_pac_reg = 0;
	int16_t int_pac_reg = 0;
    uint16_t uint16_pac_reg = 0;
	float float_pac_val = 0.0;
    static uint16_t debug_log_count = 0;
    uint8_t dev_type;
    static uint8_t pac_busy_count = 0;
    static uint8_t pac_sensor_err = 0;

    time_t timeinfo = 0;
    struct tm* date_time = NULL;

#ifdef THRESHOLD_CHECK 
    float cur_bat_v = 0.0;
    float cur_chg_i = 0.0;
    float cur_dis_i = 0.0;
    float cur_sol_i = 0.0;
    float cur_sol_v = 0.0;
    static uint16_t bat_ov_uv_cnt = 0;
    static uint16_t chg_oc_cnt = 0;
    static uint16_t dis_oc_cnt = 0;
    static uint16_t sol_oc_cnt = 0;
    static uint16_t sol_ov_cnt = 0;
#endif

    memset(&_adc_buff,0,sizeof(_adc_buff));
	if(test_error(ERR_NOT_CALIB)){
		memset((uint8_t *)&algo_param, 0, SUMMARY_LEN);
	}
	else{
		get_uc_adc_all(_adc_buff);
        _inv_v  = (_adc_buff[0] << 8) | _adc_buff[1];
		_sol_v  = (_adc_buff[2] << 8) | _adc_buff[3];
		_load_i = (_adc_buff[4] << 8) | _adc_buff[5];
		_temp1  = (_adc_buff[6] << 8) | _adc_buff[7];
        _v_ref  = (_adc_buff[8] << 8) | _adc_buff[9];
        _temp2  = (_adc_buff[10] << 8) | _adc_buff[11];
		if(is_flush){
            inv_v_sum  = (_inv_v << ADC_MUL_DIV_FACT);
			sol_v_sum  = (_sol_v << ADC_MUL_DIV_FACT);
			load_i_sum = (_load_i << ADC_MUL_DIV_FACT);
            temp1_sum  = (_temp1 << ADC_MUL_DIV_FACT);
            vref_sum   = (_v_ref << ADC_MUL_DIV_FACT);
            temp2_sum  = (_temp2 << ADC_MUL_DIV_FACT);
		}
		else{
            inv_v_sum += (_inv_v);
			sol_v_sum += (_sol_v );
			load_i_sum += (_load_i);
            temp1_sum += (_temp1);
            vref_sum += (_v_ref);
            temp2_sum += (_temp2);
		}
        moving_avg = (inv_v_sum >> ADC_MUL_DIV_FACT); //Moving average for inverter vol current
		inv_v_sum -= moving_avg;
		inverter_vol = ((float)moving_avg * uc_bat_v_const);
		
        moving_avg = (sol_v_sum >> ADC_MUL_DIV_FACT); //Moving average for solar voltage
		sol_v_sum -= moving_avg;

#ifdef THRESHOLD_CHECK
        cur_sol_v = (float)moving_avg * uc_sol_v_const;
        if(cur_sol_v > MAX_SOLAR_VOL) {
            sol_ov_cnt++;
            time(&timeinfo);
            date_time = localtime(&timeinfo);
            ESP_LOGI("THRSHOLD_CHECK_ALERT", "[VER: %x] [%02d-%02d-%02d %02d:%02d:%02d] sol v: %f COUNT: %d",FW_VERSION,
                    date_time->tm_mday,1+(date_time->tm_mon),date_time->tm_year+1900,date_time->tm_hour,
                    date_time->tm_min,date_time->tm_sec,
                    cur_sol_v,sol_ov_cnt);
            return 2; // Take no action on returning
        }
#endif
		algo_param.cur_sol_v = (float)moving_avg * uc_sol_v_const;

		moving_avg = (load_i_sum >> ADC_MUL_DIV_FACT); //Moving average for CT current
		load_i_sum -= moving_avg;
		algo_param.cur_ac_load_i = ((float)moving_avg / ct_load_i_const);

        moving_avg = (temp1_sum >> ADC_MUL_DIV_FACT); //Moving average for temperature
		temp1_sum -= moving_avg;
		temperature1 = (float)moving_avg * uc_temp1_const;//(float)((float)moving_avg * ((float)3.3/(float) 4096));
        temperature1 = temperature1/2; // Temp hack for temp double issue, uc_temp1_const to be altered it seems. currently it is 0.02

        moving_avg = (vref_sum >> ADC_MUL_DIV_FACT);    //Moving average for voltage reference
        vref_sum -= moving_avg;
        vol_ref = (float) ((float)moving_avg*((float)3.3/(float)4096));
        
        moving_avg = (temp2_sum >> ADC_MUL_DIV_FACT); //Moving average for temperature
		temp2_sum -= moving_avg;
		temperature2 = (float)moving_avg * uc_temp2_const;//(float)((float)moving_avg * ((float)3.3/(float) 4096));
        
        if(mains_sense()){
            mains=1;
        }
        else{
            mains=0;
        }     
        
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
                ESP_LOGI("get_sensor_values","Calling fatal error handler,since i2c bus is busy");
                fatal_err_handler(I2C_MODULE,I2C_BUS_IS_BUSY);
            }
        };
        
        mark_as_i2c_in_use();
        if( pac1952_busy() != 0) {
            ESP_LOGI(TAG,"----------------------------------------------------");
            ESP_LOGI(TAG,"Error reading PAC1953 sensor, since i2c bus is busy");
            ESP_LOGI(TAG,"----------------------------------------------------");
            pac_busy_count++;
            mark_as_i2c_free();
            if(pac_busy_count >= PAC_BUSY_MAX_CNT) {
                ESP_LOGI("get_sensor_values","Calling fatal error handler,since PAC1952 is busy");
                fatal_err_handler(I2C_MODULE,I2C_BUS_IS_BUSY);
            }

            return 1;
        }
        pac_busy_count = 0;
		pac_stat = PAC194x5x_GetVIP_digital(pPACdevice, _adc_buff,16);
		if(pac_stat != 0) {
			ESP_LOGI(TAG,"Error reading PAC1953 sensor");
            pac_sensor_err++;
            mark_as_i2c_free();
            if(pac_sensor_err >= PAC_ERR_MAX_COUNT) {
                ESP_LOGI("get_sensor_values","Calling fatal error handler,since PAC1952 Error");
                fatal_err_handler(I2C_MODULE,PAC_READ_ERR);
            }
            return 1; //1
	 	}
        pac_sensor_err = 0;
        mark_as_i2c_free();

		uint16_pac_reg = (((uint16_t)_adc_buff[4] << 8) | _adc_buff[5]); // CH1 source

#ifdef THRESHOLD_CHECK
        cur_bat_v = (float)uint16_pac_reg * pac_bat_v_const;
        dev_type = get_dev_type();
        if( (dev_type == 1 ) || (dev_type == 5)) {
            if(cur_bat_v > MAX_BAT_VOL_FOR_12V_SYS || cur_bat_v < MIN_BAT_VOL_FOR_12V_SYS) {
                bat_ov_uv_cnt++;
                time(&timeinfo);
                date_time = localtime(&timeinfo);
                ESP_LOGI("THRESHOLD_CHECK_ALERT", "[VER: %x] [%02d-%02d-%02d %02d:%02d:%02d] BAT V: %f COUNT: %d",FW_VERSION,
                date_time->tm_mday,1+(date_time->tm_mon),date_time->tm_year+1900,date_time->tm_hour,
                date_time->tm_min,date_time->tm_sec,
                cur_bat_v,bat_ov_uv_cnt);
                return 2; // Take no action on returning
            }
        } else if( (dev_type == 2) || (dev_type == 6)) {
            if(cur_bat_v > MAX_BAT_VOL_FOR_24V_SYS || cur_bat_v < MIN_BAT_VOL_FOR_24V_SYS) {
                bat_ov_uv_cnt++;
                time(&timeinfo);
                date_time = localtime(&timeinfo);
                ESP_LOGI("THRESHOLD_CHECK_ALERT", "[VER: %x] [%02d-%02d-%02d %02d:%02d:%02d] BAT V: %f COUNT: %d",FW_VERSION,
                date_time->tm_mday,1+(date_time->tm_mon),date_time->tm_year+1900,date_time->tm_hour,
                date_time->tm_min,date_time->tm_sec,
                cur_bat_v,bat_ov_uv_cnt);
                return 2; //Take no action on returning
            }
        }
#endif
        memset(&algo_param.cur_bat_v,0,sizeof(algo_param.cur_bat_v)); // Clearing memory before updating
		algo_param.cur_bat_v = (float)uint16_pac_reg * pac_bat_v_const;

	 	int_pac_reg = (((int16_t)_adc_buff[0] << 8) | _adc_buff[1]); // CH1 sense
	 	if(int_pac_reg & Bit(15)){ // convert 11 bit signed int to 16 bit signed int
            float_pac_val = ((float) int_pac_reg * pac_bat_dis_i_const) - pac_bat_dis_i_offset;

#ifdef THRESHOLD_CHECK
            cur_chg_i = 0.0;
            cur_dis_i = float_pac_val * -1;
            if( cur_dis_i < 0.0) {  // If sensor value itself is -ve it is discharging, else charging
                cur_chg_i = float_pac_val;
                cur_dis_i = 0.0;
            }
            
            if( cur_chg_i > MAX_CHARGING_CUR) {
                chg_oc_cnt++;
                time(&timeinfo);
                date_time = localtime(&timeinfo);
                ESP_LOGI("THRESHOLD_CHECK_ALERT", "[VER: %x] [%02d-%02d-%02d %02d:%02d:%02d] chg i: %f COUNT: %d",FW_VERSION,
                        date_time->tm_mday,1+(date_time->tm_mon),date_time->tm_year+1900,date_time->tm_hour,
                        date_time->tm_min,date_time->tm_sec,
                        cur_chg_i,chg_oc_cnt);
                return 2; //Take no action on returning
            }
            if( cur_dis_i > MAX_DISCHARGING_CUR) {
                dis_oc_cnt++;
                time(&timeinfo);
                date_time = localtime(&timeinfo);
                ESP_LOGI("THRESHOLD_CHECK_ALERT", "[VER: %x] [%02d-%02d-%02d %02d:%02d:%02d] dis i: %f COUNT: %d",FW_VERSION,
                        date_time->tm_mday,1+(date_time->tm_mon),date_time->tm_year+1900,date_time->tm_hour,
                        date_time->tm_min,date_time->tm_sec,
                        cur_dis_i,dis_oc_cnt);
                return 2; //Take no action on returning
            }
#endif
            memset(&algo_param.cur_chg_i,0,sizeof(algo_param.cur_chg_i)); // Clearing memory before updating
            memset(&algo_param.cur_dis_i,0,sizeof(algo_param.cur_dis_i)); // Clearing memory before updating
            algo_param.cur_chg_i = 0.0;
			algo_param.cur_dis_i = float_pac_val * -1;
            if(algo_param.cur_dis_i<0.0){
                algo_param.cur_chg_i = float_pac_val;
                algo_param.cur_dis_i = 0.0;
                pac_bat_dis_p = 0.0;
				pac_bat_chg_p = algo_param.cur_chg_i * algo_param.cur_bat_v;
            }
            else{
                pac_bat_dis_p = algo_param.cur_dis_i * algo_param.cur_bat_v;
                pac_bat_chg_p = 0.0;
            }
		} else {
            float_pac_val = ((float)int_pac_reg * pac_bat_chg_i_const) - pac_bat_chg_i_offset;
#ifdef THRESHOLD_CHECK
            cur_chg_i = float_pac_val;
            if( cur_chg_i > MAX_CHARGING_CUR) {
                chg_oc_cnt++;
                time(&timeinfo);
                date_time = localtime(&timeinfo);
                ESP_LOGI("THRESHOLD_CHECK_ALERT", "[VER: %x] [%02d-%02d-%02d %02d:%02d:%02d] chg i: %f COUNT: %d",FW_VERSION,
                        date_time->tm_mday,1+(date_time->tm_mon),date_time->tm_year+1900,date_time->tm_hour,
                        date_time->tm_min,date_time->tm_sec,
                        cur_chg_i,chg_oc_cnt);
                return 2; //Take no action on returning
            }
#endif
            algo_param.cur_chg_i = float_pac_val;
	 		algo_param.cur_dis_i = 0.0;
	 		pac_bat_dis_p = 0.0;
			pac_bat_chg_p = algo_param.cur_chg_i * algo_param.cur_bat_v;
        }

	 	//int_pac_reg = (((int16_t)_adc_buff[2] << 8) | _adc_buff[3]); // CH2 sense
        memset(&uint16_pac_reg,0,sizeof(uint16_pac_reg));
	 	uint16_pac_reg = (((uint16_t)_adc_buff[2] << 8) | _adc_buff[3]); // CH2 sense //VIN

#ifdef THRESHOLD_CHECK
        cur_sol_i = ((float)uint16_pac_reg * pac_sol_i_const * 1)-pac_sol_i_offset;
        if(cur_sol_i > MAX_SOLAR_CUR) {
            sol_oc_cnt++;
            time(&timeinfo);
            date_time = localtime(&timeinfo);
                ESP_LOGI("ALERT", "[VER: %x] [%02d-%02d-%02d %02d:%02d:%02d] sol i: %f COUNT: %d",FW_VERSION,
                date_time->tm_mday,1+(date_time->tm_mon),date_time->tm_year+1900,date_time->tm_hour,
                date_time->tm_min,date_time->tm_sec,
                cur_sol_i,sol_oc_cnt);
                return 2; // Take no action on returning
        }
#endif
        memset(&algo_param.cur_sol_i,0,sizeof(algo_param.cur_sol_i)); // Clearing memory before updating
		algo_param.cur_sol_i = ((float)uint16_pac_reg * pac_sol_i_const * 1)-pac_sol_i_offset; 
		if(algo_param.cur_sol_i < 0){
			algo_param.cur_sol_i = 0.0;
			pac_sol_p = 0.0;
		}

		dc_load_i = ((algo_param.cur_sol_i + algo_param.cur_dis_i) - algo_param.cur_chg_i);
		if(dc_load_i < 0){
			dc_load_i = 0;
		}
		ac_load_p = algo_param.cur_ac_load_i * AC_POWER_CONST;	

		pac_sol_p = algo_param.cur_sol_i * algo_param.cur_sol_v;
		
		if(pac_sol_p < pac_bat_chg_p){
			mains_chg_p = (pac_bat_chg_p - pac_sol_p);
			sol_bat_p = pac_sol_p;
		}
		else{
			mains_chg_p = 0.0;
			sol_bat_p = pac_bat_chg_p;
		}

		float_pac_val = (algo_param.cur_dis_i * LVD_COEFF);
		bat_frc_trip_exit_v = (bat_frc_trip_exit_v_thr - float_pac_val);
		bat_mains_chg_in_v = (bat_mains_chg_in_v_thr - float_pac_val);
		sol_bat_chg_i_diff = (algo_param.cur_sol_i - algo_param.cur_chg_i);
        
        // DEBUG LOGS
        time(&timeinfo);
        date_time = localtime(&timeinfo);
        dev_type = get_dev_type();
        if( (dev_type == 1 ) || (dev_type == 5)) {
            if(algo_param.cur_bat_v > MAX_BAT_VOL_FOR_12V_SYS) {
                ESP_LOGI("ALERT", "[VER: %x] [%02d-%02d-%02d %02d:%02d:%02d] BAT V: %f ",FW_VERSION,
                date_time->tm_mday,1+(date_time->tm_mon),date_time->tm_year+1900,date_time->tm_hour,
                date_time->tm_min,date_time->tm_sec,
                algo_param.cur_bat_v);
            }
        } else if( (dev_type == 2) || (dev_type == 6)) {
            if(algo_param.cur_bat_v > MAX_BAT_VOL_FOR_24V_SYS) {
                ESP_LOGI("ALERT", "[VER: %x] [%02d-%02d-%02d %02d:%02d:%02d] BAT V: %f ",FW_VERSION,
                date_time->tm_mday,1+(date_time->tm_mon),date_time->tm_year+1900,date_time->tm_hour,
                date_time->tm_min,date_time->tm_sec,
                algo_param.cur_bat_v);
            }
        }
        
        if(algo_param.cur_chg_i > MAX_CHARGING_CUR) {
                ESP_LOGI("ALERT", "[VER: %x] [%02d-%02d-%02d %02d:%02d:%02d] chg i: %f ",FW_VERSION,
                date_time->tm_mday,1+(date_time->tm_mon),date_time->tm_year+1900,date_time->tm_hour,
                date_time->tm_min,date_time->tm_sec,
                algo_param.cur_chg_i);
        }

        if(algo_param.cur_sol_i > MAX_SOLAR_CUR) {
                ESP_LOGI("ALERT", "[VER: %x] [%02d-%02d-%02d %02d:%02d:%02d] sol i: %f ",FW_VERSION,
                date_time->tm_mday,1+(date_time->tm_mon),date_time->tm_year+1900,date_time->tm_hour,
                date_time->tm_min,date_time->tm_sec,
                algo_param.cur_sol_i);
        }

        if(algo_param.cur_sol_v > MAX_SOLAR_VOL) {
                ESP_LOGI("ALERT", "[VER: %x] [%02d-%02d-%02d %02d:%02d:%02d] sol v: %f ",FW_VERSION,
                date_time->tm_mday,1+(date_time->tm_mon),date_time->tm_year+1900,date_time->tm_hour,
                date_time->tm_min,date_time->tm_sec,
                algo_param.cur_sol_v);
        }

        if(debug_log_count >= MAX_DEBUG_LOG_COUNT) {
            ESP_LOGI("------","------------------- [V: %x] ----------------------------",FW_VERSION);
            ESP_LOGI("------","BAT V: %f ",algo_param.cur_bat_v);
            ESP_LOGI("------","cur_chg_i: %f, cur_dis_i: %f",algo_param.cur_chg_i,algo_param.cur_dis_i);
            ESP_LOGI("------","cur_sol_i: %f",algo_param.cur_sol_i);
            ESP_LOGI("------","cur_sol_v: %f",algo_param.cur_sol_v);
            ESP_LOGI("------","cur_ac_load_i: %f",algo_param.cur_ac_load_i);
            ESP_LOGI("------","bat_chg_p: %f",pac_bat_chg_p);
            ESP_LOGI("------","------------------- [V: %x] ----------------------------",FW_VERSION);
            debug_log_count = 0;
        }
        debug_log_count++;
        //DEBUG LOGS
	}
	return 0;
}

void mark_as_i2c_in_use()
{
    i2cFlag = 1;
}

void mark_as_i2c_free()
{
    i2cFlag = 0;
}

uint8_t if_i2c_bus_is_busy()
{
    return i2cFlag;
}
