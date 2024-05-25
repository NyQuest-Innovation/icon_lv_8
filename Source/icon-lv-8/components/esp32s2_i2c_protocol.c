#include "esp32s2_i2c_protocol.h"
 
i2c_port_t i2c_num; 
i2c_slave_addr_t i2c_slave_addr;
SemaphoreHandle_t i2c_bus_lock_semaphore = NULL;

void init_i2c(void)
{   
    i2c_num = I2C_MASTER;
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_GPIO,
        .scl_io_num = SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000};
    i2c_param_config(I2C_MASTER, &i2c_config);
    i2c_driver_install(I2C_MASTER, I2C_MODE_MASTER, 0, 0, 0);
    // Create a semaphore for i2c locking purpose
    i2c_bus_lock_semaphore = xSemaphoreCreateMutex();
    assert(i2c_bus_lock_semaphore != NULL);
}

esp_err_t i2c_master_detect_slave(void)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_slave_addr) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

bool lock_i2c_bus(uint32_t wait_time_ms)
{
    if(i2c_bus_lock_semaphore != NULL) {
        if( xSemaphoreTake( i2c_bus_lock_semaphore, (wait_time_ms/portTICK_RATE_MS) ) == pdTRUE ) {
            // Able to obatin the semaphore, can access the i2c bus now
            return true;
        } else {
            // Could not obtain the semaphore, cannot access the i2c bus now
            return false;
        }
    
    } else {
        ESP_LOGI("I2C", "Locking semaphore is not initialised");
    }
    return false;
}

void unlock_i2c_bus()
{
     if(i2c_bus_lock_semaphore != NULL) {
         xSemaphoreGive(i2c_bus_lock_semaphore);
     }
}
