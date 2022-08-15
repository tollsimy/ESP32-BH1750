#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ESP32_BH1750.h"

static const char *TAG = "Demo-BH1750";

#define SDA_PIN (27)
#define SCL_PIN (32)
#define I2C_PORT (0)

I2C_CONF={
    .mode = I2C_MODE_MASTER;
    .sda_io_num = SDA_PIN;
    .scl_io_num = SCL_PIN;
    .sda_pullup_en = GPIO_PULLUP_DISABLE;     //disable if you have external pullup
    .scl_pullup_en = GPIO_PULLUP_DISABLE;
    .master.clk_speed = 400000;               //I2C Full Speed
}

void BH1750_task(){
    //Install I2C Driver
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &(I2C_CONF)));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    for(;;){
        uint16_t lux;
        uint8_t MTime=69; //default value
        ESP32_BH1750 BH1750={0};

        ESP_LOGI(TAG, "BH1750 task started");
        ESP_ERROR_CHECK(BH1750_init(&BH1750,BH1750_ADDR_LO, BH1750_I2C_PORT));
        BH1750_power_on(&BH1750);
        BH1750_set(&BH1750, BH1750_MODE_CONTINUOUS, BH1750_RES_HIGH, MTime);

        BH1750_measure_and_read(&BH1750,&lux);

        ESP_LOGI(TAG, "BH1750: %d", lux);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_PORT));
}

void app_main(void)
{   
    xTaskCreate(&BH1750_task, "BH1750_task", 2048, NULL, 5, NULL);
}
