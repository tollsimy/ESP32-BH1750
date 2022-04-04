#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

uint8_t BH1750_SDA_PIN=18;
uint8_t BH1750_SCL_PIN=19;
uint8_t BH1750_I2C_PORT=0;

#include "ESP32_BH1750.h"

static const char *TAG = "Demo-BH1750";

void BH1750_task(){
    for(;;){
        uint16_t lux;
        uint8_t MTime=69; //default value
        ESP32_BH1750 BH1750={0};

        ESP_LOGI(TAG, "BH1750 task started");
        BH1750_init(&BH1750,BH1750_ADDR_LO,BH1750_I2C_PORT,BH1750_SDA_PIN,BH1750_SCL_PIN);
        BH1750_power_on(&BH1750);
        BH1750_set(&BH1750, BH1750_MODE_CONTINUOUS, BH1750_RES_HIGH, MTime);

        BH1750_measure_and_read(&BH1750,&lux);

        ESP_LOGI(TAG, "BH1750: %d", lux);
        BH1750_delete();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{   
    xTaskCreate(&BH1750_task, "BH1750_task", 2048, NULL, 5, NULL);
}
