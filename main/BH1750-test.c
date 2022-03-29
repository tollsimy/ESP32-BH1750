#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

uint8_t SDA_PIN=18;
uint8_t SCL_PIN=19;
uint8_t I2C_PORT=0;

#include "bh1750.h"

static const char *TAG = "Demo-BH1750";

void BH1750_task(){
    uint16_t lux;
    uint8_t MTime=69; //default value
    BH1750 BH1750={0};

    ESP_LOGI(TAG, "BH1750 task started");
    bh1750_init_desc(&BH1750,BH1750_ADDR_LO,I2C_PORT,SDA_PIN,SCL_PIN);
    bh1750_power_on(&BH1750);
    bh1750_set_measurement_time(&BH1750, MTime);
    bh1750_set_and_measure(&BH1750, BH1750_MODE_CONTINUOUS, BH1750_RES_HIGH2);

    bh1750_read_measure(&BH1750,&lux);

    ESP_LOGI(TAG, "BH1750: %d", lux);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void app_main(void)
{   
    xTaskCreate(&BH1750_task, "BH1750_task", 2048, NULL, 5, NULL);
}
