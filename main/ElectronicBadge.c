/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "esp_log.h"
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_sleep.h"
#include "nmea.h"
#include "driver/uart.h"
#include "sdkconfig.h"
#include "esp_idf_version.h"
#include "driver/gpio.h"
#include "readLocation.h"
#include "buzzer.h"


#define RAK_RX_BUF_SIZE        (1024)
#define RAK_TXD 21
#define RAK_RXD 20
#define EN_LC76F 1
#define EN_RAK 4
#define BUTTON 2
#define BUZZER 10
#define NOTIFY_FREQUENCY2 4000

Coordinate coord;
SemaphoreHandle_t locatedSemaphore = NULL;

//Create payload of LoRaWAN packet
uint8_t payload[64] = {0};
uint8_t payload_length;

uint32_t lat = 0;
uint32_t lon = 0;
uint16_t voltage = 0;


static void recieve_data_from_RAK(void *arg);
int send_data_to_RAK(const char* logName, const char* data);


void init_RAK3172_interface(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RAK_RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, RAK_TXD, RAK_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void setup_RAK_module(const char* logName) 
{
    send_data_to_RAK(logName, "AT+ATM\n");
    send_data_to_RAK(logName, "AT+NWM=1\n");
    send_data_to_RAK(logName, "AT+BAND=9\n");
    send_data_to_RAK(logName, "AT+NJM=0\n");
    send_data_to_RAK(logName, "AT+CLASS=A\n");

    //Device 1
    send_data_to_RAK(logName, "AT+DEVADDR=260B5782\n");
    send_data_to_RAK(logName, "AT+APPSKEY=C331A4DE9D877AD3B17794914958C63B\n");
    send_data_to_RAK(logName, "AT+NWKSKEY=7EDEC499C79A1BA56056D6D19111AF64\n");

    // Device 2
    // send_data_to_RAK(logName, "AT+DEVADDR=260B37AA\n");
    // send_data_to_RAK(logName, "AT+APPSKEY=0A47A418539390FDF8A5FD3A39FEECA8\n");
    // send_data_to_RAK(logName, "AT+NWKSKEY=0697FFA8981B5B4D897070C77A7A523E\n");
    // send_data_to_RAK(logName, "AT+LPMLVL=2\n");
}

int send_data_to_RAK(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    recieve_data_from_RAK(NULL);
    return txBytes;
}

static void recieve_data_from_RAK(void *arg)
{
    static const char *RX_TASK_TAG = "RAK_RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RAK_RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RAK_RX_BUF_SIZE, 80 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0; // Null-terminate the string
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
        } else {
            // No data received within the timeout period
            break; // Exit the loop if no data received
        }
        // vTaskDelay(100/portTICK_PERIOD_MS);
    }
    free(data);  
}

int Uplink_message(const char* logName) 
{
    if (xSemaphoreTake(locatedSemaphore, portMAX_DELAY) == pdTRUE) {
        printf("UPLINK TASK: Coordinates - lat: %d°%f', lon: %d°%f'\n",
        coord.lat_degrees, coord.lat_minutes,
        coord.lon_degrees, coord.lon_minutes);
        lat = (uint32_t)((coord.lat_degrees + coord.lat_minutes/60) * 1.0e6);
        lon = (uint32_t)((coord.lon_degrees + coord.lon_minutes/60) * 1.0e6);
        payload_length = 0;
        payload[payload_length++] = (uint8_t) lat;
        payload[payload_length++] = (uint8_t) (lat >> 8);
        payload[payload_length++] = (uint8_t) (lat >> 16);
        payload[payload_length++] = (uint8_t) (lat >> 24);
        payload[payload_length++] = (uint8_t) lon;
        payload[payload_length++] = (uint8_t) (lon >> 8);
        payload[payload_length++] = (uint8_t) (lon >> 16);
        payload[payload_length++] = (uint8_t) (lon >> 24);
        payload[payload_length++] = (uint8_t)(voltage >> 8) & 0xff;
        payload[payload_length++] = (uint8_t)voltage & 0xff;
        char hexStr[3];
        send_data_to_RAK(logName, "AT+SEND=2:");
        for(int i = 0; i < payload_length; i++){
            sprintf(hexStr, "%02X", payload[i]);
            send_data_to_RAK(logName, hexStr);
        }
        send_data_to_RAK(logName, "\n");
        // send_data_to_RAK(logName, "AT+SEND=2:AADDBBCC\n");
        notify(NOTIFY_FREQUENCY2,200);
        vTaskDelay(100/portTICK_PERIOD_MS);
        xSemaphoreGive(locatedSemaphore);
    }

    
    vTaskDelay(1000/portTICK_PERIOD_MS);
    return true;
}

void init_semaphore(SemaphoreHandle_t *semaphore) 
{
    // Tạo semaphore
    *semaphore = xSemaphoreCreateBinary();
    
    if (*semaphore == NULL) {
        ESP_LOGE("Semaphore", "Semaphore creation failed!");
        return;
    }
}

void app_main(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);

    // Enable LC76F module
    esp_rom_gpio_pad_select_gpio(EN_LC76F);
    gpio_set_direction(EN_LC76F, GPIO_MODE_OUTPUT);
    gpio_set_level(EN_LC76F, 1);
    init_LC76F_interface();
    init_semaphore(&locatedSemaphore);
    GPSTaskParameters params;
    params.coord = &coord;
    params.locatedSemaphore = locatedSemaphore;
    xTaskCreate((TaskFunction_t)read_and_parse_nmea, "GPS_TASK", 2048, (void*) &params, 5, NULL);

    // Enable RAK3172 module
    esp_rom_gpio_pad_select_gpio(EN_RAK);
    gpio_set_direction(EN_RAK, GPIO_MODE_OUTPUT);
    gpio_set_level(EN_RAK, 1);
    init_RAK3172_interface();
    setup_RAK_module("SET UP RAK");

    //
    // buzzer_init(BUZZER);

    while (true) 
    {
        // if (xSemaphoreTake(locatedSemaphore, portMAX_DELAY) == pdTRUE) {
            Uplink_message("UPLINK TASK");
            esp_deep_sleep_enable_gpio_wakeup(1ULL << BUTTON, 0);
            ESP_LOGI("Deep Sleep", "Entering deep sleep");
            esp_deep_sleep_start();
        // }
    }
}