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
#include "gpgll.h"
#include "gpgga.h"
#include "gprmc.h"
#include "gpgsa.h"
#include "gpvtg.h"
#include "gptxt.h"
#include "gpgsv.h"

#define LC76F_RX_BUF_SIZE        (1024)
#define RAK_RX_BUF_SIZE        (1024)
#define LC76_TXD 7
#define LC76_RXD 3
#define RAK_TXD 21
#define RAK_RXD 20
#define EN_LC76F 1
#define EN_RAK 4
#define BUTTON 2

typedef struct {
    int lat_degrees;
    float lat_minutes;
    char lat_cardinal;
    int lon_degrees;
    float lon_minutes;
    char lon_cardinal;
} Coordinate;

SemaphoreHandle_t locatedSemaphore = NULL;


Coordinate coord;

//Create payload of LoRaWAN packet
uint8_t payload[64] = {0};
uint8_t payload_length;

uint32_t lat = 0;
uint32_t lng = 0;

static char s_buf[LC76F_RX_BUF_SIZE + 1];
static size_t s_total_bytes;
static char *s_last_buf_end;
static void read_and_parse_nmea();
void LC76F_read_line(char **out_line_buf, size_t *out_line_len, int timeout_ms);
static void recieve_data_from_RAK(void *arg);
int send_data_to_RAK(const char* logName, const char* data);

static void read_and_parse_nmea(void *pvParameters)
{
    Coordinate *coord = (Coordinate *)pvParameters;
    while (1) {
        char fmt_buf[32];
        nmea_s *data;

        char *start;
        size_t length;
        LC76F_read_line(&start, &length, 100 /* ms */);
        if (length == 0) {
            printf("Waiting for LC76F start\n");
            continue;
        }

        /* handle data */
        data = nmea_parse(start, length, 0);
        if (data == NULL) {
            printf("Failed to parse the sentence!\n");
            printf("  Type: %.5s (%d)\n", start + 1, nmea_get_type(start));
        } else {
            if (data->errors != 0) {
                printf("WARN: The sentence struct contains parse errors!\n");
            }
            if (NMEA_GPGLL == data->type) {
                // printf("GPGLL sentence\n");
                nmea_gpgll_s *pos = (nmea_gpgll_s *) data;
                if (pos->latitude.degrees != 0 && pos->latitude.minutes != 0 ) 
                {
                    xSemaphoreGive(locatedSemaphore);
                        // Truy cập và cập nhật dữ liệu tọa độ
                    coord->lat_degrees = pos->latitude.degrees;
                    coord->lat_minutes = pos->latitude.minutes;
                    coord->lat_cardinal = pos->latitude.cardinal;
                    coord->lon_degrees = pos->longitude.degrees;
                    coord->lon_minutes = pos->longitude.minutes;
                    coord->lon_cardinal = pos->longitude.cardinal;
                    printf("Task 1: Updated coordinates\n");
                }  
                else 
                {
                    printf("Can't locate\n");
                }
            }
            if (NMEA_GPRMC == data->type) {
                nmea_gprmc_s *pos = (nmea_gprmc_s *) data;
                double adjusted_course = pos->track_deg;
                if (NMEA_CARDINAL_DIR_EAST == pos->magvar_cardinal) {
                    adjusted_course -= pos->magvar_deg;
                } else if (NMEA_CARDINAL_DIR_WEST == pos->magvar_cardinal) {
                    adjusted_course += pos->magvar_deg;
                } else {
                    printf("Invalid Magnetic Variation Direction!\n");
                }
            }
            nmea_free(data);
        }
        vTaskDelay(200/portTICK_PERIOD_MS);
    }
}
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

void init_LC76F_interface(void)
{
    uart_config_t uart_config = {
        // .baud_rate = 115200,
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        .source_clk = UART_SCLK_DEFAULT,
#endif
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, LC76_TXD, LC76_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, LC76F_RX_BUF_SIZE * 2, 0, 0, NULL, 0));
}

void LC76F_read_line(char **out_line_buf, size_t *out_line_len, int timeout_ms)
{
    *out_line_buf = NULL;
    *out_line_len = 0;

    if (s_last_buf_end != NULL) {
        /* Data left at the end of the buffer after the last call;
         * copy it to the beginning.
         */
        size_t len_remaining = s_total_bytes - (s_last_buf_end - s_buf);
        memmove(s_buf, s_last_buf_end, len_remaining);
        s_last_buf_end = NULL;
        s_total_bytes = len_remaining;
    }

    /* Read data from the UART */
    int read_bytes = uart_read_bytes(UART_NUM_0,
                                     (uint8_t *) s_buf + s_total_bytes,
                                     LC76F_RX_BUF_SIZE - s_total_bytes, pdMS_TO_TICKS(timeout_ms));
    if (read_bytes <= 0) {
        return;
    }
    s_total_bytes += read_bytes;

    /* find start (a dollar sign) */
    char *start = memchr(s_buf, '$', s_total_bytes);
    if (start == NULL) {
        s_total_bytes = 0;
        return;
    }

    /* find end of line */
    char *end = memchr(start, '\r', s_total_bytes - (start - s_buf));
    if (end == NULL || *(++end) != '\n') {
        return;
    }
    end++;

    end[-2] = NMEA_END_CHAR_1;
    end[-1] = NMEA_END_CHAR_2;

    *out_line_buf = start;
    *out_line_len = end - start;
    if (end < s_buf + s_total_bytes) {
        /* some data left at the end of the buffer, record its position until the next call */
        s_last_buf_end = end;
    } else {
        s_total_bytes = 0;
    }
}

void setup_RAK_module(const char* logName) 
{
    send_data_to_RAK(logName, "AT+ATM\n");
    send_data_to_RAK(logName, "AT+NWM=1\n");
    send_data_to_RAK(logName, "AT+BAND=9\n");
    send_data_to_RAK(logName, "AT+NJM=0\n");
    send_data_to_RAK(logName, "AT+CLASS=A\n");

    //Device 1
    send_data_to_RAK(logName, "AT+DEVADDR=260BE8E0\n");
    send_data_to_RAK(logName, "AT+APPSKEY=1B7AAB91AD987CA6AF11081861FA5E49\n");
    send_data_to_RAK(logName, "AT+NWKSKEY=11BB2F2767E027329F874AED5C28D7E4\n");

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
        printf("Chạy rồi nè");
        // Truy cập dữ liệu tọa độ
        printf("UPLINK TASK: Coordinates - lat: %d°%.4f', lon: %d°%.4f'\n",
        coord.lat_degrees, coord.lat_minutes,
        coord.lon_degrees, coord.lon_minutes);
        send_data_to_RAK(logName, "AT+SEND=2:AADDBBCC\n");
        vTaskDelay(100/portTICK_PERIOD_MS);
        xSemaphoreGive(locatedSemaphore);
    }
    printf("Bug đây chứ đâu");
    // payload_length = 0;
    // payload[payload_length++] = (uint8_t) lat;
    // payload[payload_length++] = (uint8_t) (lat >> 8);
    // payload[payload_length++] = (uint8_t) (lat >> 16);
    // payload[payload_length++] = (uint8_t) (lat >> 24);
    // payload[payload_length++] = (uint8_t) lng;
    // payload[payload_length++] = (uint8_t) (lng >> 8);
    // payload[payload_length++] = (uint8_t) (lng >> 16);
    // payload[payload_length++] = (uint8_t) (lng >> 24);
    // payload[payload_length++] = (uint8_t)(voltage >> 8) & 0xff;
    // payload[payload_length++] = (uint8_t)voltage & 0xff;
    
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
    init_semaphore(&startedGpsSemaphore);
    init_semaphore(&locatedSemaphore);
    xTaskCreate((TaskFunction_t)read_and_parse_nmea, "GPS_TASK", 2048, (void*) &coord, 5, NULL);

    // Enable RAK3172 module
    esp_rom_gpio_pad_select_gpio(EN_RAK);
    gpio_set_direction(EN_RAK, GPIO_MODE_OUTPUT);
    gpio_set_level(EN_RAK, 1);
    init_RAK3172_interface();
    setup_RAK_module("SET UP RAK");

    while (true) 
    {
        // if (xSemaphoreTake(locatedSemaphore, portMAX_DELAY) == pdTRUE) {
            printf("Toang rồi nó chạy trước\n");
            Uplink_message("UPLINK TASK");
            esp_deep_sleep_enable_gpio_wakeup(1ULL << BUTTON, 0);
            ESP_LOGI("Deep Sleep", "Entering deep sleep");
            esp_deep_sleep_start();
        // }
    }
}