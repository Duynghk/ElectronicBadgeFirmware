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
#include "esp_chip_info.h"
#include "esp_flash.h"
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

static void read_and_parse_nmea()
{
    printf("Example ready\n");
    while (1) {
        char fmt_buf[32];
        nmea_s *data;

        char *start;
        size_t length;
        LC76F_read_line(&start, &length, 100 /* ms */);
        if (length == 0) {
            continue;
        }

        /* handle data */
        data = nmea_parse(start, length, 0);
        if (data == NULL) {
            printf("Failed to parse the sentence!\n");
            printf("  Type: %.5s (%d)\n", start + 1, nmea_get_type(start));
        } else {
            // if (data->errors != 0) {
            //     printf("WARN: The sentence struct contains parse errors!\n");
            // }

            // if (NMEA_GPGGA == data->type) {
            //     printf("GPGGA sentence\n");
            //     nmea_gpgga_s *gpgga = (nmea_gpgga_s *) data;
            //     printf("Number of satellites: %d\n", gpgga->n_satellites);
            //     printf("Altitude: %f %c\n", gpgga->altitude,
            //            gpgga->altitude_unit);
            // }

            if (NMEA_GPGLL == data->type) {
                printf("GPGLL sentence\n");
                nmea_gpgll_s *pos = (nmea_gpgll_s *) data;
                printf("Longitude:\n");
                printf("  Degrees: %d\n", pos->longitude.degrees);
                printf("  Minutes: %f\n", pos->longitude.minutes);
                printf("  Cardinal: %c\n", (char) pos->longitude.cardinal);
                printf("Latitude:\n");
                printf("  Degrees: %d\n", pos->latitude.degrees);
                printf("  Minutes: %f\n", pos->latitude.minutes);
                printf("  Cardinal: %c\n", (char) pos->latitude.cardinal);
                strftime(fmt_buf, sizeof(fmt_buf), "%H:%M:%S", &pos->time);
                printf("Time: %s\n", fmt_buf);
            }

            // if (NMEA_GPRMC == data->type) {
            //     printf("GPRMC sentence\n");
            //     nmea_gprmc_s *pos = (nmea_gprmc_s *) data;
            //     printf("Longitude:\n");
            //     printf("  Degrees: %d\n", pos->longitude.degrees);
            //     printf("  Minutes: %f\n", pos->longitude.minutes);
            //     printf("  Cardinal: %c\n", (char) pos->longitude.cardinal);
            //     printf("Latitude:\n");
            //     printf("  Degrees: %d\n", pos->latitude.degrees);
            //     printf("  Minutes: %f\n", pos->latitude.minutes);
            //     printf("  Cardinal: %c\n", (char) pos->latitude.cardinal);
            //     strftime(fmt_buf, sizeof(fmt_buf), "%d %b %T %Y", &pos->date_time);
            //     printf("Date & Time: %s\n", fmt_buf);
            //     printf("Speed, in Knots: %f\n", pos->gndspd_knots);
            //     printf("Track, in degrees: %f\n", pos->track_deg);
            //     printf("Magnetic Variation:\n");
            //     printf("  Degrees: %f\n", pos->magvar_deg);
            //     printf("  Cardinal: %c\n", (char) pos->magvar_cardinal);
            //     double adjusted_course = pos->track_deg;
            //     if (NMEA_CARDINAL_DIR_EAST == pos->magvar_cardinal) {
            //         adjusted_course -= pos->magvar_deg;
            //     } else if (NMEA_CARDINAL_DIR_WEST == pos->magvar_cardinal) {
            //         adjusted_course += pos->magvar_deg;
            //     } else {
            //         printf("Invalid Magnetic Variation Direction!\n");
            //     }

            //     printf("Adjusted Track (heading): %f\n", adjusted_course);
            // }

            // if (NMEA_GPGSA == data->type) {
            //     nmea_gpgsa_s *gpgsa = (nmea_gpgsa_s *) data;

            //     printf("GPGSA Sentence:\n");
            //     printf("  Mode: %c\n", gpgsa->mode);
            //     printf("  Fix:  %d\n", gpgsa->fixtype);
            //     printf("  PDOP: %.2lf\n", gpgsa->pdop);
            //     printf("  HDOP: %.2lf\n", gpgsa->hdop);
            //     printf("  VDOP: %.2lf\n", gpgsa->vdop);
            // }

            // if (NMEA_GPGSV == data->type) {
            //     nmea_gpgsv_s *gpgsv = (nmea_gpgsv_s *) data;

            //     printf("GPGSV Sentence:\n");
            //     printf("  Num: %d\n", gpgsv->sentences);
            //     printf("  ID:  %d\n", gpgsv->sentence_number);
            //     printf("  SV:  %d\n", gpgsv->satellites);
            //     printf("  #1:  %d %d %d %d\n", gpgsv->sat[0].prn, gpgsv->sat[0].elevation, gpgsv->sat[0].azimuth, gpgsv->sat[0].snr);
            //     printf("  #2:  %d %d %d %d\n", gpgsv->sat[1].prn, gpgsv->sat[1].elevation, gpgsv->sat[1].azimuth, gpgsv->sat[1].snr);
            //     printf("  #3:  %d %d %d %d\n", gpgsv->sat[2].prn, gpgsv->sat[2].elevation, gpgsv->sat[2].azimuth, gpgsv->sat[2].snr);
            //     printf("  #4:  %d %d %d %d\n", gpgsv->sat[3].prn, gpgsv->sat[3].elevation, gpgsv->sat[3].azimuth, gpgsv->sat[3].snr);
            // }

            // if (NMEA_GPTXT == data->type) {
            //     nmea_gptxt_s *gptxt = (nmea_gptxt_s *) data;

            //     printf("GPTXT Sentence:\n");
            //     printf("  ID: %d %d %d\n", gptxt->id_00, gptxt->id_01, gptxt->id_02);
            //     printf("  %s\n", gptxt->text);
            // }

            // if (NMEA_GPVTG == data->type) {
            //     nmea_gpvtg_s *gpvtg = (nmea_gpvtg_s *) data;

            //     printf("GPVTG Sentence:\n");
            //     printf("  Track [deg]:   %.2lf\n", gpvtg->track_deg);
            //     printf("  Speed [kmph]:  %.2lf\n", gpvtg->gndspd_kmph);
            //     printf("  Speed [knots]: %.2lf\n", gpvtg->gndspd_knots);
            // }

            nmea_free(data);
        }
        vTaskDelay(1000/portTICK_PERIOD_MS);
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
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RAK_RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0; // Null-terminate the string
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
        } else {
            // No data received within the timeout period
            break; // Exit the loop if no data received
        }
    }
    free(data);  
}

int Uplink_message(const char* logName) 
{
    payload_length = 0;
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
    send_data_to_RAK(logName, "AT+SEND=2:AADDBBCC\n");
    vTaskDelay(1000/portTICK_PERIOD_MS);
    return true;
}

void app_main(void)
{
    static const char *TX_TASK_TAG = "RAK_TX_TASK";
    // Enable RAK3172 module
    // esp_rom_gpio_pad_select_gpio(EN_RAK);
    // gpio_set_direction(EN_RAK, GPIO_MODE_OUTPUT);
    // gpio_set_level(EN_RAK, 1);
    // init_RAK3172_interface();
    // send_data_to_RAK(TX_TASK_TAG, "AT+ATM\n");
    // send_data_to_RAK(TX_TASK_TAG, "AT+NWM=1\n");
    // send_data_to_RAK(TX_TASK_TAG, "AT+BAND=9\n");
    // send_data_to_RAK(TX_TASK_TAG, "AT+NJM=0\n");
    // send_data_to_RAK(TX_TASK_TAG, "AT+CLASS=A\n");
    // send_data_to_RAK(TX_TASK_TAG, "AT+DEVADDR=260BE8E0\n");
    // send_data_to_RAK(TX_TASK_TAG, "AT+APPSKEY=1B7AAB91AD987CA6AF11081861FA5E49\n");
    // send_data_to_RAK(TX_TASK_TAG, "AT+NWKSKEY=11BB2F2767E027329F874AED5C28D7E4\n");
    // send_data_to_RAK(TX_TASK_TAG, "AT+LPMLVL=2\n");

    // Enable LC76F module
    esp_rom_gpio_pad_select_gpio(EN_LC76F);
    gpio_set_direction(EN_LC76F, GPIO_MODE_OUTPUT);
    gpio_set_level(EN_LC76F, 1);
    init_LC76F_interface();

    char *line_buf = NULL;
    size_t line_len = 0;
    int timeout = 10000; // thời gian chờ là 1000 ms (1 giây)
    printf("\nHello. I am running\n");
    while (true) 
    {
        // Uplink_message(TX_TASK_TAG);
        // vTaskDelay(10000/portTICK_PERIOD_MS);
        // Gọi hàm
        read_and_parse_nmea();
        LC76F_read_line(&line_buf, &line_len, timeout);

        // Kiểm tra và sử dụng dữ liệu trả về (nếu cần)
        if (line_buf != NULL) {
            printf("Read line: %s\n", line_buf);
            printf("Line length: %zu\n", line_len);
        } else {
            printf("No line read within the timeout period.\n");
        }
        vTaskDelay(1000/portTICK_PERIOD_MS);

    }
}