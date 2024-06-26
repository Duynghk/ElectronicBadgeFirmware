#include <stdio.h>
#include "readLocation.h"
#include "driver/uart.h"
#include "buzzer.h"

static char s_buf[LC76F_RX_BUF_SIZE + 1];
static size_t s_total_bytes;
static char *s_last_buf_end;
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

void read_and_parse_nmea(void *pvParameters)
{
    GPSTaskParameters *params = (GPSTaskParameters *)pvParameters;
    Coordinate *coord = params->coord;
    SemaphoreHandle_t locatedSemaphore = params->locatedSemaphore;
    while (1) {
        // char fmt_buf[32];
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
                    notify(NOTIFY_FREQUENCY1,100);
                    vTaskDelay(50/portTICK_PERIOD_MS);
                    notify(NOTIFY_FREQUENCY1,100);
                    vTaskDelay(50/portTICK_PERIOD_MS);
                    notify(NOTIFY_FREQUENCY1,100);
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