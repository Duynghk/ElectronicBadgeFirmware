#include "gpgll.h"
#include "gpgga.h"
#include "gprmc.h"
#include "gpgsa.h"
#include "gpvtg.h"
#include "gptxt.h"
#include "gpgsv.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define LC76F_RX_BUF_SIZE        (1024)
#define LC76_TXD 7
#define LC76_RXD 3
#define NOTIFY_FREQUENCY1 9000

typedef struct {
    int lat_degrees;
    float lat_minutes;
    char lat_cardinal;
    int lon_degrees;
    float lon_minutes;
    char lon_cardinal;
} Coordinate;

typedef struct {
    Coordinate *coord;
    SemaphoreHandle_t locatedSemaphore;
} GPSTaskParameters;

void init_LC76F_interface(void);
void LC76F_read_line(char **out_line_buf, size_t *out_line_len, int timeout_ms);
void read_and_parse_nmea(void *pvParameters);
