#ifndef BUZZER_H
#define BUZZER_H

#include "driver/ledc.h"

// Định nghĩa pin buzzer
#define BUZZER_PIN 10

// Hàm khởi tạo buzzer với một tần số cố định và thời gian phát
void notify(int frequency, int duration_ms);

#endif // BUZZER_H
