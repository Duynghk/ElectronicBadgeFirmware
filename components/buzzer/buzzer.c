#include "buzzer.h"
#include "esp_timer.h"

static void buzzer_timer_callback(void *arg) {
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0); // Dừng buzzer khi hết thời gian
}

void notify(int frequency, int duration_ms) {
    printf("Ran\n");
    // Cấu hình Timer cho LEDC
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // Độ phân giải 13-bit
        .freq_hz = frequency,                 // Tần số
        .speed_mode = LEDC_LOW_SPEED_MODE,    // Chế độ tốc độ thấp
        .timer_num = LEDC_TIMER_0,            // Sử dụng Timer 0
        .clk_cfg = LEDC_AUTO_CLK,             // Sử dụng xung nhịp tự động
    };
    ledc_timer_config(&ledc_timer);

    // Cấu hình LEDC cho kênh PWM
    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = (1 << 12), // Đặt mức duty cycle ở 50% (2^12 / 2^13 = 50%)
        .gpio_num   = BUZZER_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    };
    ledc_channel_config(&ledc_channel);

    // Bắt đầu phát buzzer
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (1 << 12)); // Đặt mức duty cycle lên 50%
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0); // Cập nhật duty cycle

    // Tạo một timer để dừng buzzer sau khi hết thời gian
    esp_timer_create_args_t buzzer_timer_args = {
        .callback = buzzer_timer_callback,
        .name = "buzzer_timer"
    };
    esp_timer_handle_t buzzer_timer;
    esp_timer_create(&buzzer_timer_args, &buzzer_timer);
    esp_timer_start_once(buzzer_timer, duration_ms * 1000); // Chuyển đổi thời gian từ ms sang microseconds
}
