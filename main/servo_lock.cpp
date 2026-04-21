#include "servo_lock.h"

#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
constexpr char TAG[] = "servo_lock";

constexpr gpio_num_t kServoGpio = GPIO_NUM_4;
constexpr ledc_timer_t kTimer = LEDC_TIMER_0;
constexpr ledc_channel_t kChannel = LEDC_CHANNEL_0;
constexpr ledc_mode_t kSpeedMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_bit_t kDutyResolution = LEDC_TIMER_14_BIT;

constexpr int kPwmFrequencyHz = 50;
constexpr int kPwmPeriodUs = 1000000 / kPwmFrequencyHz;
constexpr int kDutyMax = (1 << 14) - 1;

constexpr int kLockedAngle = 15;
constexpr int kUnlockedAngle = 105;
constexpr int kMinPulseUs = 500;
constexpr int kMaxPulseUs = 2500;

int angle_to_pulse_us(int angle)
{
    if (angle < 0) {
        angle = 0;
    } else if (angle > 180) {
        angle = 180;
    }
    return kMinPulseUs + ((kMaxPulseUs - kMinPulseUs) * angle) / 180;
}
} // namespace

esp_err_t ServoLock::init()
{
    ledc_timer_config_t timer_config = {
        .speed_mode = kSpeedMode,
        .duty_resolution = kDutyResolution,
        .timer_num = kTimer,
        .freq_hz = kPwmFrequencyHz,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_config), TAG, "Failed to configure LEDC timer");

    ledc_channel_config_t channel_config = {
        .gpio_num = kServoGpio,
        .speed_mode = kSpeedMode,
        .channel = kChannel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = kTimer,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        .flags = {
            .output_invert = 0,
        },
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&channel_config), TAG, "Failed to configure LEDC channel");

    ESP_RETURN_ON_ERROR(lock(), TAG, "Failed to move servo to initial locked angle");
    vTaskDelay(pdMS_TO_TICKS(600));
    ESP_RETURN_ON_ERROR(unlock(), TAG, "Failed to move servo during startup test");
    vTaskDelay(pdMS_TO_TICKS(600));
    return lock();
}

esp_err_t ServoLock::lock()
{
    ESP_LOGI(TAG, "Locking");
    ESP_RETURN_ON_ERROR(write_angle(kLockedAngle), TAG, "Failed to move servo to locked angle");
    locked_ = true;
    return ESP_OK;
}

esp_err_t ServoLock::unlock()
{
    ESP_LOGI(TAG, "Unlocking");
    ESP_RETURN_ON_ERROR(write_angle(kUnlockedAngle), TAG, "Failed to move servo to unlocked angle");
    locked_ = false;
    return ESP_OK;
}

bool ServoLock::is_locked() const
{
    return locked_;
}

esp_err_t ServoLock::write_angle(int angle)
{
    return write_pulse_us(angle_to_pulse_us(angle));
}

esp_err_t ServoLock::write_pulse_us(int pulse_us)
{
    const uint32_t duty = (static_cast<uint32_t>(pulse_us) * kDutyMax) / kPwmPeriodUs;
    ESP_LOGI(TAG, "Writing PWM: gpio=%d pulse=%dus duty=%lu/%d",
             static_cast<int>(kServoGpio), pulse_us, static_cast<unsigned long>(duty), kDutyMax);

    ESP_RETURN_ON_ERROR(ledc_set_duty(kSpeedMode, kChannel, duty), TAG, "Failed to set duty");
    ESP_RETURN_ON_ERROR(ledc_update_duty(kSpeedMode, kChannel), TAG, "Failed to update duty");
    return ESP_OK;
}
