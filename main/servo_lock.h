#pragma once

#include "esp_err.h"

class ServoLock {
public:
    esp_err_t init();
    esp_err_t lock();
    esp_err_t unlock();
    bool is_locked() const;

private:
    esp_err_t write_angle(int angle);
    esp_err_t write_pulse_us(int pulse_us);

    bool locked_ = true;
};
