#pragma once

#include "esp_err.h"

#include <cstdint>

struct BatterySample {
    uint32_t battery_mv;
    uint8_t percent_remaining;
    bool low;
    bool critical;
};

class BatteryMonitor {
public:
    esp_err_t init();
    esp_err_t read(BatterySample &sample);

private:
    bool initialized_ = false;
    void *adc_handle_ = nullptr;
    void *cali_handle_ = nullptr;
    int adc_unit_ = 0;
    int adc_channel_ = 0;
    bool calibrated_ = false;
};
