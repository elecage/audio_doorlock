#include "battery_monitor.h"

#include "esp_check.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

#include <algorithm>

namespace {
constexpr char TAG[] = "battery_monitor";

// ESP32-C3 Super Mini에서 서보 신호 GPIO4와 겹치지 않는 ADC 핀을 기본값으로 둡니다.
// 실제 배선에서 다른 ADC 핀을 쓰면 이 값만 바꾸면 됩니다.
constexpr int kBatteryAdcGpio = 3;

// 기본 회로는 배터리+ -- 100k -- ADC -- 100k -- GND 의 1:1 분압을 가정합니다.
// ADC 입력 전압 = 배터리 전압 * R_BOTTOM / (R_TOP + R_BOTTOM)
constexpr uint32_t kDividerTopOhms = 100000;
constexpr uint32_t kDividerBottomOhms = 100000;

// 단일 Li-ion/LiPo 셀 기준의 보수적인 기본값입니다.
// 다른 배터리 팩을 쓰면 실제 전압 곡선에 맞게 조정해야 합니다.
constexpr uint32_t kBatteryFullMv = 4200;
constexpr uint32_t kBatteryEmptyMv = 3300;
constexpr uint32_t kBatteryLowMv = 3500;
constexpr uint32_t kBatteryCriticalMv = 3300;

constexpr int kSampleCount = 8;

uint8_t voltage_to_percent(uint32_t battery_mv)
{
    if (battery_mv <= kBatteryEmptyMv) {
        return 0;
    }
    if (battery_mv >= kBatteryFullMv) {
        return 100;
    }

    return static_cast<uint8_t>(((battery_mv - kBatteryEmptyMv) * 100U) /
                                (kBatteryFullMv - kBatteryEmptyMv));
}

uint32_t adc_mv_to_battery_mv(uint32_t adc_mv)
{
    return static_cast<uint32_t>(
        (static_cast<uint64_t>(adc_mv) * (kDividerTopOhms + kDividerBottomOhms)) /
        kDividerBottomOhms);
}
} // namespace

esp_err_t BatteryMonitor::init()
{
    if (initialized_) {
        return ESP_OK;
    }

    adc_unit_t unit = ADC_UNIT_1;
    adc_channel_t channel = ADC_CHANNEL_0;
    esp_err_t err = adc_oneshot_io_to_channel(kBatteryAdcGpio, &unit, &channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO%d is not an ADC-capable pin: %s", kBatteryAdcGpio, esp_err_to_name(err));
        return err;
    }

    adc_oneshot_unit_handle_t adc_handle = nullptr;
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = unit,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_config, &adc_handle), TAG, "Failed to create ADC unit");

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc_handle, channel, &channel_config),
                        TAG, "Failed to configure battery ADC channel");

    adc_cali_handle_t cali_handle = nullptr;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit,
        .chan = channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle);
#else
    err = ESP_ERR_NOT_SUPPORTED;
#endif
    calibrated_ = (err == ESP_OK);
    if (!calibrated_) {
        ESP_LOGW(TAG, "ADC calibration unavailable, using raw estimate: %s", esp_err_to_name(err));
    }

    adc_handle_ = adc_handle;
    cali_handle_ = cali_handle;
    adc_unit_ = static_cast<int>(unit);
    adc_channel_ = static_cast<int>(channel);
    initialized_ = true;

    ESP_LOGI(TAG, "Battery monitor initialized: gpio=%d adc_unit=%d channel=%d divider=%lu:%lu",
             kBatteryAdcGpio, adc_unit_, adc_channel_,
             static_cast<unsigned long>(kDividerTopOhms),
             static_cast<unsigned long>(kDividerBottomOhms));
    return ESP_OK;
}

esp_err_t BatteryMonitor::read(BatterySample &sample)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    auto adc_handle = static_cast<adc_oneshot_unit_handle_t>(adc_handle_);
    auto cali_handle = static_cast<adc_cali_handle_t>(cali_handle_);
    auto channel = static_cast<adc_channel_t>(adc_channel_);

    uint32_t adc_mv_sum = 0;
    int raw_sum = 0;
    for (int i = 0; i < kSampleCount; ++i) {
        int raw = 0;
        ESP_RETURN_ON_ERROR(adc_oneshot_read(adc_handle, channel, &raw), TAG, "Failed to read battery ADC");
        raw_sum += raw;

        int adc_mv = 0;
        if (calibrated_) {
            ESP_RETURN_ON_ERROR(adc_cali_raw_to_voltage(cali_handle, raw, &adc_mv),
                                TAG, "Failed to calibrate battery ADC");
        } else {
            adc_mv = (raw * 2500) / 4095;
        }
        adc_mv_sum += static_cast<uint32_t>(std::max(adc_mv, 0));
    }

    const uint32_t adc_mv = adc_mv_sum / kSampleCount;
    const uint32_t battery_mv = adc_mv_to_battery_mv(adc_mv);

    sample.battery_mv = battery_mv;
    sample.percent_remaining = voltage_to_percent(battery_mv);
    sample.low = battery_mv <= kBatteryLowMv;
    sample.critical = battery_mv <= kBatteryCriticalMv;

    ESP_LOGI(TAG, "Battery sample: raw=%d adc=%lumV battery=%lumV percent=%u low=%d critical=%d",
             raw_sum / kSampleCount,
             static_cast<unsigned long>(adc_mv),
             static_cast<unsigned long>(sample.battery_mv),
             sample.percent_remaining,
             sample.low,
             sample.critical);
    return ESP_OK;
}
