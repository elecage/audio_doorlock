#include "servo_lock.h"

#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_matter.h"
#include "esp_matter_attribute.h"
#include "esp_matter_cluster.h"
#include "esp_matter_endpoint.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "nvs_flash.h"

#include <app-common/zap-generated/ids/Attributes.h>
#include <app-common/zap-generated/ids/Clusters.h>
#include <app/clusters/door-lock-server/door-lock-server.h>
#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>
#include <platform/CHIPDeviceLayer.h>

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

namespace {
// ESP-IDF 로그에서 이 파일의 메시지를 구분하기 위한 태그입니다.
constexpr char TAG[] = "matter_lock";

// 아직 어떤 Matter fabric에도 등록되지 않은 새 장치라면,
// Google Home 같은 컨트롤러가 페어링할 수 있도록 commissioning window를 엽니다.
// 이 값은 commissioning window를 열어둘 시간입니다.
constexpr uint32_t kCommissioningWindowTimeoutSeconds = 300;

// Google Assistant는 Door Lock endpoint의 음성 잠금해제를 막습니다.
// 그래서 별도 On/Off 스위치 endpoint를 "열림 트리거"로 사용합니다.
// 스위치가 켜지면 즉시 열고, 이 시간 뒤 자동으로 다시 잠급니다.
constexpr uint64_t kVoiceTriggerRelockDelayUs = 5ULL * 1000ULL * 1000ULL;

// Matter Door Lock cluster의 LockState 값을 애플리케이션에서 다루기 쉽게 표현한 enum입니다.
// 숫자 값은 Matter Door Lock specification의 LockState enum 값과 맞춰 둡니다.
enum class LockState : uint8_t {
    NotFullyLocked = 0,
    Locked = 1,
    Unlocked = 2,
    Unlatched = 3,
    Unknown = 4,
};

// 실제 서보 제어 객체입니다.
// 이 객체는 PWM만 담당하고, Matter 네트워크나 endpoint 상태는 아래 전역 값들이 담당합니다.
ServoLock g_servo_lock;

// Door Lock endpoint id입니다.
// esp-matter가 endpoint를 생성한 뒤 실제 id를 할당하므로, 생성 후 저장해 둡니다.
uint16_t g_lock_endpoint_id = 0;

// Google Assistant 음성 트리거용 On/Off Plug-in Unit endpoint id입니다.
// Google Home 앱에서는 별도 스위치처럼 보이며, 켜짐 명령을 "잠깐 열기"로 해석합니다.
uint16_t g_voice_trigger_endpoint_id = 0;

// 음성 트리거로 문을 열었을 때 5초 뒤 다시 잠그기 위한 one-shot 타이머입니다.
esp_timer_handle_t g_auto_relock_timer = nullptr;

void open_commissioning_window_if_needed()
{
    // 이미 하나 이상의 Matter fabric에 등록되어 있으면 commissioning window를 열 필요가 없습니다.
    // Google Home에 등록된 이후 매번 부팅할 때 BLE 페어링 광고가 뜨는 것을 막기 위한 조건입니다.
    if (chip::Server::GetInstance().GetFabricTable().FabricCount() != 0) {
        return;
    }

    auto &commissioning = chip::Server::GetInstance().GetCommissioningWindowManager();

    // 이미 열려 있으면 중복으로 열지 않습니다.
    if (commissioning.IsCommissioningWindowOpen()) {
        return;
    }

    // Basic commissioning window는 수동 페어링 코드나 QR 코드로 새 controller가 붙을 수 있게 합니다.
    // 이 프로젝트에서는 개발용 example commissionable data provider를 사용하므로
    // README에 적은 manual pairing code/PIN으로 등록할 수 있습니다.
    const auto timeout = chip::System::Clock::Seconds32(kCommissioningWindowTimeoutSeconds);
    CHIP_ERROR error = commissioning.OpenBasicCommissioningWindow(
        timeout, chip::CommissioningWindowAdvertisement::kAllSupported);
    if (error != CHIP_NO_ERROR) {
        ESP_LOGE(TAG, "Failed to open commissioning window: %" CHIP_ERROR_FORMAT, error.Format());
        return;
    }

    ESP_LOGI(TAG, "Matter commissioning window opened for %lu seconds",
             static_cast<unsigned long>(kCommissioningWindowTimeoutSeconds));
}

void open_commissioning_window_work(intptr_t)
{
    // esp_matter::start()가 내부 Matter stack을 시작한 뒤,
    // CHIP platform work queue에서 commissioning window를 열도록 예약합니다.
    // Matter stack 내부 객체를 다루는 작업은 이 queue에서 실행하는 편이 안전합니다.
    open_commissioning_window_if_needed();
}

esp_err_t update_lock_state_attribute(LockState state)
{
    // Google Home에 "현재 잠금 상태"를 알려주기 위해 DoorLock cluster의 LockState attribute를 갱신합니다.
    // 실제 서보를 움직이는 것과 Matter attribute를 갱신하는 것은 별도 작업입니다.
    // 서보가 움직였는데 attribute 갱신이 실패하면 앱에는 상태가 늦게 반영될 수 있습니다.
    esp_matter_attr_val_t value = esp_matter_enum8(static_cast<uint8_t>(state));
    return attribute::update(g_lock_endpoint_id, DoorLock::Id, DoorLock::Attributes::LockState::Id, &value);
}

esp_err_t update_voice_trigger_attribute(bool on)
{
    // 보조 스위치 endpoint가 아직 생성되지 않은 상태에서 호출되면 잘못된 상태입니다.
    if (g_voice_trigger_endpoint_id == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    // 음성 트리거 스위치는 "켜짐"을 잠깐 열기 명령으로 사용합니다.
    // 자동 재잠금이 끝난 뒤에는 다시 false로 갱신해서 Google Home UI도 꺼짐 상태로 되돌립니다.
    esp_matter_attr_val_t value = esp_matter_bool(on);
    return attribute::update(g_voice_trigger_endpoint_id, OnOff::Id, OnOff::Attributes::OnOff::Id, &value);
}

esp_err_t move_servo_for_lock_state(LockState state)
{
    // Matter LockState를 실제 서보 동작으로 변환하는 중앙 함수입니다.
    // DoorLock 명령, DoorLock attribute 변경, 자동 재잠금 등 여러 경로에서 이 함수 또는
    // g_servo_lock.lock()/unlock()을 통해 동일한 물리 동작을 수행합니다.
    switch (state) {
    case LockState::Locked:
        return g_servo_lock.lock();
    case LockState::Unlocked:
    case LockState::Unlatched:
        return g_servo_lock.unlock();
    default:
        // NotFullyLocked, Unknown 같은 상태는 실제 서보 위치를 새로 정하기 애매하므로 무시합니다.
        ESP_LOGW(TAG, "Ignoring unsupported lock state %u", static_cast<unsigned>(state));
        return ESP_OK;
    }
}

void auto_relock_work(intptr_t)
{
    // esp_timer callback은 타이머 task 문맥에서 호출됩니다.
    // 실제 Matter attribute update는 CHIP platform work queue에서 처리하는 것이 안전하므로
    // auto_relock_timer_cb()에서 이 함수를 ScheduleWork로 예약합니다.
    ESP_LOGI(TAG, "Voice trigger auto-relock after 5 seconds");

    // 5초가 지나면 물리 서보를 먼저 잠김 위치로 보냅니다.
    esp_err_t lock_err = g_servo_lock.lock();
    if (lock_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to auto-relock servo: %s", esp_err_to_name(lock_err));
        return;
    }

    // 서보가 잠겼다는 사실을 DoorLock endpoint의 상태에도 반영합니다.
    // 이 값이 Google Home의 도어락 타일 상태 표시와 연결됩니다.
    esp_err_t lock_state_err = update_lock_state_attribute(LockState::Locked);
    if (lock_state_err != ESP_OK) {
        ESP_LOGW(TAG, "Auto-relocked servo, but DoorLock state update failed: %s",
                 esp_err_to_name(lock_state_err));
    }

    // 음성 트리거 스위치도 다시 꺼짐으로 돌려놓습니다.
    // 이렇게 해야 다음에 "스위치 켜줘" 명령을 다시 보낼 수 있고,
    // Google Home UI에서도 순간 버튼처럼 동작합니다.
    esp_err_t trigger_state_err = update_voice_trigger_attribute(false);
    if (trigger_state_err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to reset voice trigger switch state: %s", esp_err_to_name(trigger_state_err));
    }
}

void auto_relock_timer_cb(void *)
{
    // esp_timer callback 안에서 직접 Matter attribute를 만지지 않고,
    // CHIP platform work queue로 넘깁니다.
    chip::DeviceLayer::PlatformMgr().ScheduleWork(auto_relock_work, 0);
}

esp_err_t schedule_auto_relock()
{
    // 타이머는 app_main()에서 한 번 생성해 둡니다.
    // 생성 전 호출되면 코드 흐름상 오류이므로 ESP_ERR_INVALID_STATE를 반환합니다.
    if (g_auto_relock_timer == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    // 이미 자동 재잠금 타이머가 돌고 있다면 먼저 멈춥니다.
    // 예를 들어 사용자가 5초 안에 스위치를 다시 켰을 때 타이머를 새로 5초로 연장하는 효과가 있습니다.
    esp_err_t stop_err = esp_timer_stop(g_auto_relock_timer);
    if (stop_err != ESP_OK && stop_err != ESP_ERR_INVALID_STATE) {
        return stop_err;
    }

    // one-shot 타이머로 5초 뒤 auto_relock_timer_cb()가 한 번 호출됩니다.
    return esp_timer_start_once(g_auto_relock_timer, kVoiceTriggerRelockDelayUs);
}

esp_err_t handle_voice_trigger(bool on)
{
    // On/Off Plug-in Unit endpoint의 OnOff attribute가 바뀔 때 호출됩니다.
    //
    // on == true:
    //   Google Home 또는 Nest Mini에서 보조 스위치를 켠 상황입니다.
    //   서보를 열림 위치로 보내고 5초 자동 재잠금을 예약합니다.
    //
    // on == false:
    //   사용자가 스위치를 끄거나, 앱에서 꺼짐 상태가 들어온 상황입니다.
    //   자동 재잠금을 취소하고 즉시 잠김 위치로 보냅니다.
    ESP_LOGI(TAG, "Voice trigger switch command: %s", on ? "open" : "close");

    if (!on) {
        // 사용자가 명시적으로 스위치를 끈 경우에는 예정된 자동 재잠금 타이머가 있다면 취소합니다.
        esp_err_t stop_err = esp_timer_stop(g_auto_relock_timer);
        if (stop_err != ESP_OK && stop_err != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "Failed to stop auto-relock timer: %s", esp_err_to_name(stop_err));
        }

        // 꺼짐 명령은 "닫기/잠그기"로 해석합니다.
        ESP_RETURN_ON_ERROR(g_servo_lock.lock(), TAG, "Failed to lock from voice trigger off command");
        return update_lock_state_attribute(LockState::Locked);
    }

    // 켜짐 명령은 "잠깐 열기"로 해석합니다.
    ESP_RETURN_ON_ERROR(g_servo_lock.unlock(), TAG, "Failed to unlock from voice trigger on command");

    // DoorLock endpoint도 Unlocked 상태로 갱신합니다.
    // Google Home 앱에서 도어락 타일을 함께 보고 있을 때 상태가 맞도록 하기 위한 처리입니다.
    esp_err_t lock_state_err = update_lock_state_attribute(LockState::Unlocked);
    if (lock_state_err != ESP_OK) {
        ESP_LOGW(TAG, "Voice trigger unlocked servo, but DoorLock state update failed: %s",
                 esp_err_to_name(lock_state_err));
    }

    // 5초 뒤 자동으로 다시 잠그도록 타이머를 예약합니다.
    ESP_RETURN_ON_ERROR(schedule_auto_relock(), TAG, "Failed to schedule auto-relock");
    return ESP_OK;
}

LockState to_app_lock_state(DlLockState state)
{
    // ESP-Matter Door Lock 서버 콜백에서 넘겨주는 DlLockState를
    // 이 파일에서 사용하는 LockState enum으로 변환합니다.
    switch (state) {
    case DlLockState::kLocked:
        return LockState::Locked;
    case DlLockState::kUnlocked:
        return LockState::Unlocked;
    case DlLockState::kUnlatched:
        return LockState::Unlatched;
    case DlLockState::kNotFullyLocked:
        return LockState::NotFullyLocked;
    default:
        return LockState::Unknown;
    }
}

bool set_lock_state_from_matter_command(chip::EndpointId endpoint_id, DlLockState state,
                                        const Nullable<chip::FabricIndex> &fabric_idx,
                                        const Nullable<chip::NodeId> &node_id,
                                        OperationErrorEnum &err)
{
    // DoorLock 명령은 반드시 도어락 endpoint로 들어와야 합니다.
    // 다른 endpoint로 들어온 명령은 잘못된 명령으로 보고 무시합니다.
    if (endpoint_id != g_lock_endpoint_id) {
        err = OperationErrorEnum::kUnspecified;
        ESP_LOGW(TAG, "Ignoring DoorLock command for unexpected endpoint=%u", endpoint_id);
        return false;
    }

    // Matter DoorLock 명령의 상태 값을 애플리케이션 상태로 변환한 뒤,
    // 먼저 실제 서보를 움직입니다.
    LockState app_state = to_app_lock_state(state);
    ESP_LOGI(TAG, "Applying DoorLock command endpoint=%u state=%u app_state=%u", endpoint_id,
             static_cast<unsigned>(state), static_cast<unsigned>(app_state));
    if (move_servo_for_lock_state(app_state) != ESP_OK) {
        err = OperationErrorEnum::kUnspecified;
        ESP_LOGE(TAG, "Failed to move servo for Matter lock state %u", static_cast<unsigned>(state));
        return false;
    }

    // 서보 동작이 성공하면 Matter attribute도 같은 상태로 갱신합니다.
    // 이 갱신은 Google Home UI의 상태 표시와 subscription report에 영향을 줍니다.
    esp_err_t update_err = update_lock_state_attribute(app_state);
    if (update_err != ESP_OK) {
        ESP_LOGW(TAG, "Servo moved, but Matter lock state update failed: %s", esp_err_to_name(update_err));
    }

    // 현재 구현에서는 fabric/node id와 PIN code를 별도로 검증하지 않습니다.
    // Google Home이 Matter 세션 보안을 통해 이미 인증된 controller로 접근한다고 가정합니다.
    (void)fabric_idx;
    (void)node_id;
    err = OperationErrorEnum::kUnspecified;
    return true;
}

void app_event_cb(const ChipDeviceEvent *event, intptr_t)
{
    // Matter stack에서 발생하는 주요 장치 이벤트를 받는 콜백입니다.
    // 여기서는 commissioning 완료, 실패, fabric 삭제 상황만 처리합니다.
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Matter commissioning complete");
        break;
    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGW(TAG, "Matter commissioning failed: fail-safe timer expired");
        break;
    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        ESP_LOGI(TAG, "Matter fabric removed");
        // Google Home에서 장치를 삭제해 fabric이 사라지면,
        // 다시 페어링할 수 있도록 commissioning window를 열어 줍니다.
        open_commissioning_window_if_needed();
        break;
    default:
        break;
    }
}

const char *wifi_disconnect_reason_to_name(uint8_t reason)
{
    switch (reason) {
    case WIFI_REASON_AUTH_EXPIRE:
        return "AUTH_EXPIRE";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
        return "4WAY_HANDSHAKE_TIMEOUT";
    case WIFI_REASON_NO_AP_FOUND:
        return "NO_AP_FOUND";
    case WIFI_REASON_AUTH_FAIL:
        return "AUTH_FAIL";
    case WIFI_REASON_ASSOC_FAIL:
        return "ASSOC_FAIL";
    case WIFI_REASON_HANDSHAKE_TIMEOUT:
        return "HANDSHAKE_TIMEOUT";
    case WIFI_REASON_CONNECTION_FAIL:
        return "CONNECTION_FAIL";
    case WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY:
        return "NO_AP_FOUND_WITH_COMPATIBLE_SECURITY";
    case WIFI_REASON_NO_AP_FOUND_IN_AUTHMODE_THRESHOLD:
        return "NO_AP_FOUND_IN_AUTHMODE_THRESHOLD";
    case WIFI_REASON_NO_AP_FOUND_IN_RSSI_THRESHOLD:
        return "NO_AP_FOUND_IN_RSSI_THRESHOLD";
    default:
        return "UNKNOWN";
    }
}

uint32_t hash_bytes(const uint8_t *data, size_t length)
{
    uint32_t hash = 2166136261UL;
    for (size_t i = 0; i < length; ++i) {
        hash ^= data[i];
        hash *= 16777619UL;
    }
    return hash;
}

void log_current_wifi_config()
{
    wifi_config_t config = {};
    esp_err_t err = esp_wifi_get_config(WIFI_IF_STA, &config);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Unable to read current Wi-Fi STA config: %s", esp_err_to_name(err));
        return;
    }

    const size_t ssid_len = strnlen(reinterpret_cast<const char *>(config.sta.ssid),
                                    sizeof(config.sta.ssid));
    const size_t password_len = strnlen(reinterpret_cast<const char *>(config.sta.password),
                                        sizeof(config.sta.password));
    const uint32_t password_hash = hash_bytes(config.sta.password, password_len);

    ESP_LOGI(TAG, "Current Wi-Fi config: ssid='%.*s' ssid_len=%u password_len=%u password_hash=0x%08lx",
             static_cast<int>(ssid_len),
             reinterpret_cast<const char *>(config.sta.ssid),
             static_cast<unsigned>(ssid_len),
             static_cast<unsigned>(password_len),
             static_cast<unsigned long>(password_hash));
}

void apply_wifi_sta_tuning()
{
    // Matter 장치는 부팅 직후 저장된 Wi-Fi에 자동 재접속합니다.
    // 일부 공유기/ESP32-C3 조합에서는 기본 절전 상태나 HT40 협상 중 auth expire가 반복될 수 있어
    // STA 동작을 보수적으로 고정합니다.
    esp_err_t ps_err = esp_wifi_set_ps(WIFI_PS_NONE);
    if (ps_err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to disable Wi-Fi power save: %s", esp_err_to_name(ps_err));
    }

    esp_err_t bw_err = esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
    if (bw_err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to force Wi-Fi HT20 bandwidth: %s", esp_err_to_name(bw_err));
    }

    ESP_LOGI(TAG, "Wi-Fi STA tuning applied: power_save=off bandwidth=HT20");
}

void wifi_debug_event_handler(void *, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base != WIFI_EVENT) {
        return;
    }

    if (event_id == WIFI_EVENT_STA_START) {
        apply_wifi_sta_tuning();
        return;
    }

    if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        const auto *event = static_cast<wifi_event_sta_disconnected_t *>(event_data);
        ESP_LOGW(TAG, "Wi-Fi disconnected: ssid='%.*s' bssid=%02x:%02x:%02x:%02x:%02x:%02x reason=%u (%s) rssi=%d",
                 event->ssid_len,
                 reinterpret_cast<const char *>(event->ssid),
                 event->bssid[0], event->bssid[1], event->bssid[2],
                 event->bssid[3], event->bssid[4], event->bssid[5],
                 event->reason,
                 wifi_disconnect_reason_to_name(event->reason),
                 event->rssi);
        apply_wifi_sta_tuning();
        log_current_wifi_config();
        return;
    }

    if (event_id == WIFI_EVENT_STA_CONNECTED) {
        const auto *event = static_cast<wifi_event_sta_connected_t *>(event_data);
        ESP_LOGI(TAG, "Wi-Fi connected: ssid='%.*s' channel=%u authmode=%u",
                 event->ssid_len,
                 reinterpret_cast<const char *>(event->ssid),
                 event->channel,
                 event->authmode);
        log_current_wifi_config();
    }
}

esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                uint8_t effect_variant, void *)
{
    // Google Home 앱에서 "기기 식별" 같은 동작을 실행할 때 호출될 수 있습니다.
    // 이 프로젝트에는 별도 LED 식별 효과가 없으므로 로그만 남깁니다.
    ESP_LOGI(TAG, "Identify endpoint=%u type=%u effect=%u variant=%u", endpoint_id, type, effect_id, effect_variant);
    return ESP_OK;
}

esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                  uint32_t attribute_id, esp_matter_attr_val_t *value, void *)
{
    // esp-matter는 attribute가 변경되기 전(PRE_UPDATE)과 변경된 후(POST_UPDATE)에 콜백을 줄 수 있습니다.
    // 실제 서보 동작은 상태가 저장되기 전에 실패 여부를 반영할 수 있도록 PRE_UPDATE에서 처리합니다.
    if (type != PRE_UPDATE) {
        return ESP_OK;
    }

    // DoorLock endpoint의 LockState attribute가 바뀌면 서보 위치도 맞춰 줍니다.
    // Google Home 앱에서 도어락 타일을 직접 눌렀을 때 이 경로가 사용될 수 있습니다.
    if (endpoint_id == g_lock_endpoint_id &&
        cluster_id == DoorLock::Id && attribute_id == DoorLock::Attributes::LockState::Id) {
        const auto requested_state = static_cast<LockState>(value->val.u8);
        return move_servo_for_lock_state(requested_state);
    }

    // 보조 On/Off 스위치 endpoint의 OnOff attribute가 바뀌면
    // "켜짐 = 잠깐 열기", "꺼짐 = 닫기"로 해석합니다.
    if (endpoint_id == g_voice_trigger_endpoint_id &&
        cluster_id == OnOff::Id && attribute_id == OnOff::Attributes::OnOff::Id) {
        return handle_voice_trigger(value->val.b);
    }

    return ESP_OK;
}

} // namespace

bool emberAfPluginDoorLockOnDoorLockCommand(chip::EndpointId endpoint_id,
                                            const Nullable<chip::FabricIndex> &fabric_idx,
                                            const Nullable<chip::NodeId> &node_id,
                                            const Optional<chip::ByteSpan> &pin_code,
                                            OperationErrorEnum &err)
{
    // Door Lock cluster의 LockDoor command가 들어오면 esp-matter가 이 weak callback을 호출합니다.
    // 여기서는 PIN 코드를 별도로 확인하지 않고, Matter 세션을 통해 인증된 controller의 명령으로 간주합니다.
    (void)pin_code;
    ESP_LOGI(TAG, "Matter LockDoor command received");
    return set_lock_state_from_matter_command(endpoint_id, DlLockState::kLocked, fabric_idx, node_id, err);
}

bool emberAfPluginDoorLockOnDoorUnlockCommand(chip::EndpointId endpoint_id,
                                              const Nullable<chip::FabricIndex> &fabric_idx,
                                              const Nullable<chip::NodeId> &node_id,
                                              const Optional<chip::ByteSpan> &pin_code,
                                              OperationErrorEnum &err)
{
    // Door Lock cluster의 UnlockDoor command가 들어왔을 때 호출됩니다.
    // Google Assistant 스피커는 보안 정책상 이 명령을 음성으로 직접 보내지 않지만,
    // Google Home 앱의 도어락 타일에서는 이 경로가 사용될 수 있습니다.
    (void)pin_code;
    ESP_LOGI(TAG, "Matter UnlockDoor command received");
    return set_lock_state_from_matter_command(endpoint_id, DlLockState::kUnlocked, fabric_idx, node_id, err);
}

bool emberAfPluginDoorLockOnDoorUnboltCommand(chip::EndpointId endpoint_id,
                                              const Nullable<chip::FabricIndex> &fabric_idx,
                                              const Nullable<chip::NodeId> &node_id,
                                              const Optional<chip::ByteSpan> &pin_code,
                                              OperationErrorEnum &err)
{
    // 일부 controller는 UnlockDoor 대신 UnboltDoor command를 사용할 수 있습니다.
    // 이 프로젝트에서는 Unbolt도 물리적으로는 "열림 위치로 이동"과 동일하게 처리합니다.
    (void)pin_code;
    ESP_LOGI(TAG, "Matter UnboltDoor command received");
    return set_lock_state_from_matter_command(endpoint_id, DlLockState::kUnlocked, fabric_idx, node_id, err);
}

void emberAfPluginDoorLockOnAutoRelock(chip::EndpointId endpoint_id)
{
    // Matter Door Lock cluster 자체의 AutoRelock 이벤트가 발생했을 때 호출될 수 있는 hook입니다.
    // 현재 실제 자동 재잠금은 보조 스위치용 esp_timer에서 직접 처리하므로 여기서는 로그만 남깁니다.
    ESP_LOGI(TAG, "Matter auto relock fired for endpoint=%u", endpoint_id);
}

extern "C" void app_main()
{
    // NVS는 Wi-Fi credential, Matter fabric, commissioning 정보 등을 저장하는 플래시 영역입니다.
    // NVS가 비어 있거나 버전이 맞지 않으면 지우고 다시 초기화합니다.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // 서보 PWM을 먼저 초기화합니다.
    // 부팅 중 잠김 -> 열림 -> 잠김 테스트 동작이 수행되므로,
    // 하드웨어 배선과 전원이 맞는지 초기 로그/동작으로 확인할 수 있습니다.
    ESP_ERROR_CHECK(g_servo_lock.init());

    // 보조 음성 트리거가 켜졌을 때 5초 뒤 자동 잠금을 수행할 타이머를 생성합니다.
    // 실제 시작은 handle_voice_trigger(true)에서 esp_timer_start_once()로 합니다.
    const esp_timer_create_args_t auto_relock_timer_args = {
        .callback = &auto_relock_timer_cb,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "voice_relock",
        .skip_unhandled_events = true,
    };
    ESP_ERROR_CHECK(esp_timer_create(&auto_relock_timer_args, &g_auto_relock_timer));

    // Matter node는 여러 endpoint를 담는 루트 객체입니다.
    // app_attribute_update_cb는 attribute 변경 시 호출되고,
    // app_identification_cb는 Identify cluster 동작 시 호출됩니다.
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ESP_ERROR_CHECK(node == nullptr ? ESP_FAIL : ESP_OK);

    // 표준 Door Lock endpoint 설정입니다.
    // 초기 상태는 Locked로 두고, 물리 액추에이터가 있는 도어락임을 actuator_enabled로 표시합니다.
    door_lock::config_t lock_config;
    lock_config.door_lock.lock_state = static_cast<uint8_t>(LockState::Locked);
    lock_config.door_lock.lock_type = 0;
    lock_config.door_lock.actuator_enabled = true;

    // Door Lock endpoint를 생성하고 실제 endpoint id를 저장합니다.
    endpoint_t *endpoint = door_lock::create(node, &lock_config, ENDPOINT_FLAG_NONE, nullptr);
    ESP_ERROR_CHECK(endpoint == nullptr ? ESP_FAIL : ESP_OK);
    g_lock_endpoint_id = endpoint::get_id(endpoint);

    // esp-matter의 기본 door_lock endpoint에는 DoorState attribute가 없는 경우가 있어
    // Google Home과 DoorLock server 내부 처리가 실패할 수 있습니다.
    // 여기서는 DoorState를 명시적으로 추가하고, 1(door closed)로 둡니다.
    cluster_t *lock_cluster = cluster::get(endpoint, DoorLock::Id);
    ESP_ERROR_CHECK(lock_cluster == nullptr ? ESP_FAIL : ESP_OK);
    esp_matter::cluster::door_lock::attribute::create_door_state(lock_cluster, nullable<uint8_t>(1));

    // Matter attribute 상태도 초기 잠김 상태로 맞춰 둡니다.
    ESP_ERROR_CHECK(update_lock_state_attribute(LockState::Locked));
    ESP_LOGI(TAG, "Matter Door Lock endpoint created: endpoint=%u", g_lock_endpoint_id);

    // Google Assistant/Nest Mini는 Door Lock의 음성 잠금해제를 허용하지 않습니다.
    // 이를 우회하기 위해 별도 On/Off Plug-in Unit endpoint를 만들고,
    // 이 스위치의 "켜짐"을 "문을 5초 동안 열기" 트리거로 사용합니다.
    on_off_plugin_unit::config_t voice_trigger_config;
    voice_trigger_config.on_off.on_off = false;

    // 보조 스위치 endpoint를 생성하고 endpoint id를 저장합니다.
    // Google Home 앱에서는 도어락과 별개의 스위치/플러그류 기기로 보일 수 있습니다.
    endpoint_t *voice_trigger_endpoint =
        on_off_plugin_unit::create(node, &voice_trigger_config, ENDPOINT_FLAG_NONE, nullptr);
    ESP_ERROR_CHECK(voice_trigger_endpoint == nullptr ? ESP_FAIL : ESP_OK);
    g_voice_trigger_endpoint_id = endpoint::get_id(voice_trigger_endpoint);
    ESP_LOGI(TAG, "Matter voice trigger switch endpoint created: endpoint=%u", g_voice_trigger_endpoint_id);

    // Matter stack을 시작합니다.
    // 이후 Wi-Fi commissioning, CASE session, Google Home subscription 등이 Matter stack 내부에서 동작합니다.
    ESP_ERROR_CHECK(esp_matter::start(app_event_cb));

    // Matter NetworkCommissioning이 실제 Wi-Fi 접속을 시도할 때 ESP-IDF가 제공하는 상세 끊김 사유를 남깁니다.
    // Google Home 앱에는 "액세서리를 추가할 수 없음"처럼 뭉뚱그려 보이기 때문에,
    // 여기서 reason 값을 봐야 비밀번호/보안방식/신호/공유기 거절 중 어느 쪽인지 구분할 수 있습니다.
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED,
                                               &wifi_debug_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED,
                                               &wifi_debug_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_START,
                                               &wifi_debug_event_handler, nullptr));
    apply_wifi_sta_tuning();

    // Matter stack 시작 직후, 아직 fabric이 없으면 commissioning window를 열도록 예약합니다.
    // 이미 Google Home에 등록된 장치라면 open_commissioning_window_if_needed()에서 아무 일도 하지 않습니다.
    chip::DeviceLayer::PlatformMgr().ScheduleWork(open_commissioning_window_work, 0);
}
