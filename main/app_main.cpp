#include "servo_lock.h"

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_matter.h"
#include "esp_matter_attribute.h"
#include "esp_matter_cluster.h"
#include "esp_matter_endpoint.h"
#include "esp_timer.h"
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
constexpr char TAG[] = "matter_lock";
constexpr uint32_t kCommissioningWindowTimeoutSeconds = 300;
constexpr uint64_t kVoiceTriggerRelockDelayUs = 5ULL * 1000ULL * 1000ULL;

enum class LockState : uint8_t {
    NotFullyLocked = 0,
    Locked = 1,
    Unlocked = 2,
    Unlatched = 3,
    Unknown = 4,
};

ServoLock g_servo_lock;
uint16_t g_lock_endpoint_id = 0;
uint16_t g_voice_trigger_endpoint_id = 0;
esp_timer_handle_t g_auto_relock_timer = nullptr;

void open_commissioning_window_if_needed()
{
    if (chip::Server::GetInstance().GetFabricTable().FabricCount() != 0) {
        return;
    }

    auto &commissioning = chip::Server::GetInstance().GetCommissioningWindowManager();
    if (commissioning.IsCommissioningWindowOpen()) {
        return;
    }

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
    open_commissioning_window_if_needed();
}

esp_err_t update_lock_state_attribute(LockState state)
{
    esp_matter_attr_val_t value = esp_matter_enum8(static_cast<uint8_t>(state));
    return attribute::update(g_lock_endpoint_id, DoorLock::Id, DoorLock::Attributes::LockState::Id, &value);
}

esp_err_t update_voice_trigger_attribute(bool on)
{
    if (g_voice_trigger_endpoint_id == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_matter_attr_val_t value = esp_matter_bool(on);
    return attribute::update(g_voice_trigger_endpoint_id, OnOff::Id, OnOff::Attributes::OnOff::Id, &value);
}

esp_err_t move_servo_for_lock_state(LockState state)
{
    switch (state) {
    case LockState::Locked:
        return g_servo_lock.lock();
    case LockState::Unlocked:
    case LockState::Unlatched:
        return g_servo_lock.unlock();
    default:
        ESP_LOGW(TAG, "Ignoring unsupported lock state %u", static_cast<unsigned>(state));
        return ESP_OK;
    }
}

void auto_relock_work(intptr_t)
{
    ESP_LOGI(TAG, "Voice trigger auto-relock after 5 seconds");

    esp_err_t lock_err = g_servo_lock.lock();
    if (lock_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to auto-relock servo: %s", esp_err_to_name(lock_err));
        return;
    }

    esp_err_t lock_state_err = update_lock_state_attribute(LockState::Locked);
    if (lock_state_err != ESP_OK) {
        ESP_LOGW(TAG, "Auto-relocked servo, but DoorLock state update failed: %s",
                 esp_err_to_name(lock_state_err));
    }

    esp_err_t trigger_state_err = update_voice_trigger_attribute(false);
    if (trigger_state_err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to reset voice trigger switch state: %s", esp_err_to_name(trigger_state_err));
    }
}

void auto_relock_timer_cb(void *)
{
    chip::DeviceLayer::PlatformMgr().ScheduleWork(auto_relock_work, 0);
}

esp_err_t schedule_auto_relock()
{
    if (g_auto_relock_timer == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t stop_err = esp_timer_stop(g_auto_relock_timer);
    if (stop_err != ESP_OK && stop_err != ESP_ERR_INVALID_STATE) {
        return stop_err;
    }

    return esp_timer_start_once(g_auto_relock_timer, kVoiceTriggerRelockDelayUs);
}

esp_err_t handle_voice_trigger(bool on)
{
    ESP_LOGI(TAG, "Voice trigger switch command: %s", on ? "open" : "close");

    if (!on) {
        esp_err_t stop_err = esp_timer_stop(g_auto_relock_timer);
        if (stop_err != ESP_OK && stop_err != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "Failed to stop auto-relock timer: %s", esp_err_to_name(stop_err));
        }

        ESP_RETURN_ON_ERROR(g_servo_lock.lock(), TAG, "Failed to lock from voice trigger off command");
        return update_lock_state_attribute(LockState::Locked);
    }

    ESP_RETURN_ON_ERROR(g_servo_lock.unlock(), TAG, "Failed to unlock from voice trigger on command");

    esp_err_t lock_state_err = update_lock_state_attribute(LockState::Unlocked);
    if (lock_state_err != ESP_OK) {
        ESP_LOGW(TAG, "Voice trigger unlocked servo, but DoorLock state update failed: %s",
                 esp_err_to_name(lock_state_err));
    }

    ESP_RETURN_ON_ERROR(schedule_auto_relock(), TAG, "Failed to schedule auto-relock");
    return ESP_OK;
}

LockState to_app_lock_state(DlLockState state)
{
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
    if (endpoint_id != g_lock_endpoint_id) {
        err = OperationErrorEnum::kUnspecified;
        ESP_LOGW(TAG, "Ignoring DoorLock command for unexpected endpoint=%u", endpoint_id);
        return false;
    }

    LockState app_state = to_app_lock_state(state);
    ESP_LOGI(TAG, "Applying DoorLock command endpoint=%u state=%u app_state=%u", endpoint_id,
             static_cast<unsigned>(state), static_cast<unsigned>(app_state));
    if (move_servo_for_lock_state(app_state) != ESP_OK) {
        err = OperationErrorEnum::kUnspecified;
        ESP_LOGE(TAG, "Failed to move servo for Matter lock state %u", static_cast<unsigned>(state));
        return false;
    }

    esp_err_t update_err = update_lock_state_attribute(app_state);
    if (update_err != ESP_OK) {
        ESP_LOGW(TAG, "Servo moved, but Matter lock state update failed: %s", esp_err_to_name(update_err));
    }

    (void)fabric_idx;
    (void)node_id;
    err = OperationErrorEnum::kUnspecified;
    return true;
}

void app_event_cb(const ChipDeviceEvent *event, intptr_t)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Matter commissioning complete");
        break;
    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGW(TAG, "Matter commissioning failed: fail-safe timer expired");
        break;
    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        ESP_LOGI(TAG, "Matter fabric removed");
        open_commissioning_window_if_needed();
        break;
    default:
        break;
    }
}

esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                uint8_t effect_variant, void *)
{
    ESP_LOGI(TAG, "Identify endpoint=%u type=%u effect=%u variant=%u", endpoint_id, type, effect_id, effect_variant);
    return ESP_OK;
}

esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                  uint32_t attribute_id, esp_matter_attr_val_t *value, void *)
{
    if (type != PRE_UPDATE) {
        return ESP_OK;
    }

    if (endpoint_id == g_lock_endpoint_id &&
        cluster_id == DoorLock::Id && attribute_id == DoorLock::Attributes::LockState::Id) {
        const auto requested_state = static_cast<LockState>(value->val.u8);
        return move_servo_for_lock_state(requested_state);
    }

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
    (void)pin_code;
    ESP_LOGI(TAG, "Matter UnboltDoor command received");
    return set_lock_state_from_matter_command(endpoint_id, DlLockState::kUnlocked, fabric_idx, node_id, err);
}

void emberAfPluginDoorLockOnAutoRelock(chip::EndpointId endpoint_id)
{
    ESP_LOGI(TAG, "Matter auto relock fired for endpoint=%u", endpoint_id);
}

extern "C" void app_main()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(g_servo_lock.init());

    const esp_timer_create_args_t auto_relock_timer_args = {
        .callback = &auto_relock_timer_cb,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "voice_relock",
        .skip_unhandled_events = true,
    };
    ESP_ERROR_CHECK(esp_timer_create(&auto_relock_timer_args, &g_auto_relock_timer));

    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ESP_ERROR_CHECK(node == nullptr ? ESP_FAIL : ESP_OK);

    door_lock::config_t lock_config;
    lock_config.door_lock.lock_state = static_cast<uint8_t>(LockState::Locked);
    lock_config.door_lock.lock_type = 0;
    lock_config.door_lock.actuator_enabled = true;

    endpoint_t *endpoint = door_lock::create(node, &lock_config, ENDPOINT_FLAG_NONE, nullptr);
    ESP_ERROR_CHECK(endpoint == nullptr ? ESP_FAIL : ESP_OK);
    g_lock_endpoint_id = endpoint::get_id(endpoint);

    cluster_t *lock_cluster = cluster::get(endpoint, DoorLock::Id);
    ESP_ERROR_CHECK(lock_cluster == nullptr ? ESP_FAIL : ESP_OK);
    esp_matter::cluster::door_lock::attribute::create_door_state(lock_cluster, nullable<uint8_t>(1));

    ESP_ERROR_CHECK(update_lock_state_attribute(LockState::Locked));
    ESP_LOGI(TAG, "Matter Door Lock endpoint created: endpoint=%u", g_lock_endpoint_id);

    on_off_plugin_unit::config_t voice_trigger_config;
    voice_trigger_config.on_off.on_off = false;

    endpoint_t *voice_trigger_endpoint =
        on_off_plugin_unit::create(node, &voice_trigger_config, ENDPOINT_FLAG_NONE, nullptr);
    ESP_ERROR_CHECK(voice_trigger_endpoint == nullptr ? ESP_FAIL : ESP_OK);
    g_voice_trigger_endpoint_id = endpoint::get_id(voice_trigger_endpoint);
    ESP_LOGI(TAG, "Matter voice trigger switch endpoint created: endpoint=%u", g_voice_trigger_endpoint_id);

    ESP_ERROR_CHECK(esp_matter::start(app_event_cb));
    chip::DeviceLayer::PlatformMgr().ScheduleWork(open_commissioning_window_work, 0);
}
