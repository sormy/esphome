#include "radon_eye_rd200.h"

#ifdef USE_ESP32

namespace esphome {
namespace radon_eye_rd200 {

static const char *const TAG = "radon_eye_rd200";

static const esp32_ble_tracker::ESPBTUUID SERVICE_UUID_V1 =
    esp32_ble_tracker::ESPBTUUID::from_raw("00001523-1212-efde-1523-785feabcd123");
static const esp32_ble_tracker::ESPBTUUID WRITE_CHARACTERISTIC_UUID_V1 =
    esp32_ble_tracker::ESPBTUUID::from_raw("00001524-1212-efde-1523-785feabcd123");
static const esp32_ble_tracker::ESPBTUUID READ_CHARACTERISTIC_UUID_V1 =
    esp32_ble_tracker::ESPBTUUID::from_raw("00001525-1212-efde-1523-785feabcd123");
static const uint8_t WRITE_COMMAND_V1 = 0x50;

static const esp32_ble_tracker::ESPBTUUID SERVICE_UUID_V2 =
    esp32_ble_tracker::ESPBTUUID::from_raw("00001523-0000-1000-8000-00805f9b34fb");
static const esp32_ble_tracker::ESPBTUUID WRITE_CHARACTERISTIC_UUID_V2 =
    esp32_ble_tracker::ESPBTUUID::from_raw("00001524-0000-1000-8000-00805f9b34fb");
static const esp32_ble_tracker::ESPBTUUID READ_CHARACTERISTIC_UUID_V2 =
    esp32_ble_tracker::ESPBTUUID::from_raw("00001525-0000-1000-8000-00805f9b34fb");
static const uint8_t WRITE_COMMAND_V2 = 0x40;

void RadonEyeRD200::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                        esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_OPEN_EVT: {
      if (param->open.status == ESP_GATT_OK) {
        ESP_LOGI(TAG, "Connected successfully!");
      }
      break;
    }

    case ESP_GATTC_DISCONNECT_EVT: {
      ESP_LOGW(TAG, "Disconnected!");
      break;
    }

    case ESP_GATTC_SEARCH_CMPL_EVT: {
      if (this->parent()->get_service(SERVICE_UUID_V1) != nullptr) {
        service_uuid_ = SERVICE_UUID_V1;
        sensors_write_characteristic_uuid_ = WRITE_CHARACTERISTIC_UUID_V1;
        sensors_read_characteristic_uuid_ = READ_CHARACTERISTIC_UUID_V1;
        write_command_ = WRITE_COMMAND_V1;
      } else if (this->parent()->get_service(SERVICE_UUID_V2) != nullptr) {
        service_uuid_ = SERVICE_UUID_V2;
        sensors_write_characteristic_uuid_ = WRITE_CHARACTERISTIC_UUID_V2;
        sensors_read_characteristic_uuid_ = READ_CHARACTERISTIC_UUID_V2;
        write_command_ = WRITE_COMMAND_V2;
      } else {
        ESP_LOGW(TAG, "No supported device has been found, disconnecting");
        parent()->set_enabled(false);
        break;
      }

      this->read_handle_ = 0;
      auto *chr = this->parent()->get_characteristic(service_uuid_, sensors_read_characteristic_uuid_);
      if (chr == nullptr) {
        ESP_LOGW(TAG, "No sensor read characteristic found at service %s char %s", service_uuid_.to_string().c_str(),
                 sensors_read_characteristic_uuid_.to_string().c_str());
        break;
      }
      this->read_handle_ = chr->handle;

      auto *write_chr = this->parent()->get_characteristic(service_uuid_, sensors_write_characteristic_uuid_);
      if (write_chr == nullptr) {
        ESP_LOGW(TAG, "No sensor write characteristic found at service %s char %s", service_uuid_.to_string().c_str(),
                 sensors_read_characteristic_uuid_.to_string().c_str());
        break;
      }
      this->write_handle_ = write_chr->handle;

      this->node_state = esp32_ble_tracker::ClientState::ESTABLISHED;

      request_read_values_();

      write_query_message_();

      break;
    }

    case ESP_GATTC_NOTIFY_EVT: {
      if (param->read.handle == this->read_handle_) {
        read_sensors_(param->read.value, param->read.value_len);
      }
      break;
    }

    default:
      break;
  }
}

void RadonEyeRD200::read_sensors_(uint8_t *value, uint16_t value_len) {
  if (value_len < 1) {
    ESP_LOGW(TAG, "Unexpected empty message");
    return;
  }

  uint8_t command = value[0];

  if ((command == WRITE_COMMAND_V1 && value_len < 20) || (command == WRITE_COMMAND_V2 && value_len < 68)) {
    ESP_LOGW(TAG, "Unexpected command 0x%02X message length %d", command, value_len);
    return;
  }

  // Example data V1:
  // 501085EBB9400000000000000000220025000000
  // Example data V2:
  // 4042323230313033525532303338330652443230304e56322e302e3200014a00060a00080000000300010079300000e01108001c00020000003822005c8f423fa4709d3f
  ESP_LOGV(TAG, "radon sensors raw bytes");
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, value, value_len, ESP_LOG_VERBOSE);

  // Convert from pCi/L to Bq/m³
  constexpr float convert_to_bwpm3 = 37.0;

  float radon_now;    // in Bq/m³
  float radon_day;    // in Bq/m³
  float radon_month;  // in Bq/m³
  if (command == WRITE_COMMAND_V1) {
    radon_now = *(float *) (value + 2) * convert_to_bwpm3;
    radon_day = *(float *) (value + 6) * convert_to_bwpm3;
    radon_month = *(float *) (value + 10) * convert_to_bwpm3;
  } else if (command == WRITE_COMMAND_V2) {
    radon_now = *(uint16_t *) (value + 33);
    radon_day = *(uint16_t *) (value + 35);
    radon_month = *(uint16_t *) (value + 37);
  } else {
    ESP_LOGW(TAG, "Unexpected command value: 0x%02X", command);
    return;
  }

  if (is_valid_radon_value_(radon_now)) {
    radon_sensor_->publish_state(radon_now);
  }

  if (is_valid_radon_value_(radon_month)) {
    ESP_LOGV(TAG, "Radon Long Term based on month");
    radon_long_term_sensor_->publish_state(radon_month);
  } else if (is_valid_radon_value_(radon_day)) {
    ESP_LOGV(TAG, "Radon Long Term based on day");
    radon_long_term_sensor_->publish_state(radon_day);
  }

  ESP_LOGV(TAG, "  Measurements (Bq/m³) now: %0.03f, day: %0.03f, month: %0.03f", radon_now, radon_day, radon_month);

  ESP_LOGV(TAG, "  Measurements (pCi/L) now: %0.03f, day: %0.03f, month: %0.03f", radon_now / convert_to_bwpm3,
           radon_day / convert_to_bwpm3, radon_month / convert_to_bwpm3);

  // This instance must not stay connected
  // so other clients can connect to it (e.g. the
  // mobile app).
  parent()->set_enabled(false);
}

bool RadonEyeRD200::is_valid_radon_value_(float radon) { return radon >= 0.0 and radon < 37000; }

void RadonEyeRD200::update() {
  if (this->node_state != esp32_ble_tracker::ClientState::ESTABLISHED) {
    if (!parent()->enabled) {
      ESP_LOGW(TAG, "Reconnecting to device");
      parent()->set_enabled(true);
      parent()->connect();
    } else {
      ESP_LOGW(TAG, "Connection in progress");
    }
  }
}

void RadonEyeRD200::write_query_message_() {
  ESP_LOGV(TAG, "writing 0x%02x to write service", write_command_);
  auto status = esp_ble_gattc_write_char_descr(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                               this->write_handle_, sizeof(write_command_), (uint8_t *) &write_command_,
                                               ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);
  if (status) {
    ESP_LOGW(TAG, "Error sending write request for sensor, status=%d", status);
  }
}

void RadonEyeRD200::request_read_values_() {
  auto status = esp_ble_gattc_register_for_notify(this->parent()->get_gattc_if(), this->parent()->get_remote_bda(),
                                                  this->read_handle_);
  if (status) {
    ESP_LOGW(TAG, "Error registering for sensor notify, status=%d", status);
  }
}

void RadonEyeRD200::dump_config() {
  LOG_SENSOR("  ", "Radon", this->radon_sensor_);
  LOG_SENSOR("  ", "Radon Long Term", this->radon_long_term_sensor_);
}

RadonEyeRD200::RadonEyeRD200() : PollingComponent(10000) {}

}  // namespace radon_eye_rd200
}  // namespace esphome

#endif  // USE_ESP32
