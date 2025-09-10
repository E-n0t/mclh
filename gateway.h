#pragma once

#ifdef USE_ESP32

#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include "esphome/components/mqtt/mqtt_client.h"
#include "esphome/components/myhomeiot_ble_client2/myhomeiot_ble_client2.h"
#include "esphome/components/myhomeiot_ble_host/myhomeiot_ble_host.h"
#include "esphome/core/application.h"
#include "esphome/core/base_automation.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences.h"
#include <algorithm>
#include <sstream>
#include <vector>

namespace esphome {
namespace mclh_09_mqtt_gateway {

// USE_ESP32_FRAMEWORK_ARDUINO
#ifdef USE_ESP_IDF
#define snprintf_P snprintf
#define strncat_P strncat
#define PSTR
#endif

const char *TAG = "mclh-09";
const char *ADVR_MODEL = "4D434C482D3039";

const std::vector<float> temp_input{1035, 909, 668, 424, 368, 273, 159, 0};
const std::vector<float> temp_output{68.8, 49.8, 24.3, 6.4, 1.0, -5.5, -20.5, -41.0};
const std::vector<float> soil_input{1254, 1249, 1202, 1104, 944, 900};
const std::vector<float> soil_output{60.0, 58.0, 54.0, 22.0, 2.0, 0.0};
const std::vector<float> lumi_input{911, 764, 741, 706, 645, 545, 196, 117, 24, 17, 0};
const std::vector<float> lumi_output{175300.0, 45400.0, 32100.0, 20300.0, 14760.0, 7600.0, 1200.0, 444.0, 29.0, 17.0, 0.0};

const char *available_suffix = "status", *state_suffix = "state", *alert_set_suffix = "alert/set", *alert_state_suffix = "alert";
const char *default_payload_online = "online", *default_payload_offline = "offline";

struct Sensor {
  const char *id, *name, *unit, *dev_class, *icon, *state_class;
  uint8_t accuracy;
  bool diag;
};

struct Select {
  const char *id, *name, *icon, *set_suffix, *state_suffix;
  bool diag, optimistic;
  const std::vector<std::string> &options;
};

const char *MEASURE = "measurement", *HUMIDITY = "humidity";
Sensor BATT{"batt", "Battery", "%", "battery", nullptr, MEASURE, 0, true};
Sensor TEMP{"temp", "Temperature", "°C", "temperature", nullptr, MEASURE, 1, false};
Sensor LUMI{"lumi", "Illuminance", "lx", "illuminance", nullptr, MEASURE, 0, false};
Sensor SOIL{"soil", "Soil moisture", "%", HUMIDITY, "mdi:water", MEASURE, 0, false};
Sensor HUMI{"humi", "Air humidity", "%", HUMIDITY, nullptr, MEASURE, 0, false};
Sensor RSSI{"rssi", "RSSI", "dBm", "signal_strength", "mdi:signal", nullptr, 0, true};
Sensor ERRORS{"errors", "Error count", nullptr, nullptr, "mdi:alert-circle", MEASURE, 0, true};

const std::vector<std::string> alert_options{"Off", "Red (once)", "Green (once)", "Red + green (once)", "Red (every update)", "Green (every update)", "Red + green (every update)", "Green (always)"};
Select ALERT{"alert", "Alert", "mdi:alarm-light", alert_set_suffix, alert_state_suffix, false, false, alert_options};
std::vector<Sensor *> mclh09_sensors;
std::vector<Select *> mclh09_selects{&ALERT};

class Mclh09Device : public Component {
 public:
  Mclh09Device(mqtt::MQTTClientComponent *mqtt_client, myhomeiot_ble_host::MyHomeIOT_BLEHost *ble_host, const std::string &topic_prefix,
               uint32_t update_interval, uint64_t address)
      : mqtt_client_(mqtt_client), ble_host_(ble_host), address_(address), update_interval_(update_interval) {
    char temp_buffer[64];
    snprintf_P(temp_buffer, sizeof(temp_buffer), PSTR("mclh09_%012llx"), address);
    id_ = temp_buffer;
    snprintf_P(temp_buffer, sizeof(temp_buffer), PSTR("MCLH-09 %06x"), (uint32_t)address & 0xFFFFFF);
    name_ = temp_buffer;

    const char *prefix = topic_prefix.c_str();
    const char *delimiter = (prefix[strlen(prefix) - 1] == '/' ? "" : "/");
    snprintf_P(temp_buffer, sizeof(temp_buffer), PSTR("%s%s%012llx"), prefix, delimiter, address);
    topic_prefix_ = temp_buffer;
    status_topic_ = topic_prefix_ + "/" + available_suffix;
    state_topic_ = topic_prefix_ + "/" + state_suffix;
    alert_state_topic_ = topic_prefix_ + "/" + alert_state_suffix;
    alert_set_topic_ = topic_prefix_ + "/" + alert_set_suffix;

    device_has_availability_ = !mqtt_client->get_availability().topic.empty() &&
                               !mqtt_client->get_availability().payload_available.empty() &&
                               !mqtt_client->get_availability().payload_not_available.empty();
    payload_online_ = mqtt_client->get_availability().payload_available.empty()
                          ? default_payload_online
                          : mqtt_client->get_availability().payload_available.c_str();
    payload_offline_ = mqtt_client->get_availability().payload_not_available.empty()
                           ? default_payload_offline
                           : mqtt_client->get_availability().payload_not_available.c_str();

    // BLE client
    ble_client_ = new myhomeiot_ble_client2::MyHomeIOT_BLEClient2();
    ble_client_->set_address(address);
    ble_client_->set_update_interval(update_interval);

    // Services
    auto serv_bat = std::make_unique<myhomeiot_ble_client2::MyHomeIOT_BLEClientService>();
    serv_bat->set_service_uuid16(0x180F);
    serv_bat->set_char_uuid16(0x2A19);
    ble_client_->add_service(serv_bat.release());

    auto serv_data = std::make_unique<myhomeiot_ble_client2::MyHomeIOT_BLEClientService>();
    serv_data->set_service_uuid128((uint8_t[]){0x1B, 0xC5, 0xD5, 0xA5, 0x02, 0x00, 0xB8, 0xAC, 0xE3, 0x11, 0xC7, 0xEA, 0x00, 0xB6, 0x4C, 0xC4});
    serv_data->set_char_uuid128((uint8_t[]){0x1B, 0xC5, 0xD5, 0xA5, 0x02, 0x00, 0x8A, 0x91, 0xE3, 0x11, 0xCB, 0xEA, 0x20, 0x29, 0x48, 0x55});
    ble_client_->add_service(serv_data.release());

    auto serv_alert = std::make_unique<myhomeiot_ble_client2::MyHomeIOT_BLEClientService>();
    serv_alert->set_service_uuid16(0x1802);
    serv_alert->set_char_uuid16(0x2A06);
    serv_alert->set_skip_empty();
    serv_alert->set_value_template([this]() -> std::vector<uint8_t> {
      size_t index = alert_selected_;
      size_t prev_value = alert_value_;
      if (index >= 4) {
        if (index == 7 && prev_value == index)
          return {};
        alert_value_ = index;
        return {(uint8_t)(index == 7 ? 0x00 : index == 4 ? 0x01 : index == 5 ? 0x02 : 0x03)};
      } else {
        alert_value_ = 0;
        if (index > 0) {
          set_alert(0);
          return {(uint8_t)(index == 1 ? 0x01 : index == 2 ? 0x02 : 0x03)};
        } else if (prev_value == 7) {
          return {(uint8_t)0x02};
        } else {
          return {};
        }
      }
    });
    ble_client_->add_service(serv_alert.release());

    // Automations
    auto connect_trigger = new myhomeiot_ble_client2::MyHomeIOT_BLEClientConnectTrigger(ble_client_);
    auto connect_action = new LambdaAction<int, const myhomeiot_ble_client2::MyHomeIOT_BLEClient2 &>(
        [this](int rssi, const myhomeiot_ble_client2::MyHomeIOT_BLEClient2 &) {
          last_online_ = millis();
          online_ = true;
          rssi_ = rssi;
          next_time_ = 0;
        });
    (new Automation<int, const myhomeiot_ble_client2::MyHomeIOT_BLEClient2 &>(connect_trigger))->add_actions({connect_action});

    auto value_trigger = new myhomeiot_ble_client2::MyHomeIOT_BLEClientValueTrigger(ble_client_);
    auto value_action = new LambdaAction<std::vector<uint8_t>, int, bool &, const myhomeiot_ble_client2::MyHomeIOT_BLEClient2 &>(
        [this](std::vector<uint8_t> x, int service, bool &stop_processing, const myhomeiot_ble_client2::MyHomeIOT_BLEClient2 &) {
          if (service == 1) {
            batt_ = x[0];
          } else if (service == 2) {
            temp_ = interpolate((float)(*(uint16_t *)&x[0]), temp_input, temp_output);
            humi_ = (float)(*(uint16_t *)&x[2]) / 13.0;
            float raw_soil = (float)(*(uint16_t *)&x[4]);
            soil_ = SOIL.unit == nullptr ? raw_soil : interpolate(raw_soil, soil_input, soil_output, true);
            lumi_ = interpolate((float)(*(uint16_t *)&x[6]), lumi_input, lumi_output);
            new_state_ = true;
            next_time_ = 0;
          }
        });
    (new Automation<std::vector<uint8_t>, int, bool &, const myhomeiot_ble_client2::MyHomeIOT_BLEClient2 &>(value_trigger))->add_actions({value_action});

    auto error_trigger = new myhomeiot_ble_client2::MyHomeIOT_BLEClientErrorTrigger(ble_client_);
    auto error_action = new LambdaAction<uint32_t, const myhomeiot_ble_client2::MyHomeIOT_BLEClient2 &>(
        [this](uint32_t error_count, const myhomeiot_ble_client2::MyHomeIOT_BLEClient2 &) {
          error_count_ = error_count;
        });
    (new Automation<uint32_t, const myhomeiot_ble_client2::MyHomeIOT_BLEClient2 &>(error_trigger))->add_actions({error_action});

    set_component_source(id_.c_str());
    ble_host->register_ble_client(ble_client_);
    App.register_component(ble_client_);
  }

  void dump_config() override {
    ESP_LOGCONFIG(TAG, "Mclh09Device '%s'", id_.c_str());
    ESP_LOGCONFIG(TAG, "  MAC address: %s", to_string(address_).c_str());
    ESP_LOGCONFIG(TAG, "  State Topic: '%s'", state_topic_.c_str());
    ESP_LOGCONFIG(TAG, "  Status Topic: '%s'", status_topic_.c_str());
    ESP_LOGCONFIG(TAG, "  Payload online/offline: '%s', '%s'", payload_online_, payload_offline_);
    ESP_LOGCONFIG(TAG, "  Alert State Topic: '%s'", alert_state_topic_.c_str());
    ESP_LOGCONFIG(TAG, "  Alert Command Topic: '%s'", alert_set_topic_.c_str());
  }

  void setup() override {
    ESP_LOGV(id_.c_str(), "Setup");
    pref_ = global_preferences->make_preference<size_t>(fnv1_hash(id_), true);
    if (!pref_.load(&alert_selected_) || alert_selected_ >= alert_options.size()) {
      ESP_LOGD(id_.c_str(), "Unknown/new alert state pref: %d", alert_selected_);
      alert_selected_ = 0;
    }
    new_alert_ = true;

    mqtt_client_->subscribe(alert_set_topic_, [this](const std::string &topic, const std::string &payload) {
      ESP_LOGD(id_.c_str(), "Alert: '%s'", payload.c_str());
      for (size_t i = 0; i < alert_options.size(); ++i) {
        if (alert_options[i] == payload) {
          set_alert(i);
          ble_client_->force_update();
          return;
        }
      }
      ESP_LOGE(id_.c_str(), "Unknown alert '%s' received!", payload.c_str());
      set_alert(alert_selected_);
    });
  }

  void loop() override {
    uint32_t now = millis();
    if (next_time_ > 0 && now < next_time_)
      return;

    bool discovery_enabled = mqtt_client_->is_discovery_enabled();
    if (connected_ && discovery_enabled && discovered_sensors_ < mclh09_sensors.size()) {
      if (discover_sensor(mclh09_sensors[discovered_sensors_]))
        ++discovered_sensors_;
      return;
    }
    if (connected_ && discovery_enabled && discovered_selects_ < mclh09_selects.size()) {
      if (discover_select(mclh09_selects[discovered_selects_]))
        ++discovered_selects_;
      return;
    }

    if (online_ && last_online_ + 2 * update_interval_ < now) {
      ESP_LOGD(id_.c_str(), "Going to offline");
      online_ = false;
    }

    if (connected_ && online_ != prev_online_) {
      const char *payload = online_ ? payload_online_ : payload_offline_;
      ESP_LOGD(id_.c_str(), "Sending status '%s'", payload);
      if (mqtt_client_->publish(status_topic_, payload, strlen(payload), 0, true)) {
        prev_online_ = online_;
      } else {
        ESP_LOGE(id_.c_str(), "Failed to send status '%s' into topic '%s'", payload, status_topic_.c_str());
      }
      return;
    }

    if (connected_ && online_ && new_state_) {
      std::string payload = create_state_payload();
      ESP_LOGD(id_.c_str(), "Sending sensor data: '%s'", payload.c_str());
      if (mqtt_client_->publish(state_topic_, payload.c_str(), payload.length())) {
        new_state_ = false;
      } else {
        ESP_LOGE(id_.c_str(), "Failed to send sensor data into topic '%s'", state_topic_.c_str());
      }
      return;
    }

    if (connected_ && online_ && new_alert_) {
      const std::string &selected = alert_options[alert_selected_];
      ESP_LOGD(id_.c_str(), "Sending alert state '%s'", selected.c_str());
      if (mqtt_client_->publish(alert_state_topic_, selected.c_str(), selected.length(), 0, true)) {
        new_alert_ = false;
      } else {
        ESP_LOGE(id_.c_str(), "Failed to send alert state into topic '%s'", alert_state_topic_.c_str());
      }
      return;
    }

    next_time_ = now + 10000;
    ESP_LOGV(id_.c_str(), "Loop idle");
  }

  myhomeiot_ble_client2::MyHomeIOT_BLEClient2 *get_ble_client() { return ble_client_; }
  uint64_t get_address() { return address_; }
    void set_alert(size_t index) {
    if (index >= alert_options.size())
      index = 0;
    if (alert_selected_ != index) {
      alert_selected_ = index;
      pref_.save(alert_selected_);
      new_alert_ = true;
    }
  }

  static float interpolate(float x, const std::vector<float> &input, const std::vector<float> &output, bool reverse = false) {
    if (input.empty() || output.empty() || input.size() != output.size())
      return x;
    if (reverse) {
      if (x >= input.front())
        return output.front();
      if (x <= input.back())
        return output.back();
    } else {
      if (x <= input.front())
        return output.front();
      if (x >= input.back())
        return output.back();
    }
    for (size_t i = 0; i < input.size() - 1; ++i) {
      if ((!reverse && x >= input[i] && x <= input[i + 1]) || (reverse && x <= input[i] && x >= input[i + 1])) {
        float x0 = input[i], x1 = input[i + 1];
        float y0 = output[i], y1 = output[i + 1];
        return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
      }
    }
    return x;
  }

 protected:
  bool discover_sensor(Sensor *sensor) {
    if (sensor == nullptr)
      return true;
    std::string topic = topic_prefix_ + "/" + sensor->id + "/" + state_suffix;
    std::string config_topic = topic_prefix_ + "/" + sensor->id + "/config";

    std::ostringstream config_payload;
    config_payload << "{";
    config_payload << "\"name\":\"" << name_ << " " << sensor->name << "\",";
    config_payload << "\"state_topic\":\"" << topic << "\",";
    if (sensor->unit)
      config_payload << "\"unit_of_measurement\":\"" << sensor->unit << "\",";
    if (sensor->dev_class)
      config_payload << "\"device_class\":\"" << sensor->dev_class << "\",";
    if (sensor->icon)
      config_payload << "\"icon\":\"" << sensor->icon << "\",";
    if (sensor->state_class)
      config_payload << "\"state_class\":\"" << sensor->state_class << "\",";
    config_payload << "\"unique_id\":\"" << id_ << "_" << sensor->id << "\",";
    config_payload << "\"availability_topic\":\"" << status_topic_ << "\"";
    config_payload << "}";

    if (!mqtt_client_->publish(config_topic, config_payload.str(), config_payload.str().length(), 1, true)) {
      ESP_LOGE(id_.c_str(), "Failed to publish config for sensor '%s'", sensor->id);
      return false;
    }
    return true;
  }

  bool discover_select(Select *select) {
    if (select == nullptr)
      return true;
    std::string set_topic = topic_prefix_ + "/" + select->id + "/" + select->set_suffix;
    std::string state_topic = topic_prefix_ + "/" + select->id + "/" + select->state_suffix;
    std::string config_topic = topic_prefix_ + "/" + select->id + "/config";

    std::ostringstream config_payload;
    config_payload << "{";
    config_payload << "\"name\":\"" << name_ << " " << select->name << "\",";
    config_payload << "\"command_topic\":\"" << set_topic << "\",";
    config_payload << "\"state_topic\":\"" << state_topic << "\",";
    config_payload << "\"options\":[";
    for (size_t i = 0; i < select->options.size(); ++i) {
      config_payload << "\"" << select->options[i] << "\"";
      if (i + 1 < select->options.size())
        config_payload << ",";
    }
    config_payload << "],";
    config_payload << "\"unique_id\":\"" << id_ << "_" << select->id << "\",";
    config_payload << "\"availability_topic\":\"" << status_topic_ << "\"";
    config_payload << "}";

    if (!mqtt_client_->publish(config_topic, config_payload.str(), config_payload.str().length(), 1, true)) {
      ESP_LOGE(id_.c_str(), "Failed to publish config for select '%s'", select->id);
      return false;
    }
    return true;
  }

  std::string create_state_payload() {
    std::ostringstream payload;
    payload << "{";
    payload << "\"batt\":" << batt_ << ",";
    payload << "\"temp\":" << temp_ << ",";
    payload << "\"humi\":" << humi_ << ",";
    payload << "\"soil\":" << soil_ << ",";
    payload << "\"lumi\":" << lumi_ << ",";
    payload << "\"rssi\":" << rssi_ << ",";
    payload << "\"errors\":" << error_count_;
    payload << "}";
    return payload.str();
  }

  std::string to_string(uint64_t mac) {
    char buf[18];
    snprintf(buf, sizeof(buf), "%012llx", mac);
    return std::string(buf);
  }

  mqtt::MQTTClientComponent *mqtt_client_;
  myhomeiot_ble_host::MyHomeIOT_BLEHost *ble_host_;
  myhomeiot_ble_client2::MyHomeIOT_BLEClient2 *ble_client_;
  uint64_t address_;
  uint32_t update_interval_;
  std::string id_;
  std::string name_;
  std::string topic_prefix_;
  std::string status_topic_;
  std::string state_topic_;
  std::string alert_state_topic_;
  std::string alert_set_topic_;
  bool device_has_availability_{false};
  const char *payload_online_;
  const char *payload_offline_;

  bool connected_{true};
  bool online_{false};
  bool prev_online_{false};
  uint32_t last_online_{0};
  uint32_t next_time_{0};
  bool new_state_{false};
  bool new_alert_{false};
  size_t alert_selected_{0};
  size_t discovered_sensors_{0};
  size_t discovered_selects_{0};
  size_t alert_value_{0};

  float batt_{0};
  float temp_{0};
  float humi_{0};
  float soil_{0};
  float lumi_{0};
  int rssi_{0};
  uint32_t error_count_{0};

  preferences::Preference<size_t> pref_;
};

}  // namespace mclh_09_mqtt_gateway
}  // namespace esphome

#endif  // USE_ESP32
