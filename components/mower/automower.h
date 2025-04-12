// automower.h (inside mower namespace)
#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace mower {

class Automower : public PollingComponent, public uart::UARTDevice {
 public:
  Automower(uart::UARTComponent *parent, uint32_t update_interval)
    : PollingComponent(update_interval), uart::UARTDevice(parent) {}

  void set_battery_level_sensor(sensor::Sensor *s) { battery_level_sensor_ = s; }
  void set_battery_used_sensor(sensor::Sensor *s) { battery_used_sensor_ = s; }
  void set_charging_time_sensor(sensor::Sensor *s) { charging_time_sensor_ = s; }
  void set_battery_voltage_sensor(sensor::Sensor *s) { battery_voltage_sensor_ = s; }
  void set_firmware_version_sensor(sensor::Sensor *s) { firmware_version_sensor_ = s; }
  void set_mode_text_sensor(text_sensor::TextSensor *s) { mode_text_sensor_ = s; }
  void set_status_text_sensor(text_sensor::TextSensor *s) { status_text_sensor_ = s; }
  void set_last_code_received_text_sensor(text_sensor::TextSensor *s) { last_code_received_text_sensor_ = s; }

  void setup() override {}
  void update() override { sendCommands(0); }
  void loop() override { checkUartRead(); }

 protected:
  bool _writable = true;
  bool stopStatus = false;

  sensor::Sensor *battery_level_sensor_ = nullptr;
  sensor::Sensor *battery_used_sensor_ = nullptr;
  sensor::Sensor *charging_time_sensor_ = nullptr;
  sensor::Sensor *battery_voltage_sensor_ = nullptr;
  sensor::Sensor *firmware_version_sensor_ = nullptr;
  text_sensor::TextSensor *mode_text_sensor_ = nullptr;
  text_sensor::TextSensor *status_text_sensor_ = nullptr;
  text_sensor::TextSensor *last_code_received_text_sensor_ = nullptr;

  static constexpr uint8_t MAN_DATA[5]  = {0x0F, 0x81, 0x2C, 0x00, 0x00};
  static constexpr uint8_t AUTO_DATA[5] = {0x0F, 0x81, 0x2C, 0x00, 0x01};
  static constexpr uint8_t HOME_DATA[5] = {0x0F, 0x81, 0x2C, 0x00, 0x03};
  static constexpr uint8_t DEMO_DATA[5] = {0x0F, 0x81, 0x2C, 0x00, 0x04};

  const std::list<const uint8_t *> pollingCommandList = {
    getModeCmd, getStatusCode, getBatteryLevel,
    getChargingTime, getBatteryUsed, getBatteryVoltage,
    getFirmwareVersion, READ_STOP_CMD
  };

  void sendCommands(int index) {
    if (index < (int)pollingCommandList.size()) {
      set_retry(5, 3, [this, index](uint8_t attempt) -> RetryResult {
        if (!_writable) return RetryResult::RETRY;
        auto it = pollingCommandList.begin();
        std::advance(it, index);
        write_array(*it, 5);
        _writable = false;
        sendCommands(index + 1);
        return RetryResult::DONE;
      }, 2.0f);
    }
  }

  void checkUartRead() {
    while (available() > 0 && peek() != 0x0F) read();
    while (available() >= 5 && peek() == 0x0F) {
      uint8_t readData[5];
      read_array(readData, 5);
      _writable = true;
      uint16_t addr = ((readData[1] & 0x7F) << 8) | readData[2];
      uint16_t val  = (readData[4] << 8) | readData[3];

      switch (addr) {
        case 0x012C: publishMode(val); break;
        case 0x01F1: publishStatus(val); break;
        case 0x01EF: if (battery_level_sensor_) battery_level_sensor_->publish_state(val); break;
        case 0x01EC: if (charging_time_sensor_) charging_time_sensor_->publish_state(val); break;
        case 0x2EE0: if (battery_used_sensor_) battery_used_sensor_->publish_state(val); break;
        case 0x2EF4: if (battery_voltage_sensor_) battery_voltage_sensor_->publish_state(val / 1000.0f); break;
        case 0x3390: if (firmware_version_sensor_) firmware_version_sensor_->publish_state(val); break;
        case 0x012F: setStopStatusFromCode(val); break;
        default: break;
      }
    }
  }

  void setStopStatusFromCode(uint16_t val) {
    stopStatus = (val == 0x0002);
  }

  void publishMode(uint16_t val) {
    if (!mode_text_sensor_) return;
    std::string mode;
    switch (val) {
      case 0x0000: mode = "MAN"; break;
      case 0x0001: mode = "AUTO"; break;
      case 0x0003: mode = "HOME"; break;
      case 0x0004: mode = "DEMO"; break;
      default: mode = "MODE_" + formatHex(val); break;
    }
    mode_text_sensor_->publish_state(mode);
  }

  void publishStatus(uint16_t val) {
    if (!status_text_sensor_) return;
    std::string s;
    switch (val) {
      case 0x0012: s = "LBV Low battery voltage"; break;
      case 0x03EA: s = "MIP Mowing in progress"; break;
      default: s = "STATUS_" + formatHex(val); break;
    }
    status_text_sensor_->publish_state(s);
  }

  std::string formatHex(uint16_t v) {
    char s[16];
    sprintf(s, "%04x", v);
    return std::string(s);
  }
};

}  // namespace mower
}  // namespace esphome
