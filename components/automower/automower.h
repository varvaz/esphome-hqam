#pragma once

#include "esphome.h"

namespace esphome {
namespace automower {

class Automower : public PollingComponent, public uart::UARTDevice {
 public:
  Automower(uart::UARTComponent *parent, uint32_t update_interval);

  void setup() override;
  void update() override;
  void loop() override;

  void set_mode(const std::string &value);
  void set_stop(bool stop);
  void set_right_motor(int value);
  void set_left_motor(int value);
  void set_mode_0D();
  void set_mode_9A();
  void set_mode_99();
  void back_key();
  void yes_key();
  void num_key(uint8_t num);

  // Public sensors
  sensor::Sensor *battery_level_sensor = new sensor::Sensor();
  sensor::Sensor *battery_used_sensor = new sensor::Sensor();
  sensor::Sensor *charging_time_sensor = new sensor::Sensor();
  sensor::Sensor *battery_voltage_sensor = new sensor::Sensor();
  sensor::Sensor *firmware_version_sensor = new sensor::Sensor();
  text_sensor::TextSensor *mode_text_sensor = new text_sensor::TextSensor();
  text_sensor::TextSensor *status_text_sensor = new text_sensor::TextSensor();
  text_sensor::TextSensor *last_code_received_text_sensor = new text_sensor::TextSensor();

  bool stop_status = false;

 protected:
  void check_uart_read();
  void publish_mode(uint16_t value);
  void publish_status(uint16_t value);
  void set_stop_status_from_code(uint16_t value);
  void send_commands(int index);

  bool writable_ = true;

  const std::list<const uint8_t *> polling_commands_ = {
      get_mode_cmd_, get_status_code_, get_battery_level_, get_charging_time_,
      get_battery_used_, get_battery_voltage_, get_firmware_version_, read_stop_cmd_
  };

  // Command constants
  static constexpr uint8_t man_data_[5] = {0x0F, 0x81, 0x2C, 0x00, 0x00};
  static constexpr uint8_t auto_data_[5] = {0x0F, 0x81, 0x2C, 0x00, 0x01};
  static constexpr uint8_t home_data_[5] = {0x0F, 0x81, 0x2C, 0x00, 0x03};
  static constexpr uint8_t demo_data_[5] = {0x0F, 0x81, 0x2C, 0x00, 0x04};
  static constexpr uint8_t stop_on_data_[5] = {0x0F, 0x81, 0x2F, 0x00, 0x02};
  static constexpr uint8_t stop_off_data_[5] = {0x0F, 0x81, 0x2F, 0x00, 0x00};

  static constexpr uint8_t get_mode_cmd_[5] = {0x0F, 0x01, 0x2C, 0x00, 0x00};
  static constexpr uint8_t get_status_code_[5] = {0x0F, 0x01, 0xF1, 0x00, 0x00};
  static constexpr uint8_t get_charging_time_[5] = {0x0F, 0x01, 0xEC, 0x00, 0x00};
  static constexpr uint8_t get_battery_current_[5] = {0x0F, 0x01, 0xEB, 0x00, 0x00};
  static constexpr uint8_t get_battery_level_[5] = {0x0F, 0x01, 0xEF, 0x00, 0x00};
  static constexpr uint8_t get_battery_capacity_at_search_start_[5] = {0x0F, 0x01, 0xF0, 0x00, 0x00};
  static constexpr uint8_t get_battery_used_[5] = {0x0F, 0x2E, 0xE0, 0x00, 0x00};
  static constexpr uint8_t get_battery_voltage_[5] = {0x0F, 0x2E, 0xF4, 0x00, 0x00};
  static constexpr uint8_t get_firmware_version_[5] = {0x0F, 0x33, 0x90, 0x00, 0x00};
  static constexpr uint8_t read_stop_cmd_[5] = {0x0F, 0x01, 0x2F, 0x00, 0x00};
};

}  // namespace automower
}  // namespace esphome
