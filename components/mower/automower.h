#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
// #include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace mower {

class Automower : public PollingComponent, public uart::UARTDevice {
 public:
  // Constructor. Uses a parent UART component and an update interval.
  Automower(uart::UARTComponent *parent, uint32_t update_interval)
    : PollingComponent(update_interval), uart::UARTDevice(parent) {
      // Set sensor accuracy if the sensors exist.
      battery_level_sensor_->set_accuracy_decimals(0);
      charging_time_sensor_->set_accuracy_decimals(0);
      battery_used_sensor_->set_accuracy_decimals(0);
      battery_voltage_sensor_->set_accuracy_decimals(3);
      firmware_version_sensor_->set_accuracy_decimals(0);
    }

  // Command arrays.
  static constexpr uint8_t MAN_DATA[5]         = {0x0F, 0x81, 0x2C, 0x00, 0x00};
  static constexpr uint8_t AUTO_DATA[5]        = {0x0F, 0x81, 0x2C, 0x00, 0x01};
  static constexpr uint8_t HOME_DATA[5]        = {0x0F, 0x81, 0x2C, 0x00, 0x03};
  static constexpr uint8_t DEMO_DATA[5]        = {0x0F, 0x81, 0x2C, 0x00, 0x04};

  static constexpr uint8_t STOP_ON_DATA[5]     = {0x0F, 0x81, 0x2F, 0x00, 0x02};
  static constexpr uint8_t STOP_OFF_DATA[5]    = {0x0F, 0x81, 0x2F, 0x00, 0x00};
  static constexpr uint8_t READ_STOP_CMD[5]    = {0x0F, 0x01, 0x2F, 0x00, 0x00};

  static constexpr uint8_t getModeCmd[5]       = {0x0F, 0x01, 0x2C, 0x00, 0x00};
  static constexpr uint8_t getStatusCode[5]    = {0x0F, 0x01, 0xF1, 0x00, 0x00};
  static constexpr uint8_t getChargingTime[5]  = {0x0F, 0x01, 0xEC, 0x00, 0x00};
  static constexpr uint8_t getBatteryCurrent[5]= {0x0F, 0x01, 0xEB, 0x00, 0x00}; // mA
  static constexpr uint8_t getBatteryLevel[5]  = {0x0F, 0x01, 0xEF, 0x00, 0x00}; // Battery capacity in mAh 
  static constexpr uint8_t getBatteryCapacityAtSearchStart[5] = {0x0F, 0x01, 0xF0, 0x00, 0x00};
  static constexpr uint8_t getBatteryUsed[5]   = {0x0F, 0x2E, 0xE0, 0x00, 0x00};
  static constexpr uint8_t getBatteryVoltage[5]= {0x0F, 0x2E, 0xF4, 0x00, 0x00}; // in mV
  static constexpr uint8_t getFirmwareVersion[5]= {0x0F, 0x33, 0x90, 0x00, 0x00};

  // List of command arrays to poll.
  const std::list<const uint8_t *> pollingCommandList = {
    getModeCmd,
    getStatusCode,
    getBatteryLevel,
    getChargingTime,
    getBatteryUsed,
    getBatteryVoltage,
    getFirmwareVersion,
    READ_STOP_CMD,
  };

  bool _writable = true;
  bool stopStatus = false;

  // Sensor objects. (Instantiated here; you may choose to configure them differently.)
  sensor::Sensor *battery_level_sensor_    = new sensor::Sensor();
  sensor::Sensor *battery_used_sensor_     = new sensor::Sensor();
  sensor::Sensor *charging_time_sensor_    = new sensor::Sensor();
  sensor::Sensor *battery_voltage_sensor_  = new sensor::Sensor();
  sensor::Sensor *firmware_version_sensor_ = new sensor::Sensor();
  text_sensor::TextSensor *mode_text_sensor_= new text_sensor::TextSensor();
  text_sensor::TextSensor *status_text_sensor_ = new text_sensor::TextSensor();
  text_sensor::TextSensor *last_code_received_text_sensor_
                                           = new text_sensor::TextSensor();

  // Action methods.
  void setMode(const std::string &value) {
    if (value == "MAN") {
      write_array(MAN_DATA, sizeof(MAN_DATA));
    } else if (value == "AUTO") {
      write_array(AUTO_DATA, sizeof(AUTO_DATA));
    } else if (value == "HOME") {
      write_array(HOME_DATA, sizeof(HOME_DATA));
    } else if (value == "DEMO") {
      write_array(DEMO_DATA, sizeof(DEMO_DATA));
    } else {
      ESP_LOGE("Automower", "Mode non géré : %s", value.c_str());
    }
  }

  void setStop(bool stop) {
    if (stop) {
      write_array(STOP_ON_DATA, sizeof(STOP_ON_DATA));
    } else {
      write_array(STOP_OFF_DATA, sizeof(STOP_OFF_DATA));
    }
  }

  void setRightMotor(int value) {
    uint8_t data[5] = {0x0F, 0x92, 0x03,
                       (uint8_t)((value >> 8) & 0xFF),
                       (uint8_t)(value & 0xFF)};
    write_array(data, 5);
  }

  void setLeftMotor(int value) {
    uint8_t data[5] = {0x0F, 0x92, 0x23,
                       (uint8_t)((value >> 8) & 0xFF),
                       (uint8_t)(value & 0xFF)};
    write_array(data, 5);
  }

  void setMode0D() {
    uint8_t data[5] = {0x0F, 0x81, 0x0D, 0x3A, 0x9D};
    write_array(data, 5);
  }

  void setMode9A() {
    uint8_t data[5] = {0x0F, 0x81, 0x9A, 0x00, 0x90};
    write_array(data, 5);
  }

  void setMode99() {
    uint8_t data[5] = {0x0F, 0x81, 0x99, 0x00, 0x90};
    write_array(data, 5);
  }

  void backKey() {
    uint8_t data[5] = {0x0F, 0x80, 0x5F, 0x00, 0x0F};
    write_array(data, 5);
  }

  void yesKey() {
    uint8_t data[5] = {0x0F, 0x80, 0x5F, 0x00, 0x12};
    write_array(data, 5);
  }

  // Numeric key press.
  void numKey(uint8_t num) {
    uint8_t data[5] = {0x0F, 0x80, 0x5F, 0x00, num};
    write_array(data, 5);
  }

  // ESPHome lifecycle methods.
  void setup() override {
    // Additional setup code can be added here if needed.
  }

  void update() override {
    sendCommands(0);
  }

  void loop() override {
    checkUartRead();
  }

 protected:
  // Recursive send command method.
  void sendCommands(int index) {
    if (index <= (int)pollingCommandList.size() - 1) {
      set_retry(
          5,    // Initial wait time in milliseconds.
          3,    // Maximum number of attempts.
          [this, index](uint8_t attempt) -> RetryResult {
            if (!_writable) {
              return RetryResult::RETRY;
            } else {
              auto it = pollingCommandList.begin();
              std::advance(it, index);
              write_array(*it, 5);
              _writable = false;
              sendCommands(index + 1);
              return RetryResult::DONE;
            }
          },
          2.0f  // Factor to increase wait time between attempts.
      );
    }
  }

  // Reads data from UART and processes it.
  void checkUartRead() {
    while (available() > 0 && peek() != 0x0F) {
      read();
    }
    while (available() >= 5 && peek() == 0x0F) {
      uint8_t readData[5];
      read_array(readData, 5);
      _writable = true;

      uint16_t receivedAddress = ((readData[1] & 0x7F) << 8) | readData[2];
      uint16_t receivedValue   = (readData[4] << 8) | readData[3];

      switch (receivedAddress) {
        case 0x012C:
          publishMode(receivedValue);
          break;
        case 0x01F1:
          publishStatus(receivedValue);
          break;
        case 0x01EF:
          battery_level_sensor_->publish_state(static_cast<float>(receivedValue));
          break;
        case 0x01EC:
          charging_time_sensor_->publish_state(static_cast<float>(receivedValue));
          break;
        case 0x2EE0:
          battery_used_sensor_->publish_state(static_cast<float>(receivedValue));
          break;
        case 0x2EF4:
          battery_voltage_sensor_->publish_state(static_cast<float>(receivedValue) / 1000.0f);
          break;
        case 0x3390:
          firmware_version_sensor_->publish_state(static_cast<float>(receivedValue));
          break;
        case 0x012F:
          setStopStatusFromCode(receivedValue);
          break;
        default:
          break;
      }
    }
  }

  // Sets the stop status based on a received code.
  void setStopStatusFromCode(uint16_t receivedValue) {
    switch (receivedValue) {
      case 0x0000:
        stopStatus = false;
        break;
      case 0x0002:
        stopStatus = true;
        break;
      default: {
        char s[16];
        sprintf(s, "%04x", receivedValue);
        ESP_LOGE("Automower", "Stop status non géré : %s", s);
        break;
      }
    }
  }

  // Publishes the operating mode.
  void publishMode(uint16_t receivedValue) {
    std::string mode;
    switch (receivedValue) {
      case 0x0000:
        mode = "MAN";
        break;
      case 0x0001:
        mode = "AUTO";
        break;
      case 0x0003:
        mode = "HOME";
        break;
      case 0x0004:
        mode = "DEMO";
        break;
      default: {
        char s[16];
        sprintf(s, "MODE_%04x", receivedValue);
        mode.assign(s);
        break;
      }
    }
    mode_text_sensor_->publish_state(mode);
  }

  // Publishes the status message.
  void publishStatus(uint16_t receivedValue) {
    switch (receivedValue) {
      case 0x0006:
        status_text_sensor_->publish_state("LEB Left engine blocked");
        break;
      case 0x000C:
        status_text_sensor_->publish_state("NCS No cable signal");
        break;
      case 0x0010:
        status_text_sensor_->publish_state("OUT");
        break;
      case 0x0012:
        status_text_sensor_->publish_state("LBV Low battery voltage");
        break;
      case 0x001A:
        status_text_sensor_->publish_state("CSB Charging station blocked");
        break;
      case 0x0022:
        status_text_sensor_->publish_state("MRA  Mower raised");
        break;
      case 0x0034:
        status_text_sensor_->publish_state("NCC No contact with the charging station");
        break;
      case 0x0036:
        status_text_sensor_->publish_state("PIN expired");
        break;
      case 0x03E8:
        status_text_sensor_->publish_state("SE1  Station exit");
        break;
      case 0x03EA:
        status_text_sensor_->publish_state("MIP Mowing in progress");
        break;
      case 0x03EE:
        status_text_sensor_->publish_state("STM Starting the mower");
        break;
      case 0x03F0:
        status_text_sensor_->publish_state("MST  Mower started");
        break;
      case 0x03F4:
        status_text_sensor_->publish_state("MSS Mower start signal");
        break;
      case 0x03F6:
        status_text_sensor_->publish_state("ICH  In charge");
        break;
      case 0x03F8:
        status_text_sensor_->publish_state("WIS Waiting in station");
        break;
      case 0x0400:
        status_text_sensor_->publish_state("SE2 Station exit 2");
        break;
      case 0x040C:
        status_text_sensor_->publish_state("SMO  Square Mode");
        break;
      case 0x040E:
        status_text_sensor_->publish_state("STU Stuck");
        break;
      case 0x0410:
        status_text_sensor_->publish_state("COL Collision");
        break;
      case 0x0412:
        status_text_sensor_->publish_state("RES Research");
        break;
      case 0x0414:
        status_text_sensor_->publish_state("STP Stop");
        break;
      case 0x0418:
        status_text_sensor_->publish_state("DOC Docking");
        break;
      case 0x041A:
        status_text_sensor_->publish_state("SE3 Station exit 3");
        break;
      case 0x041C:
        status_text_sensor_->publish_state("ERR Error");
        break;
      case 0x0420:
        status_text_sensor_->publish_state("WHO Waiting HOME");
        break;
      case 0x0422:
        status_text_sensor_->publish_state("FTL Follow the limit");
        break;
      case 0x0424:
        status_text_sensor_->publish_state("SFD Signal found");
        break;
      case 0x0426:
        status_text_sensor_->publish_state("ST2 Stuck2");
        break;
      case 0x0428:
        status_text_sensor_->publish_state("RE2 Research2");
        break;
      case 0x042E:
        status_text_sensor_->publish_state("FGC Following guide cable");
        break;
      case 0x0430:
        status_text_sensor_->publish_state("FC2 Following cable 430");
        break;
      case 0x001c:
        status_text_sensor_->publish_state("MCN Manual charge needed");
        break;
      case 0x001e:
        status_text_sensor_->publish_state("CDB Cutting disc blocked 1e");
        break;
      case 0x0020:
        status_text_sensor_->publish_state("CD2 Cutting disc blocked 20");
        break;
      default: {
        char s[16];
        sprintf(s, "STATUS_%04x", receivedValue);
        status_text_sensor_->publish_state(s);
        break;
      }
    }
  }
};

}  // namespace mower
}  // namespace esphome
