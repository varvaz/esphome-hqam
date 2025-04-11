#include "esphome.h"
#include <sstream>
#include <iostream>
#include <iomanip>
#include <list>

class Automower : public PollingComponent, public UARTDevice {
private:
    const uint8_t MAN_DATA[5] = {0x0F, 0x81, 0x2C, 0x00, 0x00};
    const uint8_t AUTO_DATA[5] = {0x0F, 0x81, 0x2C, 0x00, 0x01};
    const uint8_t HOME_DATA[5] = {0x0F, 0x81, 0x2C, 0x00, 0x03};
    const uint8_t DEMO_DATA[5] = {0x0F, 0x81, 0x2C, 0x00, 0x04};

    const uint8_t STOP_ON_DATA[5] = {0x0F, 0x81, 0x2F, 0x00, 0x02};
    const uint8_t STOP_OFF_DATA[5] = {0x0F, 0x81, 0x2F, 0x00, 0x00};
    const uint8_t READ_STOP_CMD[5] = {0x0F, 0x01, 0x2F, 0x00, 0x00};

    const uint8_t getModeCmd[5] = {0x0F, 0x01, 0x2C, 0x00, 0x00};
    const uint8_t getStatusCode[5] = {0x0F, 0x01, 0xF1, 0x00, 0x00};
    const uint8_t getChargingTime[5] = {0x0F, 0x01, 0xEC, 0x00, 0x00};

    const uint8_t getBatteryCurrent[5] = {0x0F, 0x01, 0xEB, 0x00, 0x00}; //mA
    const uint8_t getBatteryLevel[5] = {0x0F, 0x01, 0xEF, 0x00, 0x00}; //BatteryCapacityMah
    const uint8_t getBatteryCapacityAtSearchStart[5] = {0x0F, 0x01, 0xF0, 0x00, 0x00};
    const uint8_t getBatteryUsed[5] = {0x0F, 0x2E, 0xE0, 0x00, 0x00};
    const uint8_t getBatteryVoltage[5] = {0x0F, 0x2E, 0xF4, 0x00, 0x00}; //Battery voltage [mV]
    const uint8_t getFirmwareVersion[5] = {0x0F, 0x33, 0x90, 0x00, 0x00};

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

public:
    bool stopStatus = false;
    Sensor *batteryLevelSensor = new Sensor();
    Sensor *batteryUsedSensor = new Sensor();
    Sensor *chargingTimeSensor = new Sensor();
    Sensor *batteryVoltageSensor = new Sensor();
    Sensor *firmwareVersionSensor = new Sensor();
    TextSensor *modeTextSensor = new TextSensor();
    TextSensor *statusTextSensor = new TextSensor();
    TextSensor *lastCodeReceivedTextSensor = new TextSensor();

    Automower(UARTComponent *parent, uint32_t update_interval) :
        PollingComponent(update_interval), UARTDevice(parent) {
        batteryLevelSensor->set_accuracy_decimals(0);
        chargingTimeSensor->set_accuracy_decimals(0);
        batteryUsedSensor->set_accuracy_decimals(0);
        batteryVoltageSensor->set_accuracy_decimals(3);
        firmwareVersionSensor->set_accuracy_decimals(0);
    }

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
        uint8_t data[5] = {0x0F, 0x92, 0x03, (uint8_t)((value >> 8) & 0xFF), (uint8_t)(value & 0xFF)};
        write_array(data, 5);
    }

    void setLeftMotor(int value) {
        uint8_t data[5] = {0x0F, 0x92, 0x23, (uint8_t)((value >> 8) & 0xFF), (uint8_t)(value & 0xFF)};
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

    void numKey(uint8_t num) {
        uint8_t data[5] = {0x0F, 0x80, 0x5F, 0x00, num};
        write_array(data, 5);
    }

    void setup() override {}

    void sendCommands(int index) {
        if (index <= pollingCommandList.size() - 1) {
            set_retry(5, 3, [this, index](uint8_t attempt) -> RetryResult {
                if (!_writable) {
                    return RetryResult::RETRY;
                } else {
                    auto it = pollingCommandList.begin();
                    std::advance(it, index);
                    this->write_array(*it, 5);
                    this->_writable = false;
                    this->sendCommands(index + 1);
                    return RetryResult::DONE;
                }
            }, 2.0f);
        }
    }

    void update() {
        sendCommands(0);
    }

    void checkUartRead() {
        while (available() > 0 && peek() != 0x0F) {
            read();
        }
        while (available() >= 5 && peek() == 0x0F) {
            uint8_t readData[5];
            read_array(readData, 5);
            _writable = true;

            uint16_t receivedAddress = ((readData[1] & 0x7f) << 8) | readData[2];
            uint16_t receivedValue = (readData[4] << 8) | readData[3];

            switch (receivedAddress) {
                case 0x012C:
                    publishMode(receivedValue);
                    break;
                case 0x01F1:
                    publishStatus(receivedValue);
                    break;
                case 0x01EF:
                    batteryLevelSensor->publish_state((float)receivedValue);
                    break;
                case 0x01EC:
                    chargingTimeSensor->publish_state((float)receivedValue);
                    break;
                case 0x2EE0:
                    batteryUsedSensor->publish_state((float)receivedValue);
                    break;
                case 0x2EF4:
                    batteryVoltageSensor->publish_state(((float)receivedValue) / 1000.0f);
                    break;
                case 0x3390:
                    firmwareVersionSensor->publish_state((float)receivedValue);
                    break;
                case 0x012F:
                    setStopStatusFromCode(receivedValue);
                    break;
            }
        }
    }

    void setStopStatusFromCode(uint16_t receivedValue) {
        switch (receivedValue) {
            case 0x0000:
                stopStatus = false;
                break;
            case 0x0002:
                stopStatus = true;
                break;
            default:
                char s[16];
                sprintf(s, "%04x", receivedValue);
                ESP_LOGE("Automower", "Stop status non géré : %s", s);
                break;
        }
    }

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
            default:
                char s[16];
                sprintf(s, "MODE_%04x", receivedValue);
                mode.assign(s);
                break;
        }
        modeTextSensor->publish_state(mode);
    }

    void publishStatus(uint16_t receivedValue) {
        switch (receivedValue) {
            case 0x0006:
                statusTextSensor->publish_state("LEB Left engine blocked");
                break;
            case 0x000C:
                statusTextSensor->publish_state("NCS No cable signal");
                break;
            case 0x0010:
                statusTextSensor->publish_state("OUT");
                break;
            case 0x0012:
                statusTextSensor->publish_state("LBV Low battery voltage");
                break;
            case 0x001A:
                statusTextSensor->publish_state("CSB Charging station blocked");
                break;
            case 0x0022:
                statusTextSensor->publish_state("MRA Mower raised");
                break;
            case 0x0034:
                statusTextSensor->publish_state("NCC No contact with the charging station");
                break;
            case 0x0036:
                statusTextSensor->publish_state("PIN expired");
                break;
            case 0x03E8:
                statusTextSensor->publish_state("SE1 Station exit");
                break;
            case 0x03EA:
                statusTextSensor->publish_state("MIP Mowing in progress");
                break;
            case 0x03EE:
                statusTextSensor->publish_state("STM Starting the mower");
                break;
            case 0x03F0:
                statusTextSensor->publish_state("MST Mower started");
                break;
            case 0x03F4:
                statusTextSensor->publish_state("MSS Mower start signal");
                break;
            case 0x03F6:
                statusTextSensor->publish_state("ICH In charge");
                break;
            case 0x03F8:
                statusTextSensor->publish_state("WIS Waiting in station");
                break;
            case 0x0400:
                statusTextSensor->publish_state("SE2 Station exit 2");
                break;
            case 0x040C:
                statusTextSensor->publish_state("SMO Square Mode");
                break;
            case 0x040E:
                statusTextSensor->publish_state("STU Stuck");
                break;
            case 0x0410:
                statusTextSensor->publish_state("COL Collision");
                break;
            case 0x0412:
                statusTextSensor->publish_state("RES Research");
                break;
            case 0x0414:
                statusTextSensor->publish_state("STP Stop");
                break;
            case 0x0418:
                statusTextSensor->publish_state("DOC Docking");
                break;
            case 0x041A:
                statusTextSensor->publish_state("SE3 Station exit 3");
                break;
            case 0x041C:
                statusTextSensor->publish_state("ERR Error");
                break;
            case 0x0420:
                statusTextSensor->publish_state("WHO Waiting HOME");
                break;
            case 0x0422:
                statusTextSensor->publish_state("FTL Follow the limit");
                break;
            case 0x0424:
                statusTextSensor->publish_state("SFD Signal found");
                break;
            case 0x0426:
                statusTextSensor->publish_state
