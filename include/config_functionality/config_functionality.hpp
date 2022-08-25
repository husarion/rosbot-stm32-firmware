#pragma once
#include <RosbotDrive.h>
#include <rosbot_ekf/Configuration.h>
#include <rosbot_kinematics.h>
#include <rosbot_sensors.h>

#include <main.hpp>
#include <map>
#include <string>



class ConfigFunctionality {
   public:
    typedef uint8_t (ConfigFunctionality::*configuration_srv_fun_t)(const char *datain, const char **dataout);
    static ConfigFunctionality *getInstance();
    configuration_srv_fun_t findFunctionality(const char *command);
    uint8_t setLed(const char *datain, const char **dataout);
    uint8_t enableImu(const char *datain, const char **dataout);
    uint8_t enableDistanceSensors(const char *datain, const char **dataout);
    uint8_t enableJointStates(const char *datain, const char **dataout);
    uint8_t resetOdom(const char *datain, const char **dataout);
    uint8_t enableSpeedWatchdog(const char *datain, const char **dataout);
    uint8_t getAngle(const char *datain, const char **dataout);
    uint8_t resetImu(const char *datain, const char **dataout);
    uint8_t setMotorsAccelDeaccel(const char *datain, const char **dataout);
    uint8_t setAnimation(const char *datain, const char **dataout);
    uint8_t enableTfMessages(const char *datain, const char **dataout);
    uint8_t calibrateOdometry(const char *datain, const char **dataout);
    uint8_t enableMotors(const char *datain, const char **dataout);
    uint8_t configureServo(const char *datain, const char **dataout);
    uint8_t getPid(const char *datain, const char **dataout);
    uint8_t configurePid(const char *datain, const char **dataout);
    uint8_t setKinematics(const char *datain, const char **dataout);
    uint8_t getKinematics(const char *datain, const char **dataout);

   private:
    ConfigFunctionality();
    char _buffer[128];
    static ConfigFunctionality *_instance;
    static const char SLED_COMMAND[];
    static const char EIMU_COMMAND[];
    static const char EDSE_COMMAND[];
    static const char EJSM_COMMAND[];
    static const char RODOM_COMMAND[];
    static const char EWCH_COMMAND[];
    static const char RIMU_COMMAND[];
    static const char SMAD_COMMAND[];
    static const char SANI_COMMAND[];
    static const char ETFM_COMMAND[];
    static const char CALI_COMMAND[];
    static const char EMOT_COMMAND[];
    static const char CSER_COMMAND[];
    static const char GSER_COMMAND[];
    static const char GPID_COMMAND[];
    static const char CPID_COMMAND[];
    static const char SKIN_COMMAND[];
    static const char GKIN_COMMAND[];
    std::map<std::string, configuration_srv_fun_t> _commands;
};

ConfigFunctionality *ConfigFunctionality::_instance = NULL;

const char ConfigFunctionality::SLED_COMMAND[] = "SLED";
const char ConfigFunctionality::EIMU_COMMAND[] = "EIMU";
const char ConfigFunctionality::EDSE_COMMAND[] = "EDSE";
const char ConfigFunctionality::EJSM_COMMAND[] = "EJSM";
const char ConfigFunctionality::RODOM_COMMAND[] = "RODOM";
const char ConfigFunctionality::EWCH_COMMAND[] = "EWCH";
const char ConfigFunctionality::RIMU_COMMAND[] = "RIMU";
const char ConfigFunctionality::SMAD_COMMAND[] = "SMAD";
const char ConfigFunctionality::SANI_COMMAND[] = "SANI";
const char ConfigFunctionality::ETFM_COMMAND[] = "ETFM";
const char ConfigFunctionality::CALI_COMMAND[] = "CALI";
const char ConfigFunctionality::EMOT_COMMAND[] = "EMOT";
const char ConfigFunctionality::CSER_COMMAND[] = "CSER";
const char ConfigFunctionality::GSER_COMMAND[] = "GSER";
const char ConfigFunctionality::GPID_COMMAND[] = "GPID";
const char ConfigFunctionality::CPID_COMMAND[] = "CPID";
const char ConfigFunctionality::SKIN_COMMAND[] = "SKIN";
const char ConfigFunctionality::GKIN_COMMAND[] = "GKIN";

/**
 * @brief Parse servo commands.
 * @param command string to be parsed
 * @return true if command was successfully parsed
 */
static bool servoCommandParser(const char *command) {
    char buffer[64], key;
    int value;
    if (command == nullptr || strlen(command) == 0)
        return false;
    strncpy(buffer, command, 64);
    char *token = strtok(buffer, " ");

    // servo configuration data
    int servo_num = -1;
    int servo_width = -1;
    int servo_period = -1;
    int servo_enabled = -1;
    int servo_power = -1;
    int servo_voltage = -1;

    // parsing commands
    while (token != NULL) {
        if (sscanf(token, "%c:%d", &key, &value) == 2) {
            switch (key) {
                case 'S':
                case 's':
                    servo_num = value - 1;
                    break;
                case 'P':
                case 'p':
                    servo_period = value;
                    break;
                case 'E':
                case 'e':
                    servo_enabled = value;
                    break;
                case 'V':
                case 'v':
                    servo_voltage = value;
                    break;
                case 'W':
                case 'w':
                    servo_width = value;
                    break;
            }
        } else
            return false;
        token = strtok(NULL, " ");
    }

    if (servo_voltage != -1) {
        servo_manager.setPowerMode(servo_voltage);
    }

    if (servo_enabled != -1) {
        if (servo_num == -1)
            return false;

        servo_manager.enableOutput(servo_num, servo_enabled);
    }

    if (servo_period != -1) {
        if (servo_num == -1)
            return false;

        if (!servo_manager.setPeriod(servo_num, servo_period))
            return false;
    }

    if (servo_width != -1) {
        if (servo_num == -1)
            return false;

        if (!servo_manager.setWidth(servo_num, servo_width))
            return false;
    }

    if (servo_manager.getEnabledOutputs() > 0)
        servo_manager.enablePower(true);
    else
        servo_manager.enablePower(false);
    return true;
}

static bool pidCommandParser(const char *command) {
    char buffer[64], key[10];
    float value = 8.0f;
    if (command == nullptr || strlen(command) == 0)
        return false;
    strncpy(buffer, command, 64);
    char *token = strtok(buffer, " ");

    // servo configuration data
    float kp = -1.0f;
    float ki = -1.0f;
    float kd = -1.0f;
    float out_max = -1.0f;
    float out_min = -2.0f;
    float speed_max = -1.0f;
    float a_max = -1.0f;

    // parsing commands
    while (token != NULL) {
        if (sscanf(token, "%[^:]s", key) != 1)
            return false;

        if (strcmp("kp", key) == 0) {
            if (sscanf(token, "kp:%f", &value) == 1)
                kp = value;
            else
                return false;
        } else if (strcmp("ki", key) == 0) {
            if (sscanf(token, "ki:%f", &value) == 1)
                ki = value;
            else
                return false;
        } else if (strcmp("kd", key) == 0) {
            if (sscanf(token, "kd:%f", &value) == 1)
                kd = value;
            else
                return false;
        } else if (strcmp("out_max", key) == 0) {
            if (sscanf(token, "out_max:%f", &value) == 1)
                out_max = min<float>(value, 0.80f);
            else
                return false;
        } else if (strcmp("out_min", key) == 0) {
            if (sscanf(token, "out_min:%f", &value) == 1)
                out_min = max<float>(value, -0.80f);
            else
                return false;
        } else if (strcmp("a_max", key) == 0) {
            if (sscanf(token, "a_max:%e", &value) == 1)
                a_max = value;
            else
                return false;
        } else if (strcmp("speed_max", key) == 0) {
            if (sscanf(token, "speed_max:%f", &value) == 1)
                speed_max = min<float>(value, 1.25f);
            else
                return false;
        } else {
            return false;
        }

        token = strtok(NULL, " ");
    }

    RosbotRegulator_params params;
    RosbotDrive::getInstance().getPidParams(params);

    if (ki != -1.0f) {
        params.ki = ki;
    }

    if (kp != -1.0f) {
        params.kp = kp;
    }

    if (kd != -1.0f) {
        params.kd = kd;
    }

    if (out_max != -1.0f) {
        params.out_max = out_max;
    }

    if (out_min != -2.0f) {
        params.out_min = out_min;
    }

    if (a_max != -1.0f) {
        params.a_max = a_max;
    }

    if (speed_max != -1.0f) {
        params.speed_max = speed_max;
    }

    RosbotDrive::getInstance().updatePidParams(params);
    return true;
}
