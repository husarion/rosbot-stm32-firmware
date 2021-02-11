/** @file main.cpp
 * ROSbot firmware.
 * 
 * @copyright GNU GPL-3.0
 */
#include <rosbot_kinematics.h>
#include <rosbot_sensors.h>
#include <ImuDriver.h>
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/UInt32.h>
#include <rosbot_ekf/Imu.h>

#if defined(ROS_NOETIC_MSGS) //TODO: add information to README about compiling for noetic/melodic (flags)
    #include <sensor_msgs_noetic/BatteryState.h>
#else
    #include <sensor_msgs/BatteryState.h>
#endif

#include <sensor_msgs/Range.h>
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include <std_msgs/UInt8.h>
#include <rosbot_ekf/Configuration.h>
#include <map>
#include <string>

static const char EMPTY_STRING[] = "";

static const char WELLCOME_STR[] = "\n\n"
#if defined(WELLCOME_BANNER)
                                   " ______  _____  _____  _             _           __\n"
                                   " | ___ \\|  _  |/  ___|| |           | |         / _|\n"
                                   " | |_/ /| | | |\\ `--. | |__    ___  | |_       | |_ __      __\n"
                                   " |    / | | | | `--. \\| '_ \\  / _ \\ | __|      |  _|\\ \\ /\\ / /\n"
                                   " | |\\ \\ \\ \\_/ //\\__/ /| |_) || (_) || |_       | |   \\ V  V / \n"
                                   " \\_| \\_| \\___/ \\____/ |_.__/  \\___/  \\__|      |_|    \\_/\\_/ \n\n"
#endif
                                   " Firmware version: " ROSBOT_FW_VERSION
                                   "\n\n";

#if USE_WS2812B_ANIMATION_MANAGER
#include <AnimationManager.h>
AnimationManager *anim_manager;
static int parseColorStr(const char *color_str, Color_t *color_ptr)
{
    uint32_t num;
    if (sscanf(color_str, "%*c%X", &num) != 1)
        return 0;
    color_ptr->b = 0xff & num;
    color_ptr->g = 0xff & (num >> 8);
    color_ptr->r = 0xff & (num >> 16);
    return 1;
}
#endif

#define MAIN_LOOP_INTERVAL_MS 10
#define IMU_I2C_FREQUENCY 100000L
#define IMU_I2C_SCL SENS2_PIN3
#define IMU_I2C_SDA SENS2_PIN4

extern Mail<ImuDriver::ImuMesurement, 10> imu_sensor_mail_box;
const char *imu_sensor_type_string[] = {
    "BNO055_ADDR_A",
    "BNO055_ADDR_B",
    "MPU9250",
    "MPU9255",
    "UNKNOWN"};
char imu_description_string[64] = "";
ImuDriver *imu_driver_ptr;

geometry_msgs::Twist current_vel;
sensor_msgs::JointState joint_states;
sensor_msgs::BatteryState battery_state;
sensor_msgs::Range range_msg[4];
geometry_msgs::PoseStamped pose;
std_msgs::UInt8 button_msg;
rosbot_ekf::Imu imu_msg;
ros::NodeHandle nh;
ros::Publisher *vel_pub;
ros::Publisher *joint_state_pub;
ros::Publisher *battery_pub;
ros::Publisher *range_pub[4];
ros::Publisher *pose_pub;
ros::Publisher *button_pub;
ros::Publisher *imu_pub;
geometry_msgs::TransformStamped robot_tf;
tf::TransformBroadcaster broadcaster;

rosbot_kinematics::RosbotOdometry odometry;

volatile bool distance_sensors_enabled = false;
volatile bool joint_states_enabled = false;
volatile bool tf_msgs_enabled = false;

DigitalOut sens_power(SENS_POWER_ON, 0);

DigitalOut led2(LED2, 0);
DigitalOut led3(LED3, 0);
InterruptIn button1(BUTTON1);
InterruptIn button2(BUTTON2);
volatile bool button1_publish_flag = false;
volatile bool button2_publish_flag = false;

volatile bool is_speed_watchdog_enabled = true;
volatile bool is_speed_watchdog_active = false;
int speed_watchdog_interval = 1000; //ms

Timer odom_watchdog_timer;
volatile uint32_t last_speed_command_time = 0;

rosbot_sensors::ServoManger servo_manager;

rosbot_kinematics::RosbotKinematics *rk = nullptr;

static void button1Callback()
{
    button1_publish_flag = true;
}

static void button2Callback()
{
    button2_publish_flag = true;
}

// JointState
const char *joint_state_name[] = {"front_left_wheel_hinge", "front_right_wheel_hinge", "rear_left_wheel_hinge", "rear_right_wheel_hinge"};
double pos[] = {0, 0, 0, 0};
double vel[] = {0, 0, 0, 0};
double eff[] = {0, 0, 0, 0};

// Range
const char *range_id[] = {"range_fr", "range_fl", "range_rr", "range_rl"};
const char *range_pub_names[] = {"range/fr", "range/fl", "range/rr", "range/rl"};

static void initImuPublisher()
{
    imu_pub = new ros::Publisher("mpu9250", &imu_msg);
    nh.advertise(*imu_pub);
}

static void initButtonPublisher()
{
    button_pub = new ros::Publisher("buttons", &button_msg);
    nh.advertise(*button_pub);
}

static void initRangePublisher()
{
    for (int i = 0; i < 4; i++)
    {
        range_msg[i].field_of_view = 0.26;
        range_msg[i].min_range = 0.03;
        range_msg[i].max_range = 0.90;
        range_msg[i].header.frame_id = range_id[i];
        range_msg[i].radiation_type = sensor_msgs::Range::INFRARED;
        range_pub[i] = new ros::Publisher(range_pub_names[i], &range_msg[i]);
        nh.advertise(*range_pub[i]);
    }
}

static void initBatteryPublisher()
{
    battery_state.power_supply_status = battery_state.POWER_SUPPLY_STATUS_UNKNOWN;
    battery_state.power_supply_health = battery_state.POWER_SUPPLY_HEALTH_UNKNOWN;
    battery_state.power_supply_technology = battery_state.POWER_SUPPLY_TECHNOLOGY_LION;
    battery_pub = new ros::Publisher("battery", &battery_state);
    nh.advertise(*battery_pub);
}

static void initPosePublisher()
{
    pose.header.frame_id = "odom";
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    pose_pub = new ros::Publisher("pose", &pose);
    nh.advertise(*pose_pub);
}

static void initTfPublisher()
{
    robot_tf.header.frame_id = "odom";
    robot_tf.child_frame_id = "base_link";
    robot_tf.transform.translation.x = 0.0;
    robot_tf.transform.translation.y = 0.0;
    robot_tf.transform.translation.z = 0.0;
    robot_tf.transform.rotation.x = 0.0;
    robot_tf.transform.rotation.y = 0.0;
    robot_tf.transform.rotation.z = 0.0;
    robot_tf.transform.rotation.w = 1.0;
    broadcaster.init(nh);
}

static void initVelocityPublisher()
{
    current_vel.linear.x = 0;
    current_vel.linear.y = 0;
    current_vel.linear.z = 0;
    current_vel.angular.x = 0;
    current_vel.angular.y = 0;
    current_vel.angular.z = 0;
    vel_pub = new ros::Publisher("velocity", &current_vel);
    nh.advertise(*vel_pub);
}

static void initJointStatePublisher()
{
    joint_state_pub = new ros::Publisher("joint_states", &joint_states);
    nh.advertise(*joint_state_pub);

    joint_states.header.frame_id = "base_link";

    //assigning the arrays to the message
    joint_states.name = (char **)joint_state_name;
    joint_states.position = pos;
    // joint_states.velocity = vel;
    // joint_states.effort = eff;

    //setting the length
    joint_states.name_length = 4;
    joint_states.position_length = 4;
    // joint_states.velocity_length = 4;
    // joint_states.effort_length = 4;
}

static void velocityCallback(const geometry_msgs::Twist &twist_msg)
{
    RosbotDrive &drive = RosbotDrive::getInstance();
    RosbotSpeed speed = {static_cast<float>(twist_msg.linear.x), static_cast<float>(twist_msg.linear.y), static_cast<float>(twist_msg.angular.z)};
    rk->setRosbotSpeed(drive, speed);
    last_speed_command_time = odom_watchdog_timer.read_ms();
    is_speed_watchdog_active = false;
}

static void servoCallback(const std_msgs::UInt32 &ser_msg)
{
    int servo_num = ser_msg.data & 0xF;
    int servo_width = ser_msg.data >> 4;
    servo_manager.setWidth(servo_num - 1, servo_width);
}

/**
 * @brief Parse servo commands.
 * @param command string to be parsed
 * @return true if command was successfully parsed
 */
static bool servoCommandParser(const char *command)
{
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
    while (token != NULL)
    {
        if (sscanf(token, "%c:%d", &key, &value) == 2)
        {
            switch (key)
            {
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
        }
        else
            return false;
        token = strtok(NULL, " ");
    }

    if (servo_voltage != -1)
    {
        servo_manager.setPowerMode(servo_voltage);
    }

    if (servo_enabled != -1)
    {
        if (servo_num == -1)
            return false;

        servo_manager.enableOutput(servo_num, servo_enabled);
    }

    if (servo_period != -1)
    {
        if (servo_num == -1)
            return false;

        if (!servo_manager.setPeriod(servo_num, servo_period))
            return false;
    }

    if (servo_width != -1)
    {
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

static bool pidCommandParser(const char *command)
{
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
    while (token != NULL)
    {
        if (sscanf(token, "%[^:]s", key) != 1)
            return false;

        if (strcmp("kp", key) == 0)
        {
            if (sscanf(token, "kp:%f", &value) == 1)
                kp = value;
            else
                return false;
        }
        else if (strcmp("ki", key) == 0)
        {
            if (sscanf(token, "ki:%f", &value) == 1)
                ki = value;
            else
                return false;
        }
        else if (strcmp("kd", key) == 0)
        {
            if (sscanf(token, "kd:%f", &value) == 1)
                kd = value;
            else
                return false;
        }
        else if (strcmp("out_max", key) == 0)
        {
            if (sscanf(token, "out_max:%f", &value) == 1)
                out_max = min<float>(value, 0.80f);
            else
                return false;
        }
        else if (strcmp("out_min", key) == 0)
        {
            if (sscanf(token, "out_min:%f", &value) == 1)
                out_min = max<float>(value, -0.80f);
            else
                return false;
        }
        else if (strcmp("a_max", key) == 0)
        {
            if (sscanf(token, "a_max:%e", &value) == 1)
                a_max = value;
            else
                return false;
        }
        else if (strcmp("speed_max", key) == 0)
        {
            if (sscanf(token, "speed_max:%f", &value) == 1)
                speed_max = min<float>(value, 1.25f);
            else
                return false;
        }
        else
        {
            return false;
        }

        token = strtok(NULL, " ");
    }

    RosbotRegulator_params params;
    RosbotDrive::getInstance().getPidParams(params);

    if (ki != -1.0f)
    {
        params.ki = ki;
    }

    if (kp != -1.0f)
    {
        params.kp = kp;
    }

    if (kd != -1.0f)
    {
        params.kd = kd;
    }

    if (out_max != -1.0f)
    {
        params.out_max = out_max;
    }

    if (out_min != -2.0f)
    {
        params.out_min = out_min;
    }

    if (a_max != -1.0f)
    {
        params.a_max = a_max;
    }

    if (speed_max != -1.0f)
    {
        params.speed_max = speed_max;
    }

    RosbotDrive::getInstance().updatePidParams(params);
    return true;
}

class ConfigFunctionality
{
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
    map<std::string, configuration_srv_fun_t> _commands;
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

ConfigFunctionality::ConfigFunctionality()
{
    _commands[SLED_COMMAND] = &ConfigFunctionality::setLed;
    _commands[EIMU_COMMAND] = &ConfigFunctionality::enableImu;
    _commands[EDSE_COMMAND] = &ConfigFunctionality::enableDistanceSensors;
    _commands[EJSM_COMMAND] = &ConfigFunctionality::enableJointStates;
    _commands[RODOM_COMMAND] = &ConfigFunctionality::resetOdom;
    _commands[EWCH_COMMAND] = &ConfigFunctionality::enableSpeedWatchdog;
    _commands[RIMU_COMMAND] = &ConfigFunctionality::resetImu;
    _commands[SANI_COMMAND] = &ConfigFunctionality::setAnimation;
    _commands[ETFM_COMMAND] = &ConfigFunctionality::enableTfMessages;
    _commands[CALI_COMMAND] = &ConfigFunctionality::calibrateOdometry;
    _commands[EMOT_COMMAND] = &ConfigFunctionality::enableMotors;
    _commands[CSER_COMMAND] = &ConfigFunctionality::configureServo;
    _commands[GPID_COMMAND] = &ConfigFunctionality::getPid;
    _commands[CPID_COMMAND] = &ConfigFunctionality::configurePid;
    _commands[SKIN_COMMAND] = &ConfigFunctionality::setKinematics;
}

uint8_t ConfigFunctionality::enableTfMessages(const char *datain, const char **dataout)
{
    int en;
    if (sscanf(datain, "%d", &en) == 1)
    {
        tf_msgs_enabled = en ? true : false;
        if (tf_msgs_enabled)
        {
            initTfPublisher();
        }
        return rosbot_ekf::Configuration::Response::SUCCESS;
    }
    return rosbot_ekf::Configuration::Response::FAILURE;
}

uint8_t ConfigFunctionality::configurePid(const char *datain, const char **dataout)
{
    return pidCommandParser(datain) ? rosbot_ekf::Configuration::Response::SUCCESS : rosbot_ekf::Configuration::Response::FAILURE;
}

uint8_t ConfigFunctionality::getPid(const char *datain, const char **dataout)
{
    RosbotRegulator_params params;
    RosbotDrive::getInstance().getPidParams(params);
    sprintf(this->_buffer, "%kp:%.3f ki:%.3f kd:%.3f out_max:%.3f out_min:%.3f a_max:%.3e speed_max: %.3f",
            params.kp, params.ki, params.kd, params.out_max, params.out_min, params.a_max, params.speed_max);
    *dataout = this->_buffer;
    return rosbot_ekf::Configuration::Response::SUCCESS;
}

uint8_t ConfigFunctionality::configureServo(const char *datain, const char **dataout)
{
    return servoCommandParser(datain) ? rosbot_ekf::Configuration::Response::SUCCESS : rosbot_ekf::Configuration::Response::FAILURE;
}

uint8_t ConfigFunctionality::calibrateOdometry(const char *datain, const char **dataout)
{
    float diameter_modificator, tyre_deflation;
    if (sscanf(datain, "%f %f", &diameter_modificator, &tyre_deflation) == 2)
    {
        rk->calibrateOdometry(diameter_modificator, tyre_deflation);
        RosbotDrive &drive = RosbotDrive::getInstance();
        drive.updateWheelCoefficients(rosbot_kinematics::custom_wheel_params);
        return rosbot_ekf::Configuration::Response::SUCCESS;
    }
    return rosbot_ekf::Configuration::Response::FAILURE;
}

uint8_t ConfigFunctionality::enableMotors(const char *datain, const char **dataout)
{
    int en = atoi(datain);
    if (en)
        nh.loginfo("Motors connected.");
    else
        nh.loginfo("Motors disconnected.");
    RosbotDrive::getInstance().enable(en);
    return rosbot_ekf::Configuration::Response::SUCCESS;
}

//TODO change the implementation
uint8_t ConfigFunctionality::setAnimation(const char *datain, const char **dataout)
{
#if USE_WS2812B_ANIMATION_MANAGER
    Color_t color;
    switch (datain[0])
    {
    case 'O':
    case 'o':
        anim_manager->enableInterface(false);
        break;
    case 'F':
    case 'f':
        if (parseColorStr(&datain[2], &color))
            anim_manager->setFadingAnimation(&color);
        break;
    case 'B':
    case 'b':
        if (parseColorStr(&datain[2], &color))
            anim_manager->setBlinkingAnimation(&color);
        break;
    case 'R':
    case 'r':
        anim_manager->setRainbowAnimation();
        break;
    case 'S':
    case 's':
        if (parseColorStr(&datain[2], &color))
            anim_manager->setSolidColor(&color);
        break;
    default:
        return rosbot_ekf::Configuration::Response::FAILURE;
    }
#endif
    return rosbot_ekf::Configuration::Response::SUCCESS;
}

ConfigFunctionality::configuration_srv_fun_t ConfigFunctionality::findFunctionality(const char *command)
{
    std::map<std::string, configuration_srv_fun_t>::iterator it = _commands.find(command);
    if (it != _commands.end())
        return it->second;
    else
        return NULL;
}

uint8_t ConfigFunctionality::resetImu(const char *datain, const char **dataout)
{
    //TODO implement
    return rosbot_ekf::Configuration::Response::FAILURE;
}

uint8_t ConfigFunctionality::setMotorsAccelDeaccel(const char *datain, const char **dataout)
{
    float accel, deaccel;
    //TODO implement
    return rosbot_ekf::Configuration::Response::FAILURE;
}

uint8_t ConfigFunctionality::resetOdom(const char *datain, const char **dataout)
{
    RosbotDrive &drive = RosbotDrive::getInstance();
    rk->resetRosbotOdometry(drive, odometry);
    
    return rosbot_ekf::Configuration::Response::SUCCESS;
}

uint8_t ConfigFunctionality::setKinematics(const char *datain, const char **dataout)
{
    std::string data = datain;
    if (data == "DIFF")
    {
        nh.loginfo("Differential drive mode");
        rk->~RosbotKinematics();
        rk = rosbot_kinematics::RosbotKinematics::kinematicsType(0);
        rk->setOdomParams();
        RosbotDrive &drive = RosbotDrive::getInstance();
        rk->resetRosbotOdometry(drive, odometry);
    }
    else if (data == "MEC")
    {
        nh.loginfo("Mecanum drive mode");
        rk->~RosbotKinematics();
        rk = rosbot_kinematics::RosbotKinematics::kinematicsType(1);
        rk->setOdomParams();
        RosbotDrive &drive = RosbotDrive::getInstance();
        rk->resetRosbotOdometry(drive, odometry);
    }
    else
    {
        nh.loginfo("Unrecognized data, use DIFF or MEC, provided:");
        nh.loginfo(datain);
        return rosbot_ekf::Configuration::Response::FAILURE;
    }
    return rosbot_ekf::Configuration::Response::SUCCESS;
}

uint8_t ConfigFunctionality::enableImu(const char *datain, const char **dataout)
{
    int en;
    if (sscanf(datain, "%d", &en) == 1)
    {
        if (en)
        {
            imu_driver_ptr->start();
        }
        else
        {
            imu_driver_ptr->stop();
        }
        return rosbot_ekf::Configuration::Response::SUCCESS;
    }
    return rosbot_ekf::Configuration::Response::FAILURE;
}

uint8_t ConfigFunctionality::enableJointStates(const char *datain, const char **dataout)
{
    int en;
    if (sscanf(datain, "%d", &en) == 1)
    {
        joint_states_enabled = (en == 0 ? false : true);
        return rosbot_ekf::Configuration::Response::SUCCESS;
    }
    return rosbot_ekf::Configuration::Response::FAILURE;
}

uint8_t ConfigFunctionality::enableDistanceSensors(const char *datain, const char **dataout)
{
    int en;
    // if(sscanf(datain,"%d",&en) == 1)
    // {
    //     if(en == 0)
    //     {
    //         distance_sensors_enabled = false;
    //         for(int i=0;i<4;i++)
    //         {
    //             VL53L0X * sensor = distance_sensors->getSensor(i);
    //             sensor->stopContinuous();
    //         }
    //     }
    //     else
    //     {
    //         distance_sensors_enabled = true;
    //         for(int i=0;i<4;i++)
    //         {
    //             VL53L0X * sensor = distance_sensors->getSensor(i);
    //             sensor->setTimeout(50);
    //             sensor->startContinuous();
    //         }
    //     }
    //     return rosbot_ekf::Configuration::Response::SUCCESS;
    // }
    return rosbot_ekf::Configuration::Response::FAILURE;
}

uint8_t ConfigFunctionality::setLed(const char *datain, const char **dataout)
{
    int led_num, led_state;
    if (sscanf(datain, "%d %d", &led_num, &led_state) == 2)
    {
        switch (led_num)
        {
        case 2:
            led2 = led_state;
            return rosbot_ekf::Configuration::Response::SUCCESS;
        case 3:
            led3 = led_state;
            return rosbot_ekf::Configuration::Response::SUCCESS;
        default:
            break;
        }
    }
    return rosbot_ekf::Configuration::Response::FAILURE;
}

uint8_t ConfigFunctionality::enableSpeedWatchdog(const char *datain, const char **dataout)
{
    int en;
    if (sscanf(datain, "%d", &en) == 1)
    {
        is_speed_watchdog_enabled = (en == 0 ? false : true);
        return rosbot_ekf::Configuration::Response::SUCCESS;
    }
    return rosbot_ekf::Configuration::Response::FAILURE;
}

ConfigFunctionality *ConfigFunctionality::getInstance()
{
    if (_instance == NULL)
    {
        _instance = new ConfigFunctionality();
    }
    return _instance;
}

void responseCallback(const rosbot_ekf::Configuration::Request &req, rosbot_ekf::Configuration::Response &res)
{
    ConfigFunctionality *config_functionality = ConfigFunctionality::getInstance();
    ConfigFunctionality::configuration_srv_fun_t fun = config_functionality->findFunctionality(req.command);
    res.data = EMPTY_STRING;
    if (fun != NULL)
    {
        // nh.loginfo("Command found!");
        res.result = (config_functionality->*fun)(req.data, &res.data);
    }
    else
    {
        nh.loginfo("Command not found!");
        res.result = rosbot_ekf::Configuration::Response::COMMAND_NOT_FOUND;
    }
}

#if defined(MEMORY_DEBUG_INFO)
#define MAX_THREAD_INFO 10

mbed_stats_heap_t heap_info;
mbed_stats_stack_t stack_info[MAX_THREAD_INFO];

int print_debug_info()
{
    debug("\nThis message is from debug function");
    debug_if(1, "\nThis message is from debug_if function");
    debug_if(0, "\nSOMETHING WRONG!!! This message from debug_if function shouldn't show on bash");

    printf("\nMemoryStats:");
    mbed_stats_heap_get(&heap_info);
    printf("\n\tBytes allocated currently: %d", heap_info.current_size);
    printf("\n\tMax bytes allocated at a given time: %d", heap_info.max_size);
    printf("\n\tCumulative sum of bytes ever allocated: %d", heap_info.total_size);
    printf("\n\tCurrent number of bytes allocated for the heap: %d", heap_info.reserved_size);
    printf("\n\tCurrent number of allocations: %d", heap_info.alloc_cnt);
    printf("\n\tNumber of failed allocations: %d", heap_info.alloc_fail_cnt);

    mbed_stats_stack_get(&stack_info[0]);
    printf("\nCumulative Stack Info:");
    printf("\n\tMaximum number of bytes used on the stack: %d", stack_info[0].max_size);
    printf("\n\tCurrent number of bytes allocated for the stack: %d", stack_info[0].reserved_size);
    printf("\n\tNumber of stacks stats accumulated in the structure: %d", stack_info[0].stack_cnt);

    mbed_stats_stack_get_each(stack_info, MAX_THREAD_INFO);
    printf("\nThread Stack Info:");
    for (int i = 0; i < MAX_THREAD_INFO; i++)
    {
        if (stack_info[i].thread_id != 0)
        {
            printf("\n\tThread: %d", i);
            printf("\n\t\tThread Id: 0x%08X", stack_info[i].thread_id);
            printf("\n\t\tMaximum number of bytes used on the stack: %d", stack_info[i].max_size);
            printf("\n\t\tCurrent number of bytes allocated for the stack: %d", stack_info[i].reserved_size);
            printf("\n\t\tNumber of stacks stats accumulated in the structure: %d", stack_info[i].stack_cnt);
        }
    }

    printf("\nDone...\n\n");
}
#endif /* MEMORY_DEBUG_INFO */

int main()
{
    int spin_result;
    int err_msg = 0;
    uint32_t spin_count = 1;
    float curr_odom_calc_time, last_odom_calc_time = 0.0f;

    ThisThread::sleep_for(100);
    sens_power = 1; // sensors power on
    ThisThread::sleep_for(100);
    odom_watchdog_timer.start();

    rk = rosbot_kinematics::RosbotKinematics::kinematicsType(0);
    rk->setOdomParams();
    RosbotDrive &drive = RosbotDrive::getInstance();
    MultiDistanceSensor &distance_sensors = MultiDistanceSensor::getInstance();

    drive.setupMotorSequence(MOTOR_FR, MOTOR_FL, MOTOR_RR, MOTOR_RL);
    drive.init(rosbot_kinematics::custom_wheel_params, RosbotDrive::DEFAULT_REGULATOR_PARAMS);
    drive.enable(true);
    drive.enablePidReg(true);

    button1.mode(PullUp);
    button2.mode(PullUp);
    button1.fall(button1Callback);
    button2.fall(button2Callback);

    nh.initNode();

    bool distance_sensors_init_flag = false;
    bool imu_init_flag = false;
    bool welcome_flag = true;

    //TODO: add /diagnostic messages
    int num_sens_init;
    if ((num_sens_init = distance_sensors.init()) > 0)
    {
        distance_sensors_init_flag = true;
    }

    I2C *i2c_ptr = new I2C(IMU_I2C_SDA, IMU_I2C_SCL);
    i2c_ptr->frequency(IMU_I2C_FREQUENCY);

    ImuDriver::Type type = ImuDriver::getType(i2c_ptr, 2);
    sprintf(imu_description_string, "Detected sensor: %s\r\n", imu_sensor_type_string[type]);

    if (type != ImuDriver::UNKNOWN)
    {
        imu_driver_ptr = new ImuDriver(i2c_ptr, type);
        imu_driver_ptr->init();
        imu_driver_ptr->start();
        imu_init_flag = true;
    }

    ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &velocityCallback);
    ros::Subscriber<std_msgs::UInt32> cmd_ser_sub("cmd_ser", &servoCallback);
    ros::ServiceServer<rosbot_ekf::Configuration::Request, rosbot_ekf::Configuration::Response> config_srv("config", responseCallback);

    nh.advertiseService(config_srv);
    nh.subscribe(cmd_vel_sub);
    nh.subscribe(cmd_ser_sub);

    initBatteryPublisher();
    initPosePublisher();
    initVelocityPublisher();
    initRangePublisher();
    initJointStatePublisher();
    initImuPublisher();
    initButtonPublisher();

#if USE_WS2812B_ANIMATION_MANAGER
    anim_manager = AnimationManager::getInstance();
    anim_manager->init();
#endif

#if defined(MEMORY_DEBUG_INFO)
    print_debug_info();
#endif /* MEMORY_DEBUG_INFO */

    if (imu_init_flag)
        imu_driver_ptr->start();

    if (distance_sensors_init_flag)
    {
        uint8_t *data = distance_sensor_commands.alloc();
        *data = 1;
        distance_sensor_commands.put(data);
        distance_sensors_enabled = true;
    }

    while (1)
    {
        if (is_speed_watchdog_enabled)
        {
            if (!is_speed_watchdog_active && (odom_watchdog_timer.read_ms() - last_speed_command_time) > speed_watchdog_interval)
            {
                RosbotSpeed speed = {0.0, 0.0, 0.0};
                RosbotDrive &drive = RosbotDrive::getInstance();
                rk->setRosbotSpeed(drive, speed);
                is_speed_watchdog_active = true;
            }
        }

#if USE_WS2812B_ANIMATION_MANAGER
        if (!nh.connected())
            anim_manager->enableInterface(false);
#endif

        if (spin_count % 2 == 0)
        {
            curr_odom_calc_time = odom_watchdog_timer.read();
            RosbotDrive &drive = RosbotDrive::getInstance();
            rk->updateRosbotOdometry(drive, odometry, curr_odom_calc_time - last_odom_calc_time);
            last_odom_calc_time = curr_odom_calc_time;
        }

        if (button1_publish_flag)
        {
            button1_publish_flag = false;
            if (!button1)
            {
                button_msg.data = 1;
                if (nh.connected())
                    button_pub->publish(&button_msg);
            }
        }

        if (button2_publish_flag)
        {
            button2_publish_flag = false;
            if (!button2)
            {
                button_msg.data = 2;
                if (nh.connected())
                    button_pub->publish(&button_msg);
            }
        }

        if (spin_count % 5 == 0) /// cmd_vel, odometry, joint_states, tf messages
        {
            current_vel.linear.x = sqrt(odometry.robot_x_vel * odometry.robot_x_vel + odometry.robot_y_vel * odometry.robot_y_vel);
            current_vel.angular.z = odometry.robot_angular_vel;
            current_vel = rk->getTwist(odometry);
            pose.pose.position.x = odometry.robot_x_pos;
            pose.pose.position.y = odometry.robot_y_pos;
            pose.pose.orientation = tf::createQuaternionFromYaw(odometry.robot_angular_pos);

            pose.header.stamp = nh.now();
            if (nh.connected())
            {
                pose_pub->publish(&pose);
                vel_pub->publish(&current_vel);
            }

            if (joint_states_enabled)
            {
                pos[0] = odometry.wheel_FL_ang_pos;
                pos[1] = odometry.wheel_FR_ang_pos;
                pos[2] = odometry.wheel_RL_ang_pos;
                pos[3] = odometry.wheel_RR_ang_pos;
                joint_states.position = pos;
                joint_states.header.stamp = pose.header.stamp;
                if (nh.connected())
                    joint_state_pub->publish(&joint_states);
            }

            if (tf_msgs_enabled)
            {
                robot_tf.header.stamp = pose.header.stamp;
                robot_tf.transform.translation.x = pose.pose.position.x;
                robot_tf.transform.translation.y = pose.pose.position.y;
                robot_tf.transform.rotation.x = pose.pose.orientation.x;
                robot_tf.transform.rotation.y = pose.pose.orientation.y;
                robot_tf.transform.rotation.z = pose.pose.orientation.z;
                robot_tf.transform.rotation.w = pose.pose.orientation.w;
                if (nh.connected())
                    broadcaster.sendTransform(robot_tf);
            }
        }

        if (spin_count % 40 == 0)
        {
            battery_state.voltage = rosbot_sensors::updateBatteryWatchdog();
            if (nh.connected())
                battery_pub->publish(&battery_state);
        }

        osEvent evt1 = distance_sensor_mail_box.get(0);
        if (evt1.status == osEventMail)
        {
            SensorsMeasurement *message = (SensorsMeasurement *)evt1.value.p;
            if (message->status == MultiDistanceSensor::ERR_I2C_FAILURE)
            {
                err_msg++;
                if (distance_sensor_commands.empty() && err_msg == 3)
                {
                    if (nh.connected())
                        nh.logerror("I2C error. Restarting VL53L0X sensors...");
                    uint8_t *data = distance_sensor_commands.alloc();
                    *data = 2;
                    distance_sensor_commands.put(data);
                    data = distance_sensor_commands.alloc();
                    *data = 1;
                    distance_sensor_commands.put(data);
                    err_msg = 0;
                }
            }
            else
            {
                err_msg = 0;
                for (int i = 0; i < 4; i++)
                {
                    range_msg[i].header.stamp = nh.now(message->timestamp);
                    range_msg[i].range = message->range[i];
                    if (nh.connected())
                        range_pub[i]->publish(&range_msg[i]);
                }
            }
            distance_sensor_mail_box.free(message);
        }
        // }
        // if(spin_count % 20 == 0 && distance_sensors_enabled) // ~ 5 HZ
        // {
        //     uint16_t range;
        //     ros::Time t = nh.now();
        //     for(int i=0;i<4;i++)
        //     {
        //         range = distance_sensors->getSensor(i)->readRangeContinuousMillimeters(false);
        //         range_msg[i].header.stamp = t;
        //         range_msg[i].range = (range != 65535) ? (float)range/1000.0f : -1.0f;
        //         if(nh.connected()) range_pub[i]->publish(&range_msg[i]);
        //     }
        // }

        osEvent evt2 = imu_sensor_mail_box.get(0);

        if (evt2.status == osEventMail)
        {
            ImuDriver::ImuMesurement *message = (ImuDriver::ImuMesurement *)evt2.value.p;

            imu_msg.header.stamp = nh.now(message->timestamp);
            imu_msg.orientation.x = message->orientation[0];
            imu_msg.orientation.y = message->orientation[1];
            imu_msg.orientation.z = message->orientation[2];
            imu_msg.orientation.w = message->orientation[3];
            for (int i = 0; i < 3; i++)
            {
                imu_msg.angular_velocity[i] = message->angular_velocity[i];
                imu_msg.linear_acceleration[i] = message->linear_velocity[i];
            }
            imu_sensor_mail_box.free(message);
            if (nh.connected())
                imu_pub->publish(&imu_msg);
        }

        // LOGS
        if (nh.connected())
        {
            if (welcome_flag)
            {
                welcome_flag = false;
                nh.loginfo(WELLCOME_STR);
                if (!distance_sensors_init_flag)
                    nh.logerror("VL53L0X sensors initialisation failure!");
                if (!imu_init_flag)
                    nh.logerror("No IMU sensor detected!");
                else
                    nh.loginfo(imu_description_string);
            }
        }
        else
        {
            welcome_flag = true;
        }

        nh.spinOnce();

        // if((spin_result=nh.spinOnce()) != ros::SPIN_OK)
        // {
        //     // nh.logwarn(spin_result == -1 ? "SPIN_ERR" : "SPIN_TIMEOUT");
        //     do {}while(0); // do nothing at the moment
        // }
        spin_count++;
        ThisThread::sleep_for(MAIN_LOOP_INTERVAL_MS);
    }
}