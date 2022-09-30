#pragma once
#include <RosbotDrive.h>
#include <mbed.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/u_int16.h>
#include <microros_transport/mbed_serial_transport.hpp>

constexpr const char *NODE_NAME = "rosbot_stm32_firmware";
constexpr const char *IMU_TOPIC_NAME = "_imu/data_raw";
constexpr const char *WHEELS_STATE_TOPIC_NAME = "_motors_response";
constexpr const char *BATTERY_TOPIC_NAME = "battery";
constexpr const char *WHEELS_COMMAND_TOPIC_NAME = "_motors_cmd";

constexpr const char *FRONT_LEFT_MOTOR_NAME = "front_left_wheel_joint";
constexpr const char *FRONT_RIGHT_MOTOR_NAME = "front_right_wheel_joint";
constexpr const char *REAR_LEFT_MOTOR_NAME = "rear_left_wheel_joint";
constexpr const char *REAR_RIGHT_MOTOR_NAME = "rear_right_wheel_joint";

enum Buttons{
    button_left,
    button_right,
    BUTTONS_COUNT
};

enum Ranges {
    range_right_front,
    range_left_front,
    range_right_rear,
    range_left_rear,
    RANGES_COUNT
};

enum Motors {
    motor_right_rear,
    motor_left_rear,
    motor_right_front,
    motor_left_front,
    MOTORS_COUNT
};

enum MotorsState {
    motor_state_position,
    motor_state_velocity,
    motor_state_effort,
    MOTORS_STATE_COUNT
};

enum AgentStates {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};

static DigitalOut led2(LED2, 0);
static DigitalOut led3(LED3, 0);
void microros_deinit();

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){led3 = 1;}}



#define EXECUTE_EVERY_N_MS(MS, X)          \
    do {                                   \
        static volatile int64_t init = -1; \
        if (init == -1) {                  \
            init = uxr_millis();           \
        }                                  \
        if (uxr_millis() - init > MS) {    \
            X;                             \
            init = uxr_millis();           \
        }                                  \
    } while (0)

void error_loop();

bool microros_init();
void microros_deinit();
bool microros_spin();

bool init_imu_publisher();
bool init_wheels_state_publisher();
bool init_battery_publisher();
bool init_range_publishers();
bool init_wheels_command_subscriber();
bool init_button_publishers();

void fill_wheels_state_msg(sensor_msgs__msg__JointState *msg);
void fill_imu_msg(sensor_msgs__msg__Imu *msg);
void fill_battery_msg(sensor_msgs__msg__BatteryState *msg);
void fill_wheels_command_msg(std_msgs__msg__Float32MultiArray *msg);
void fill_range_msg(sensor_msgs__msg__Range *msg, uint8_t id);

void publish_imu_msg(sensor_msgs__msg__Imu *imu_msg);
void publish_wheels_state_msg(sensor_msgs__msg__JointState *msg);
void publish_battery_msg(sensor_msgs__msg__BatteryState *msg);
void publish_range_msg(sensor_msgs__msg__Range *msg, uint8_t id);
void publish_button_msg(std_msgs__msg__UInt16 *msg, uint8_t id);
