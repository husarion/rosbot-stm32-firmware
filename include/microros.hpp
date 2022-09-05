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
#include <std_msgs/msg/float32_multi_array.h>

#include <microros_transport/mbed_serial_transport.hpp>

constexpr const char *NODE_NAME = "rosbot_stm32_firmware";
constexpr const char *IMU_TOPIC_NAME = "imu";
constexpr const char *WHEELS_STATE_TOPIC_NAME = "wheels_state";
constexpr const char *BATTERY_TOPIC_NAME = "battery";
constexpr const char *WHEELS_COMMAND_TOPIC_NAME = "wheels_command";

constexpr const char *FRONT_LEFT_MOTOR_NAME = "front_left_wheel_joint";
constexpr const char *FRONT_RIGHT_MOTOR_NAME = "front_right_wheel_joint";
constexpr const char *REAR_LEFT_MOTOR_NAME = "rear_left_wheel_joint";
constexpr const char *REAR_RIGHT_MOTOR_NAME = "rear_right_wheel_joint";

enum Motors {
    motor_left_front,
    motor_right_front,
    motor_left_rear,
    motor_right_rear,
    MOTORS_COUNT
};

enum MotorsState {
    motor_state_position,
    motor_state_velocity,
    motor_state_effort,
    MOTORS_STATE_COUNT
};

static DigitalOut led2(LED2, 0);
static DigitalOut led3(LED3, 0);
void microros_deinit();

#define RCCHECK(fn)                          \
    {                                        \
        rcl_ret_t temp_rc = fn;              \
        if ((temp_rc != RCL_RET_OK)) {       \
            led2 = 1;                        \
            led3 = 1;                        \
            for (auto i = 0u; i < 10; ++i) { \
                ThisThread::sleep_for(500);  \
                led2 = !led2;                \
                led3 = !led3;                \
            }                                \
            led2 = 0;                        \
            led3 = 0;                        \
            ThisThread::sleep_for(100);      \
            microros_deinit();               \
            NVIC_SystemReset();              \
        }                                    \
    }

#define RCSOFTCHECK(fn)                     \
    {                                       \
        rcl_ret_t temp_rc = fn;             \
        if ((temp_rc != RCL_RET_OK)) {      \
            led2 = 1;                       \
            led3 = 1;                       \
            for (auto i = 0u; i < 4; ++i) { \
                ThisThread::sleep_for(500); \
                led2 = !led2;               \
                led3 = !led3;               \
            }                               \
            led2 = 0;                       \
            led3 = 0;                       \
            ThisThread::sleep_for(100);     \
            microros_deinit();              \
            NVIC_SystemReset();             \
        }                                   \
    }

bool microros_init();
void microros_deinit();
void microros_spin();

void init_imu_publisher();
void init_wheels_state_publisher();
void init_battery_publisher();
void init_wheels_command_subscriber();

void fill_wheels_state_msg(sensor_msgs__msg__JointState *msg);
void fill_imu_msg(sensor_msgs__msg__Imu *msg);
void fill_battery_msg(sensor_msgs__msg__BatteryState *msg);
void fill_wheels_command_msg(std_msgs__msg__Float32MultiArray *msg);

void publish_imu_msg(sensor_msgs__msg__Imu *imu_msg);
void publish_wheels_state_msg(sensor_msgs__msg__JointState *msg);
void publish_battery_msg(sensor_msgs__msg__BatteryState *msg);
