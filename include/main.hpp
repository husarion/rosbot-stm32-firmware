#pragma once
#include <ImuDriver.h>
#include <RosbotDrive.h>

#include <memory_debug_message_info.hpp>
#include <microros.hpp>

#ifndef KINEMATIC_TYPE
#define KINEMATIC_TYPE 0
#endif

#ifndef JOINT_STATES_ENABLE
#define JOINT_STATES_ENABLE 0
#endif

#if JOINT_STATES_ENABLE
static volatile bool joint_states_enabled = true;
#else
static volatile bool joint_states_enabled = false;
#endif
static UARTSerial microros_serial(RPI_SERIAL_TX, RPI_SERIAL_RX);

static volatile bool distance_sensors_enabled = false;
static volatile bool tf_msgs_enabled = false;

static volatile bool button1_publish_flag = false;
static volatile bool button2_publish_flag = false;
static volatile float battery_voltage = 0.0;

static volatile bool is_speed_watchdog_enabled = true;
static volatile bool is_speed_watchdog_active = false;
static uint64_t speed_watchdog_interval = 1000;  // ms

static Timer odom_watchdog_timer;
static volatile uint32_t last_speed_command_time = 0;

static DigitalOut sens_power(SENS_POWER_ON, 0);

static InterruptIn button1(BUTTON1);
static InterruptIn button2(BUTTON2);
static ImuDriver *imu_driver_ptr;

// Motors setup
#define MOTOR_FR MOTOR1
#define MOTOR_FL MOTOR4
#define MOTOR_RR MOTOR2
#define MOTOR_RL MOTOR3

constexpr uint8_t POLARITY = 0b00111100;
constexpr float ROBOT_LENGTH = 0.197;
constexpr uint8_t ENCODER_CPR = 48;
constexpr float ROBOT_LENGTH_HALF = ROBOT_LENGTH / 2.0;
constexpr float DISTANCE_FRONT_TO_REAR_WHEEL = 0.11;
constexpr float WHEEL_SEPARATION_LENGTH = DISTANCE_FRONT_TO_REAR_WHEEL / 2;
constexpr float ROBOT_WIDTH = 0.215;  // 0.22 0.195
constexpr float ROBOT_WIDTH_HALF = ROBOT_WIDTH / 2.0;
constexpr float DIAMETER_MODIFICATOR = 1.106;  // 1.24, 1.09, 1.164
constexpr float TYRE_DEFLATION = 1.042;        // theoretical distance / real distance
constexpr float GEAR_RATIO = 34.014;
constexpr float WHEEL_DIAMETER = 0.085;
constexpr float WHEEL_RADIUS = WHEEL_DIAMETER / 2.0;