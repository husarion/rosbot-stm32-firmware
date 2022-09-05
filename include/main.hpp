#pragma once
#include <ImuDriver.h>
#include <RosbotDrive.h>
#include <microros.hpp>
#include <memory_debug_message_info.hpp>

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

constexpr float WHEEL_RADIUS = 0.0425;
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

static rosbot_sensors::ServoManger servo_manager;
static rosbot_kinematics::RosbotOdometry odometry;
static rosbot_kinematics::DifferentialDrive diff_drive_kinematics;
static rosbot_kinematics::MecanumDrive mecanum_drive_kinematics;
static DigitalOut sens_power(SENS_POWER_ON, 0);

static InterruptIn button1(BUTTON1);
static InterruptIn button2(BUTTON2);
static ImuDriver *imu_driver_ptr;

#if KINEMATIC_TYPE
static rosbot_kinematics::RosbotKinematics *rk = &mecanum_drive_kinematics;
#else
static rosbot_kinematics::RosbotKinematics *rk = &diff_drive_kinematics;
#endif