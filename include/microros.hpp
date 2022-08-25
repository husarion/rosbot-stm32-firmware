#pragma once
#include <mbed.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>

#include <microros_transport/mbed_serial_transport.hpp>

constexpr const char *NODE_NAME = "rosbot_stm32_firmware";
constexpr const char *IMU_TOPIC_NAME = "imu";
constexpr const char *FRONT_LEFT_MOTOR_NAME = "front_left_wheel_joint";
constexpr const char *FRONT_RIGHT_MOTOR_NAME = "front_right_wheel_joint";
constexpr const char *REAR_LEFT_MOTOR_NAME = "rear_left_wheel_joint";
constexpr const char *REAR_RIGHT_MOTOR_NAME = "rear_right_wheel_joint";

enum Motors {
    motor_left_front, motor_right_front, motor_left_back, motor_right_back, MOTORS_COUNT
};

enum MotorsState {
    motor_state_position, motor_state_velocity, motor_state_effort, MOTORS_STATE_COUNT
};

static DigitalOut led2(LED2, 0);
static DigitalOut led3(LED3, 0);
#define RCCHECK(fn)                         \
    {                                       \
        rcl_ret_t temp_rc = fn;             \
        if ((temp_rc != RCL_RET_OK)) {      \
            /* todo: fill*/                 \
            while (true) {                  \
                led2 = !led2;               \
                led3 = !led3;               \
                ThisThread::sleep_for(100); \
            }                               \
        }                                   \
    }
#define RCSOFTCHECK(fn)                     \
    {                                       \
        rcl_ret_t temp_rc = fn;             \
        if ((temp_rc != RCL_RET_OK)) {      \
            while (true) {                  \
                led2 = !led2;               \
                led3 = !led3;               \
                ThisThread::sleep_for(500); \
            }                               \
        }                                   \
    }

static rclc_executor_t executor;
static rclc_support_t support;
static rcl_allocator_t rcl_allocator;
static rcl_node_t node;
static rcl_publisher_t imu_pub;
static sensor_msgs__msg__Imu imu_msg;
static sensor_msgs__msg__JointState wheel_state_msg;

static void init_imu_publisher() {
    RCCHECK(rclc_publisher_init_default(
        &imu_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        IMU_TOPIC_NAME));
    imu_msg.header.frame_id.data = (char *)"imu";
}

static void init_joint_state_publisher(){
    static double msg_data_tab[MOTORS_STATE_COUNT][MOTORS_COUNT];
    static rosidl_runtime_c__String msg_name_tab[MOTORS_COUNT];
    char *frame_id = (char *)"motors_state";
    wheel_state_msg.header.frame_id.data = frame_id;
    wheel_state_msg.position.data = msg_data_tab[motor_state_position];
    wheel_state_msg.position.capacity = wheel_state_msg.position.size = MOTORS_COUNT;
    wheel_state_msg.velocity.data = msg_data_tab[motor_state_velocity];
    wheel_state_msg.velocity.capacity = wheel_state_msg.velocity.size = MOTORS_COUNT;
    wheel_state_msg.effort.data = msg_data_tab[motor_state_effort];
    wheel_state_msg.effort.capacity = wheel_state_msg.effort.size = MOTORS_COUNT;
    wheel_state_msg.header.frame_id.capacity = wheel_state_msg.header.frame_id.size = strlen((const char *)frame_id);

    msg_name_tab->capacity = msg_name_tab->size = MOTORS_COUNT;
    msg_name_tab[motor_left_front].data = (char *)FRONT_LEFT_MOTOR_NAME;
    msg_name_tab[motor_right_front].data = (char *)FRONT_RIGHT_MOTOR_NAME;
    msg_name_tab[motor_left_back].data = (char *)REAR_LEFT_MOTOR_NAME;
    msg_name_tab[motor_right_back].data = (char *)REAR_RIGHT_MOTOR_NAME;
    for (uint8_t i = 0; i < MOTORS_COUNT; i++) {
        msg_name_tab[i].capacity = msg_name_tab[i].size = strlen(msg_name_tab[i].data);
    }
    wheel_state_msg.name.capacity = wheel_state_msg.name.size = MOTORS_COUNT;
    wheel_state_msg.name.data = msg_name_tab;
}

static bool microros_init() {
    rcl_allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &rcl_allocator));
    RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &rcl_allocator));

    init_imu_publisher();

    RCCHECK(rmw_uros_sync_session(1000));
    return true;
}
// static void microros_deinit();
// static void microros_spin();
