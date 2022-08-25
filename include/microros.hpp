#pragma once
#include <mbed.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>

#include <microros_transport/mbed_serial_transport.hpp>

constexpr const char *NODE_NAME = "rosbot_stm32_firmware";
constexpr const char *IMU_TOPIC_NAME = "imu";
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
// static rcl_timer_t timer;
static rcl_publisher_t imu_pub;
static sensor_msgs__msg__Imu imu_msg;
static void init_imu_publisher();

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
static void init_imu_publisher() {
    RCCHECK(rclc_publisher_init_default(
        &imu_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        IMU_TOPIC_NAME));
    imu_msg.header.frame_id.data = (char *)"imu";
}