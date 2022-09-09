#include <microros.hpp>

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t rcl_allocator;
rcl_node_t node;
rcl_timer_t timer;

rcl_publisher_t imu_pub;
rcl_publisher_t wheels_state_pub;
rcl_publisher_t battery_pub;
rcl_subscription_t wheels_command_sub;
std_msgs__msg__Float32MultiArray wheels_command_msg;

// Range
const char *range_id[] = {"range_fr", "range_fl", "range_rr", "range_rl"};
const char *range_pub_names[] = {"range/fr", "range/fl", "range/rr", "range/rl"};
extern void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
extern void wheels_command_callback(const void *msgin);

bool microros_init() {
    fill_wheels_command_msg(&wheels_command_msg);
    rcl_allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &rcl_allocator));
    RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

    init_wheels_command_subscriber();
    init_imu_publisher();
    init_battery_publisher();
    init_wheels_state_publisher();

    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(10),
                                    timer_callback));

    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &rcl_allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &wheels_command_sub, &wheels_command_msg,
                                           &wheels_command_callback, ON_NEW_DATA));
    RCCHECK(rmw_uros_sync_session(500));
    return true;
}

void microros_deinit() {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    (rcl_subscription_fini(&wheels_command_sub, &node));
    (rcl_publisher_fini(&wheels_state_pub, &node));
    (rcl_publisher_fini(&imu_pub, &node));
    (rclc_executor_fini(&executor));
    (rcl_node_fini(&node));
}

void microros_spin() {
    if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)) != RCL_RET_OK) {
        microros_deinit();
        NVIC_SystemReset();
    }
}

void init_imu_publisher() {
    RCCHECK(rclc_publisher_init_best_effort(
        &imu_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        IMU_TOPIC_NAME));
}

void init_wheels_state_publisher() {
    RCCHECK(rclc_publisher_init_default(
        &wheels_state_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        WHEELS_STATE_TOPIC_NAME));
}

void init_battery_publisher() {
    RCCHECK(rclc_publisher_init_default(
        &battery_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
        BATTERY_TOPIC_NAME));
}

void init_wheels_command_subscriber() {
    RCCHECK(rclc_subscription_init_best_effort(
        &wheels_command_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        WHEELS_COMMAND_TOPIC_NAME));
}

void publish_wheels_state_msg(sensor_msgs__msg__JointState *msg) {
    RCSOFTCHECK(rcl_publish(&wheels_state_pub, msg, NULL));
}

void publish_imu_msg(sensor_msgs__msg__Imu *msg) {
    RCSOFTCHECK(rcl_publish(&imu_pub, msg, NULL));
}

void publish_battery_msg(sensor_msgs__msg__BatteryState *msg){
    RCSOFTCHECK(rcl_publish(&battery_pub, msg, NULL));
}

void fill_wheels_state_msg(sensor_msgs__msg__JointState *msg) {
    static double msg_data_tab[MOTORS_STATE_COUNT][MOTORS_COUNT];
    static rosidl_runtime_c__String msg_name_tab[MOTORS_COUNT];
    char *frame_id = (char *)"wheels_state";
    msg->header.frame_id.data = frame_id;
    msg->position.data = msg_data_tab[motor_state_position];
    msg->position.capacity = msg->position.size = MOTORS_COUNT;
    msg->velocity.data = msg_data_tab[motor_state_velocity];
    msg->velocity.capacity = msg->velocity.size = MOTORS_COUNT;
    msg->effort.data = msg_data_tab[motor_state_effort];
    msg->effort.capacity = msg->effort.size = MOTORS_COUNT;
    msg->header.frame_id.capacity = msg->header.frame_id.size = strlen((const char *)frame_id);

    msg_name_tab->capacity = msg_name_tab->size = MOTORS_COUNT;
    msg_name_tab[motor_left_front].data = (char *)FRONT_LEFT_MOTOR_NAME;
    msg_name_tab[motor_right_front].data = (char *)FRONT_RIGHT_MOTOR_NAME;
    msg_name_tab[motor_left_rear].data = (char *)REAR_LEFT_MOTOR_NAME;
    msg_name_tab[motor_right_rear].data = (char *)REAR_RIGHT_MOTOR_NAME;
    for (uint8_t i = 0; i < MOTORS_COUNT; i++) {
        msg_name_tab[i].capacity = msg_name_tab[i].size = strlen(msg_name_tab[i].data);
    }
    msg->name.capacity = msg->name.size = MOTORS_COUNT;
    msg->name.data = msg_name_tab;

    if (rmw_uros_epoch_synchronized()) {
        msg->header.stamp.sec = (int32_t)(rmw_uros_epoch_nanos() / 1000000000);
        msg->header.stamp.nanosec = (uint32_t)(rmw_uros_epoch_nanos() % 1000000000);
    }
}

void fill_imu_msg(sensor_msgs__msg__Imu *msg) {
    char *frame_id = (char *)"imu";
    msg->header.frame_id.data = frame_id;

    if (rmw_uros_epoch_synchronized()) {
        msg->header.stamp.sec = (int32_t)(rmw_uros_epoch_nanos() / 1000000000);
        msg->header.stamp.nanosec = (uint32_t)(rmw_uros_epoch_nanos() % 1000000000);
    }
    msg->orientation.x = 0;
    msg->orientation.y = 0;
    msg->orientation.z = 0;
    msg->orientation.w = 1;
    msg->angular_velocity.x = 0;
    msg->angular_velocity.y = 0;
    msg->angular_velocity.z = 0;
    msg->linear_acceleration.x = 0;
    msg->linear_acceleration.y = 0;
    msg->linear_acceleration.z = 0;
    for (auto i = 0u; i < 9u; ++i) {
        msg->angular_velocity_covariance[i] = msg->linear_acceleration_covariance[i] = msg->orientation_covariance[i] = 0.0;
    }
    msg->orientation_covariance[9] = 0.0;
    msg->orientation_covariance[10] = 0.0;
    msg->orientation_covariance[11] = 0.0;
}

void fill_battery_msg(sensor_msgs__msg__BatteryState *msg){
    char *frame_id = (char *)"battery";
    msg->header.frame_id.data = frame_id;

    if (rmw_uros_epoch_synchronized()) {
        msg->header.stamp.sec = (int32_t)(rmw_uros_epoch_nanos() / 1000000000);
        msg->header.stamp.nanosec = (uint32_t)(rmw_uros_epoch_nanos() % 1000000000);
    }

    msg->power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LION;
}

void fill_wheels_command_msg(std_msgs__msg__Float32MultiArray *msg) {
    static float data[4] = {0, 0, 0, 0};
    msg->data.capacity = 4;
    msg->data.size = 0;
    msg->data.data = (float *)data;
}