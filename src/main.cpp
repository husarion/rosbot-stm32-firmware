/** @file main.cpp
 * ROSbot firmware.
 *
 * @author Husarion
 * @copyright MIT
 */
#include <rosbot_sensors.h>

#include <main.hpp>

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

static float curr_odom_calc_time = 0.0;
static float last_odom_calc_time = 0.0;

sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__BatteryState battery_msg;
sensor_msgs__msg__JointState wheels_state_msg;
sensor_msgs__msg__Range range_msgs[RANGE_COUNT];

static uint32_t spin_count = 1;

static void button1Callback() {
    button1_publish_flag = true;
}

static void button2Callback() {
    button2_publish_flag = true;
}

void range_sensors_msg_handler() {
    osEvent evt1 = distance_sensor_mail_box.get(0);
    if (evt1.status == osEventMail) {
        SensorsMeasurement *message = (SensorsMeasurement *)evt1.value.p;
        for (auto i = 0u; i < RANGE_COUNT; ++i) {
            fill_range_msg(&range_msgs[i], i);
            range_msgs[i].range = message->range[i];
            publish_range_msg(&range_msgs[i], i);
        }

        distance_sensor_mail_box.free(message);
    }
}

void imu_msg_handler() {
    osEvent evt2 = imu_sensor_mail_box.get(0);

    if (evt2.status == osEventMail) {
        ImuDriver::ImuMesurement *message = (ImuDriver::ImuMesurement *)evt2.value.p;
        fill_imu_msg(&imu_msg);
        imu_msg.orientation.y = message->orientation[1];
        imu_msg.orientation.z = message->orientation[2];
        imu_msg.orientation.x = message->orientation[0];
        imu_msg.orientation.w = message->orientation[3];

        imu_msg.angular_velocity.x = message->angular_velocity[0];
        imu_msg.angular_velocity.y = message->angular_velocity[1];
        imu_msg.angular_velocity.z = message->angular_velocity[2];

        imu_msg.linear_acceleration.x = message->linear_acceleration[0];
        imu_msg.linear_acceleration.y = message->linear_acceleration[1];
        imu_msg.linear_acceleration.z = message->linear_acceleration[2];
        publish_imu_msg(&imu_msg);
        imu_sensor_mail_box.free(message);
    }
}

void battery_msg_handler() {
    if (spin_count % 40 == 0) {
        fill_battery_msg(&battery_msg);
        battery_msg.voltage = battery_voltage;
        publish_battery_msg(&battery_msg);
    }
}

void buttons_msgs_handler() {
    // TODO: fill
}

void wheels_state_msg_handler() {
    if (spin_count % 5 == 0) {
        publish_wheels_state_msg(&wheels_state_msg);
    }
}

void read_and_show_battery_state() {
    battery_voltage = rosbot_sensors::updateBatteryWatchdog();
}

void check_speed_watchdog() {
    if (is_speed_watchdog_enabled) {
        if (!is_speed_watchdog_active && (odom_watchdog_timer.read_ms() - last_speed_command_time) > speed_watchdog_interval) {
            RosbotDrive &drive = RosbotDrive::getInstance();
            NewTargetSpeed new_speed;
            new_speed.mode = MPS;
            new_speed.speed[0] = 0;
            new_speed.speed[1] = 0;
            new_speed.speed[2] = 0;
            new_speed.speed[3] = 0;

            drive.updateTargetSpeed(new_speed);
            is_speed_watchdog_active = true;
        }
    }
}

void update_odometry() {
    if (spin_count % 2 == 0) {
        RosbotDrive &drive = RosbotDrive::getInstance();

        float current_position[MOTORS_COUNT];
        current_position[motor_left_front] = drive.getAngularPos(MOTOR_FL);
        current_position[motor_right_front] = drive.getAngularPos(MOTOR_FR);
        current_position[motor_left_rear] = drive.getAngularPos(MOTOR_RL);
        current_position[motor_right_rear] = drive.getAngularPos(MOTOR_RR);

        curr_odom_calc_time = odom_watchdog_timer.read();
        double dt = curr_odom_calc_time - last_odom_calc_time;
        last_odom_calc_time = curr_odom_calc_time;

        for (auto i = 0u; i < MOTORS_COUNT; ++i) {
            wheels_state_msg.velocity.data[i] = (current_position[i] - wheels_state_msg.position.data[i]) / dt;
            wheels_state_msg.position.data[i] = current_position[i];
        }
    }
}

void wheels_command_callback(const void *msgin) {
    const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    if (not msg == NULL and msg->data.size == 4) {
        RosbotDrive &drive = RosbotDrive::getInstance();
        NewTargetSpeed new_speed;
        new_speed.mode = MPS;
        new_speed.speed[0] = msg->data.data[motor_right_front] * WHEEL_RADIUS;
        new_speed.speed[1] = msg->data.data[motor_right_rear] * WHEEL_RADIUS;
        new_speed.speed[2] = msg->data.data[motor_left_rear] * WHEEL_RADIUS;
        new_speed.speed[3] = msg->data.data[motor_left_front] * WHEEL_RADIUS;

        drive.updateTargetSpeed(new_speed);
        last_speed_command_time = odom_watchdog_timer.read_ms();
        is_speed_watchdog_active = false;
    }
}

void odometry_callback() {
    while (true) {
        check_speed_watchdog();
        read_and_show_battery_state();
        update_odometry();
        ThisThread::sleep_for(10);
    }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        led3 = !led3;
        imu_msg_handler();
        wheels_state_msg_handler();
        buttons_msgs_handler();
        range_sensors_msg_handler();
        battery_msg_handler();
        spin_count++;
    }
}

int main() {
    ThisThread::sleep_for(100);
    sens_power = 1;  // sensors power on
    ThisThread::sleep_for(100);
    odom_watchdog_timer.start();

    RosbotDrive &drive = RosbotDrive::getInstance();
    MultiDistanceSensor &distance_sensors = MultiDistanceSensor::getInstance();

    RosbotWheel custom_wheel_params = {
        .radius = WHEEL_RADIUS,
        .diameter_modificator = DIAMETER_MODIFICATOR,
        .tyre_deflation = TYRE_DEFLATION,
        .gear_ratio = GEAR_RATIO,
        .encoder_cpr = ENCODER_CPR,
        .polarity = POLARITY};

    drive.setupMotorSequence(MOTOR_FR, MOTOR_FL, MOTOR_RR, MOTOR_RL);
    drive.init(custom_wheel_params, RosbotDrive::DEFAULT_REGULATOR_PARAMS);
    drive.enable(true);
    drive.enablePidReg(true);

    button1.mode(PullUp);
    button2.mode(PullUp);
    button1.fall(button1Callback);
    button2.fall(button2Callback);

    bool distance_sensors_init_flag = false;
    bool imu_init_flag = false;

    // TODO: add /diagnostic messages
    int num_sens_init;
    if ((num_sens_init = distance_sensors.init()) > 0) {
        distance_sensors_init_flag = true;
    }

    I2C *i2c_ptr = new I2C(IMU_I2C_SDA, IMU_I2C_SCL);
    i2c_ptr->frequency(IMU_I2C_FREQUENCY);

    ImuDriver::Type type = ImuDriver::getType(i2c_ptr, 2);
    sprintf(imu_description_string, "Detected sensor: %s\r\n", imu_sensor_type_string[type]);

    if (type != ImuDriver::UNKNOWN) {
        imu_driver_ptr = new ImuDriver(i2c_ptr, type);
        imu_driver_ptr->init();
        imu_driver_ptr->start();
        imu_init_flag = true;
    }

    Thread odometry_thread;
    odometry_thread.start(odometry_callback);

    if (imu_init_flag) {
        imu_driver_ptr->start();
    }

    if (distance_sensors_init_flag) {
        uint8_t *data = distance_sensor_commands.alloc();
        *data = 1;
        distance_sensor_commands.put(data);
        distance_sensors_enabled = true;
    }

    read_and_show_battery_state();

    set_microros_serial_transports(&microros_serial);
    while (not rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
        read_and_show_battery_state();
        ThisThread::sleep_for(100);
    }

    if (not microros_init()) {
        microros_deinit();
        led2 = 1;
        led3 = 1;
        ThisThread::sleep_for(2000);

        NVIC_SystemReset();
    }

    fill_imu_msg(&imu_msg);
    fill_battery_msg(&battery_msg);
    fill_wheels_state_msg(&wheels_state_msg);
    for (auto i = 0u; i < RANGE_COUNT; ++i) {
        fill_range_msg(&range_msgs[i], i);
    }

    std::size_t ping_count = 0;
    AgentStates state = AGENT_CONNECTED;
    while (state == AGENT_CONNECTED) {
        EXECUTE_EVERY_N_MS(2000, state = (RMW_RET_OK == rmw_uros_ping_agent(200, 5)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        microros_spin();
        led2 = 1;
    }
    led3 = 0;
    for (int i = 0; i < 10; ++i) {
        led2 = !led2;
        ThisThread::sleep_for(200);
    }
    microros_deinit();
    NVIC_SystemReset();
}
