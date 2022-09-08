/** @file main.cpp
 * ROSbot firmware.
 *
 * @author Husarion
 * @copyright MIT
 */
#include <main.hpp>
#include <rosbot_sensors.h>

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

static void button1Callback() {
    button1_publish_flag = true;
}

static void button2Callback() {
    button2_publish_flag = true;
}


static float curr_odom_calc_time = 0.0;
static float last_odom_calc_time = 0.0;

// Range
const char *range_id[] = {"range_fr", "range_fl", "range_rr", "range_rl"};
const char *range_pub_names[] = {"range/fr", "range/fl", "range/rr", "range/rl"};

static uint32_t spin_count = 1;

void range_sensors_msg_handler() {
    // TODO: fill
}
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__BatteryState battery_msg;
sensor_msgs__msg__JointState wheels_state_msg;

void imu_msg_handler() {
    osEvent evt2 = imu_sensor_mail_box.get(0);

    if (evt2.status == osEventMail) {
        ImuDriver::ImuMesurement *message = (ImuDriver::ImuMesurement *)evt2.value.p;
        led2 = !led2;
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
    if (msg->data.size == 4) {
        led3 = !led3;
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
    bool welcome_flag = true;

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

    set_microros_serial_transports(&microros_serial);
    microros_init();

    if (imu_init_flag) {
        imu_driver_ptr->start();
    }

    if (distance_sensors_init_flag) {
        uint8_t *data = distance_sensor_commands.alloc();
        *data = 1;
        distance_sensor_commands.put(data);
        distance_sensors_enabled = true;
    }

    fill_imu_msg(&imu_msg);
    fill_battery_msg(&battery_msg);
    fill_wheels_state_msg(&wheels_state_msg);

    while (1) {
        microros_spin();
    }
}
