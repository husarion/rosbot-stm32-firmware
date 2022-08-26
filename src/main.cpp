/** @file main.cpp
 * ROSbot firmware.
 *
 * @author Husarion
 * @copyright MIT
 */
#include <ImuDriver.h>
#include <geometry_msgs/msg/twist.h>
#include <rosbot_kinematics.h>
#include <rosbot_sensors.h>

#include <main.hpp>

// #include <geometry_msgs/PoseStamped.h>
// #include <std_msgs/UInt32.h>
// #include <sensor_msgs/JointState.h>

// #include <sensor_msgs/Range.h>
// #include "tf/tf.h"
// #include "tf/transform_broadcaster.h"
// #include <std_msgs/UInt8.h>

static const char EMPTY_STRING[] = "";

static const char WELLCOME_STR[] =
    "\n\n"
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
static int parseColorStr(const char *color_str, Color_t *color_ptr) {
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

static void button1Callback() {
    button1_publish_flag = true;
}

static void button2Callback() {
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

static uint32_t spin_count = 1;

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

void range_sensors_msg_handler() {
    // osEvent evt1 = distance_sensor_mail_box.get(0);
    // if (evt1.status == osEventMail) {
    //     SensorsMeasurement *message = (SensorsMeasurement *)evt1.value.p;
    //     if (message->status == MultiDistanceSensor::ERR_I2C_FAILURE) {
    //         err_msg++;
    //         if (distance_sensor_commands.empty() && err_msg == 3) {
    //             uint8_t *data = distance_sensor_commands.alloc();
    //             *data = 2;
    //             distance_sensor_commands.put(data);
    //             data = distance_sensor_commands.alloc();
    //             *data = 1;
    //             distance_sensor_commands.put(data);
    //             err_msg = 0;
    //         }
    //     } else {
    //         err_msg = 0;
    //         for (int i = 0; i < 4; i++) {
    // range_msg[i].header.stamp = nh.now(message->timestamp);
    // range_msg[i].range = message->range[i];
    // if (nh.connected())
    //     range_pub[i]->publish(&range_msg[i]);
    //         }
    //     }
    //     distance_sensor_mail_box.free(message);
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
}

void imu_msg_handler() {
    osEvent evt2 = imu_sensor_mail_box.get(0);
    if (evt2.status == osEventMail) {
        ImuDriver::ImuMesurement *message = (ImuDriver::ImuMesurement *)evt2.value.p;
        led2 = !led2;

        if (rmw_uros_epoch_synchronized()) {
            imu_msg.header.stamp.sec = (int32_t)(rmw_uros_epoch_millis() / 1000);
            imu_msg.header.stamp.nanosec = (uint32_t)(rmw_uros_epoch_nanos() % 1000000000);
        }

        imu_msg.orientation.x = message->orientation[0];
        imu_msg.orientation.y = message->orientation[1];
        imu_msg.orientation.z = message->orientation[2];
        imu_msg.orientation.w = message->orientation[3];

        imu_msg.angular_velocity.x = message->angular_velocity[0];
        imu_msg.angular_velocity.y = message->angular_velocity[1];
        imu_msg.angular_velocity.z = message->angular_velocity[2];

        imu_msg.linear_acceleration.x = message->linear_acceleration[0];
        imu_msg.linear_acceleration.y = message->linear_acceleration[1];
        imu_msg.linear_acceleration.z = message->linear_acceleration[2];

        RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
        imu_sensor_mail_box.free(message);
    }
}

void battery_msg_handler() {
    if (spin_count % 40 == 0) {
        // if (nh.connected())
        //     battery_pub->publish(&battery_state);
    }
}

void buttons_msgs_handler() {
    if (button1_publish_flag) {
        button1_publish_flag = false;
        if (!button1) {
            // button_msg.data = 1;
            // if (nh.connected())
            //     button_pub->publish(&button_msg);
        }
    }

    if (button2_publish_flag) {
        button2_publish_flag = false;
        if (!button2) {
            // button_msg.data = 2;
            // if (nh.connected())
            //     button_pub->publish(&button_msg);
        }
    }
}

void wheels_state_msg_handler() {
    if (spin_count % 5 == 0) {
        if (rmw_uros_epoch_synchronized()) {
            wheels_state_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
            wheels_state_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
        }
        wheels_state_msg.position.data[motor_left_front] = odometry.wheel_FL_ang_pos;
        wheels_state_msg.position.data[motor_right_front] = odometry.wheel_FR_ang_pos;
        wheels_state_msg.position.data[motor_left_rear] = odometry.wheel_RL_ang_pos;
        wheels_state_msg.position.data[motor_right_rear] = odometry.wheel_RR_ang_pos;

        wheels_state_msg.velocity.data[motor_left_front] = odometry.wheel_FL_ang_vel;
        wheels_state_msg.velocity.data[motor_right_front] = odometry.wheel_FR_ang_vel;
        wheels_state_msg.velocity.data[motor_left_rear] = odometry.wheel_RL_ang_vel;
        wheels_state_msg.velocity.data[motor_right_rear] = odometry.wheel_RR_ang_vel;
        RCSOFTCHECK(rcl_publish(&wheels_state_pub, &wheels_state_msg, NULL));
    }
}


void microros_spin() {
    while (true) {
        rclc_executor_spin(&executor);
    }
}

void read_and_show_battery_state() {
    // battery_state.voltage = rosbot_sensors::updateBatteryWatchdog();
}

void check_speed_watchdog() {
    if (is_speed_watchdog_enabled) {
        if (!is_speed_watchdog_active && (odom_watchdog_timer.read_ms() - last_speed_command_time) > speed_watchdog_interval) {
            RosbotSpeed speed = {0.0, 0.0, 0.0};
            RosbotDrive &drive = RosbotDrive::getInstance();
            rk->setRosbotSpeed(drive, speed);
            is_speed_watchdog_active = true;
        }
    }
}

static float curr_odom_calc_time = 0.0;
static float last_odom_calc_time = 0.0;

void update_odometry() {
    if (spin_count % 2 == 0) {
        curr_odom_calc_time = odom_watchdog_timer.read();
        RosbotDrive &drive = RosbotDrive::getInstance();
        rk->updateRosbotOdometry(drive, odometry, curr_odom_calc_time - last_odom_calc_time);
        last_odom_calc_time = curr_odom_calc_time;
    }
}

int main() {
    int spin_result;
    int err_msg = 0;

    ThisThread::sleep_for(100);
    sens_power = 1;  // sensors power on
    ThisThread::sleep_for(100);
    odom_watchdog_timer.start();

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

    set_microros_serial_transports(&microros_serial);
    microros_init();

#if USE_WS2812B_ANIMATION_MANAGER
    anim_manager = AnimationManager::getInstance();
    anim_manager->init();
#endif

#if defined(MEMORY_DEBUG_INFO)
    print_debug_info();
#endif /* MEMORY_DEBUG_INFO */

    if (imu_init_flag) {
        imu_driver_ptr->start();
    }

    if (distance_sensors_init_flag) {
        uint8_t *data = distance_sensor_commands.alloc();
        *data = 1;
        distance_sensor_commands.put(data);
        distance_sensors_enabled = true;
    }

    Thread microros_thread;

    microros_thread.start(microros_spin);

    while (1) {
        check_speed_watchdog();
        read_and_show_battery_state();

        // Restart uC when loose microros agent connection
        if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
            microros_deinit();
            NVIC_SystemReset();
        }

        update_odometry();

        wheels_state_msg_handler();
        imu_msg_handler();
        buttons_msgs_handler();
        battery_msg_handler();
        spin_count++;
        ThisThread::sleep_for(10);
    }
}