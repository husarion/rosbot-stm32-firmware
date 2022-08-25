/** @file main.cpp
 * ROSbot firmware.
 *
 * @author Husarion
 * @copyright MIT
 */
#include <rosbot_kinematics.h>
#include <rosbot_sensors.h>
#include <ImuDriver.h>
#include <main.hpp>
// #include <ros.h>
// #include <sensor_msgs/JointState.h>
#include <geometry_msgs/msg/twist.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <std_msgs/UInt32.h>
// #include <sensor_msgs/Imu.h>
// #include <rosbot_ekf/Imu.h>


// #include <sensor_msgs/Range.h>
// #include "tf/tf.h"
// #include "tf/transform_broadcaster.h"
// #include <std_msgs/UInt8.h>

static const char EMPTY_STRING[] = "";

static const char WELLCOME_STR[] = "\n\n"
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
static int parseColorStr(const char *color_str, Color_t *color_ptr)
{
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

geometry_msgs__msg__Twist current_vel;
// sensor_msgs::JointState joint_states;
// sensor_msgs::BatteryState battery_state;
// sensor_msgs::Range range_msg[4];
// geometry_msgs::PoseStamped pose;
// std_msgs::UInt8 button_msg;
// rosbot_ekf::Imu imu_msg;
// sensor_msgs::Imu imu_msg;
// ros::NodeHandle nh;
// ros::Publisher *vel_pub;
// ros::Publisher *joint_state_pub;
// ros::Publisher *battery_pub;
// ros::Publisher *range_pub[4];
// ros::Publisher *pose_pub;
// ros::Publisher *button_pub;
// ros::Publisher *imu_pub;
// geometry_msgs::TransformStamped robot_tf;
// tf::TransformBroadcaster broadcaster;



static void button1Callback()
{
    button1_publish_flag = true;
}

static void button2Callback()
{
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

// static void initImuPublisher()
// {
//     imu_pub = new ros::Publisher("imu", &imu_msg);

//     imu_msg.header.frame_id = "imu_link";

//     imu_msg.orientation_covariance[0] = 0.05;
//     imu_msg.orientation_covariance[4] = 0.05;
//     imu_msg.orientation_covariance[8] = 0.05;

//     imu_msg.angular_velocity_covariance[0] = 0.1;
//     imu_msg.angular_velocity_covariance[4] = 0.1;
//     imu_msg.angular_velocity_covariance[8] = 0.1;

//     imu_msg.linear_acceleration_covariance[0] = 0.5;
//     imu_msg.linear_acceleration_covariance[4] = 0.5;
//     imu_msg.linear_acceleration_covariance[8] = 0.5;

//     nh.advertise(*imu_pub);
// }

// static void initButtonPublisher()
// {
//     button_pub = new ros::Publisher("buttons", &button_msg);
//     nh.advertise(*button_pub);
// }

// static void initRangePublisher()
// {
//     for (int i = 0; i < 4; i++)
//     {
//         range_msg[i].field_of_view = 0.26;
//         range_msg[i].min_range = 0.03;
//         range_msg[i].max_range = 0.90;
//         range_msg[i].header.frame_id = range_id[i];
//         range_msg[i].radiation_type = sensor_msgs::Range::INFRARED;
//         range_pub[i] = new ros::Publisher(range_pub_names[i], &range_msg[i]);
//         nh.advertise(*range_pub[i]);
//     }
// }

// static void initBatteryPublisher()
// {
//     battery_state.power_supply_status = battery_state.POWER_SUPPLY_STATUS_UNKNOWN;
//     battery_state.power_supply_health = battery_state.POWER_SUPPLY_HEALTH_UNKNOWN;
//     battery_state.power_supply_technology = battery_state.POWER_SUPPLY_TECHNOLOGY_LION;
//     battery_pub = new ros::Publisher("battery", &battery_state);
//     nh.advertise(*battery_pub);
// }

// static void initPosePublisher()
// {
//     pose.header.frame_id = "odom";
//     pose.pose.position.x = 0;
//     pose.pose.position.y = 0;
//     pose.pose.position.z = 0;
//     pose.pose.orientation.x = 0;
//     pose.pose.orientation.y = 0;
//     pose.pose.orientation.z = 0;
//     pose.pose.orientation.w = 1;
//     pose_pub = new ros::Publisher("pose", &pose);
//     nh.advertise(*pose_pub);
// }

// static void initTfPublisher()
// {
//     robot_tf.header.frame_id = "odom";
//     robot_tf.child_frame_id = "base_link";
//     robot_tf.transform.translation.x = 0.0;
//     robot_tf.transform.translation.y = 0.0;
//     robot_tf.transform.translation.z = 0.0;
//     robot_tf.transform.rotation.x = 0.0;
//     robot_tf.transform.rotation.y = 0.0;
//     robot_tf.transform.rotation.z = 0.0;
//     robot_tf.transform.rotation.w = 1.0;
//     broadcaster.init(nh);
// }

// static void initVelocityPublisher()
// {
//     current_vel.linear.x = 0;
//     current_vel.linear.y = 0;
//     current_vel.linear.z = 0;
//     current_vel.angular.x = 0;
//     current_vel.angular.y = 0;
//     current_vel.angular.z = 0;
//     vel_pub = new ros::Publisher("velocity", &current_vel);
//     nh.advertise(*vel_pub);
// }

// static void initJointStatePublisher()
// {
//     joint_state_pub = new ros::Publisher("joint_states", &joint_states);
//     nh.advertise(*joint_state_pub);

//     joint_states.header.frame_id = "base_link";

//     // assigning the arrays to the message
//     joint_states.name = (char **)joint_state_name;
//     joint_states.position = pos;
//     // joint_states.velocity = vel;
//     // joint_states.effort = eff;

//     // setting the length
//     joint_states.name_length = 4;
//     joint_states.position_length = 4;
//     // joint_states.velocity_length = 4;
//     // joint_states.effort_length = 4;
// }

// static void velocityCallback(const geometry_msgs::Twist &twist_msg)
// {
//     RosbotDrive &drive = RosbotDrive::getInstance();
//     RosbotSpeed speed = {static_cast<float>(twist_msg.linear.x), static_cast<float>(twist_msg.linear.y), static_cast<float>(twist_msg.angular.z)};
//     rk->setRosbotSpeed(drive, speed);
//     last_speed_command_time = odom_watchdog_timer.read_ms();
//     is_speed_watchdog_active = false;
// }

// static void servoCallback(const std_msgs::UInt32 &ser_msg)
// {
//     int servo_num = ser_msg.data & 0xF;
//     int servo_width = ser_msg.data >> 4;
//     servo_manager.setWidth(servo_num - 1, servo_width);
// }




#if defined(MEMORY_DEBUG_INFO)
#define MAX_THREAD_INFO 10

mbed_stats_heap_t heap_info;
mbed_stats_stack_t stack_info[MAX_THREAD_INFO];

int print_debug_info()
{
    debug("\nThis message is from debug function");
    debug_if(1, "\nThis message is from debug_if function");
    debug_if(0, "\nSOMETHING WRONG!!! This message from debug_if function shouldn't show on bash");

    printf("\nMemoryStats:");
    mbed_stats_heap_get(&heap_info);
    printf("\n\tBytes allocated currently: %d", heap_info.current_size);
    printf("\n\tMax bytes allocated at a given time: %d", heap_info.max_size);
    printf("\n\tCumulative sum of bytes ever allocated: %d", heap_info.total_size);
    printf("\n\tCurrent number of bytes allocated for the heap: %d", heap_info.reserved_size);
    printf("\n\tCurrent number of allocations: %d", heap_info.alloc_cnt);
    printf("\n\tNumber of failed allocations: %d", heap_info.alloc_fail_cnt);

    mbed_stats_stack_get(&stack_info[0]);
    printf("\nCumulative Stack Info:");
    printf("\n\tMaximum number of bytes used on the stack: %d", stack_info[0].max_size);
    printf("\n\tCurrent number of bytes allocated for the stack: %d", stack_info[0].reserved_size);
    printf("\n\tNumber of stacks stats accumulated in the structure: %d", stack_info[0].stack_cnt);

    mbed_stats_stack_get_each(stack_info, MAX_THREAD_INFO);
    printf("\nThread Stack Info:");
    for (int i = 0; i < MAX_THREAD_INFO; i++)
    {
        if (stack_info[i].thread_id != 0)
        {
            printf("\n\tThread: %d", i);
            printf("\n\t\tThread Id: 0x%08X", stack_info[i].thread_id);
            printf("\n\t\tMaximum number of bytes used on the stack: %d", stack_info[i].max_size);
            printf("\n\t\tCurrent number of bytes allocated for the stack: %d", stack_info[i].reserved_size);
            printf("\n\t\tNumber of stacks stats accumulated in the structure: %d", stack_info[i].stack_cnt);
        }
    }

    printf("\nDone...\n\n");
}
#endif /* MEMORY_DEBUG_INFO */

int main()
{
    int spin_result;
    int err_msg = 0;
    uint32_t spin_count = 1;
    float curr_odom_calc_time, last_odom_calc_time = 0.0f;

    ThisThread::sleep_for(100);
    sens_power = 1; // sensors power on
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

    // nh.initNode();

    bool distance_sensors_init_flag = false;
    bool imu_init_flag = false;
    bool welcome_flag = true;

    // TODO: add /diagnostic messages
    int num_sens_init;
    if ((num_sens_init = distance_sensors.init()) > 0)
    {
        distance_sensors_init_flag = true;
    }

    I2C *i2c_ptr = new I2C(IMU_I2C_SDA, IMU_I2C_SCL);
    i2c_ptr->frequency(IMU_I2C_FREQUENCY);

    ImuDriver::Type type = ImuDriver::getType(i2c_ptr, 2);
    sprintf(imu_description_string, "Detected sensor: %s\r\n", imu_sensor_type_string[type]);

    if (type != ImuDriver::UNKNOWN)
    {
        imu_driver_ptr = new ImuDriver(i2c_ptr, type);
        imu_driver_ptr->init();
        imu_driver_ptr->start();
        imu_init_flag = true;
    }

    // ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &velocityCallback);
    // ros::Subscriber<std_msgs::UInt32> cmd_ser_sub("cmd_ser", &servoCallback);
    // ros::ServiceServer<rosbot_ekf::Configuration::Request, rosbot_ekf::Configuration::Response> config_srv("config", responseCallback);

    // nh.advertiseService(config_srv);
    // nh.subscribe(cmd_vel_sub);
    // nh.subscribe(cmd_ser_sub);

    // initBatteryPublisher();
    // initPosePublisher();
    // initVelocityPublisher();
    // initRangePublisher();
    // initJointStatePublisher();
    // initImuPublisher();
    // initButtonPublisher();

#if USE_WS2812B_ANIMATION_MANAGER
    anim_manager = AnimationManager::getInstance();
    anim_manager->init();
#endif

#if defined(MEMORY_DEBUG_INFO)
    print_debug_info();
#endif /* MEMORY_DEBUG_INFO */

    if (imu_init_flag)
        imu_driver_ptr->start();

    if (distance_sensors_init_flag)
    {
        uint8_t *data = distance_sensor_commands.alloc();
        *data = 1;
        distance_sensor_commands.put(data);
        distance_sensors_enabled = true;
    }

    while (1)
    {
        if (is_speed_watchdog_enabled)
        {
            if (!is_speed_watchdog_active && (odom_watchdog_timer.read_ms() - last_speed_command_time) > speed_watchdog_interval)
            {
                RosbotSpeed speed = {0.0, 0.0, 0.0};
                RosbotDrive &drive = RosbotDrive::getInstance();
                rk->setRosbotSpeed(drive, speed);
                is_speed_watchdog_active = true;
            }
        }

#if USE_WS2812B_ANIMATION_MANAGER
        // if (!nh.connected())
        //     anim_manager->enableInterface(false);
#endif

        if (spin_count % 2 == 0)
        {
            curr_odom_calc_time = odom_watchdog_timer.read();
            RosbotDrive &drive = RosbotDrive::getInstance();
            rk->updateRosbotOdometry(drive, odometry, curr_odom_calc_time - last_odom_calc_time);
            last_odom_calc_time = curr_odom_calc_time;
        }

        if (button1_publish_flag)
        {
            button1_publish_flag = false;
            if (!button1)
            {
                // button_msg.data = 1;
                // if (nh.connected())
                //     button_pub->publish(&button_msg);
            }
        }

        if (button2_publish_flag)
        {
            button2_publish_flag = false;
            if (!button2)
            {
                // button_msg.data = 2;
                // if (nh.connected())
                //     button_pub->publish(&button_msg);
            }
        }

        if (spin_count % 5 == 0) /// cmd_vel, odometry, joint_states, tf messages
        {
            current_vel.linear.x = sqrt(odometry.robot_x_vel * odometry.robot_x_vel + odometry.robot_y_vel * odometry.robot_y_vel);
            current_vel.angular.z = odometry.robot_angular_vel;
            current_vel = rk->getTwist(odometry);
            // pose.pose.position.x = odometry.robot_x_pos;
            // pose.pose.position.y = odometry.robot_y_pos;
            // pose.pose.orientation = tf::createQuaternionFromYaw(odometry.robot_angular_pos);

            // pose.header.stamp = nh.now();
            // if (nh.connected())
            // {
            //     pose_pub->publish(&pose);
            //     vel_pub->publish(&current_vel);
            // }

            // if (joint_states_enabled)
            // {
            //     pos[0] = odometry.wheel_FL_ang_pos;
            //     pos[1] = odometry.wheel_FR_ang_pos;
            //     pos[2] = odometry.wheel_RL_ang_pos;
            //     pos[3] = odometry.wheel_RR_ang_pos;
            //     joint_states.position = pos;
            //     joint_states.header.stamp = pose.header.stamp;
                // if (nh.connected())
                //     joint_state_pub->publish(&joint_states);
            // }

            // if (tf_msgs_enabled)
            // {
            //     robot_tf.header.stamp = pose.header.stamp;
            //     robot_tf.transform.translation.x = pose.pose.position.x;
            //     robot_tf.transform.translation.y = pose.pose.position.y;
            //     robot_tf.transform.rotation.x = pose.pose.orientation.x;
            //     robot_tf.transform.rotation.y = pose.pose.orientation.y;
            //     robot_tf.transform.rotation.z = pose.pose.orientation.z;
            //     robot_tf.transform.rotation.w = pose.pose.orientation.w;
                // if (nh.connected())
                //     broadcaster.sendTransform(robot_tf);
            // }
        }

        if (spin_count % 40 == 0)
        {
            // battery_state.voltage = rosbot_sensors::updateBatteryWatchdog();
            // if (nh.connected())
            //     battery_pub->publish(&battery_state);
        }

        osEvent evt1 = distance_sensor_mail_box.get(0);
        if (evt1.status == osEventMail)
        {
            SensorsMeasurement *message = (SensorsMeasurement *)evt1.value.p;
            if (message->status == MultiDistanceSensor::ERR_I2C_FAILURE)
            {
                err_msg++;
                if (distance_sensor_commands.empty() && err_msg == 3)
                {
                    // if (nh.connected())
                    //     nh.logerror("I2C error. Restarting VL53L0X sensors...");
                    uint8_t *data = distance_sensor_commands.alloc();
                    *data = 2;
                    distance_sensor_commands.put(data);
                    data = distance_sensor_commands.alloc();
                    *data = 1;
                    distance_sensor_commands.put(data);
                    err_msg = 0;
                }
            }
            else
            {
                err_msg = 0;
                for (int i = 0; i < 4; i++)
                {
                    // range_msg[i].header.stamp = nh.now(message->timestamp);
                    // range_msg[i].range = message->range[i];
                    // if (nh.connected())
                    //     range_pub[i]->publish(&range_msg[i]);
                }
            }
            distance_sensor_mail_box.free(message);
        }
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

        osEvent evt2 = imu_sensor_mail_box.get(0);

        if (evt2.status == osEventMail)
        {
            ImuDriver::ImuMesurement *message = (ImuDriver::ImuMesurement *)evt2.value.p;
            led2 = !led2;
            // if (nh.connected())
            // {
            //     imu_msg.header.stamp = nh.now(message->timestamp);

            //     imu_msg.orientation.x = message->orientation[0];
            //     imu_msg.orientation.y = message->orientation[1];
            //     imu_msg.orientation.z = message->orientation[2];
            //     imu_msg.orientation.w = message->orientation[3];

            //     imu_msg.angular_velocity.x = message->angular_velocity[0];
            //     imu_msg.angular_velocity.y = message->angular_velocity[1];
            //     imu_msg.angular_velocity.z = message->angular_velocity[2];

            //     imu_msg.linear_acceleration.x = message->linear_acceleration[0];
            //     imu_msg.linear_acceleration.y = message->linear_acceleration[1];
            //     imu_msg.linear_acceleration.z = message->linear_acceleration[2];

            //     imu_pub->publish(&imu_msg);
            // }

            imu_sensor_mail_box.free(message);
        }

        // LOGS
        // if (nh.connected())
        // {
        //     if (welcome_flag)
        //     {
        //         welcome_flag = false;
        //         nh.loginfo(WELLCOME_STR);
        //         if (!distance_sensors_init_flag)
        //             nh.logerror("VL53L0X sensors initialisation failure!");
        //         if (!imu_init_flag)
        //             nh.logerror("No IMU sensor detected!");
        //         else
        //             nh.loginfo(imu_description_string);
        //     }
        // }
        // else
        // {
        //     welcome_flag = true;
        // }

        // nh.spinOnce();

        // if((spin_result=nh.spinOnce()) != ros::SPIN_OK)
        // {
        //     // nh.logwarn(spin_result == -1 ? "SPIN_ERR" : "SPIN_TIMEOUT");
        //     do {}while(0); // do nothing at the moment
        // }
        spin_count++;
        ThisThread::sleep_for(MAIN_LOOP_INTERVAL_MS);
    }
}