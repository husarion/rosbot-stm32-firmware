#ifndef __ROSBOT_KINEMATICS_H__
#define __ROSBOT_KINEMATICS_H__

#include <RosbotDrive.h>
#include <geometry_msgs/Twist.h>

#define KINEMATICS_TYPE 1 // 0 diff drive, 1 mecanum

#define MOTOR_FR MOTOR1
#define MOTOR_FL MOTOR4
#define MOTOR_RR MOTOR2
#define MOTOR_RL MOTOR3

namespace rosbot_kinematics
{
    extern RosbotWheel custom_wheel_params;
    struct Odometry
    {
        float wheel_FR_ang_pos;  // radians
        float wheel_FL_ang_pos;  // radians
        float wheel_RR_ang_pos;  // radians
        float wheel_RL_ang_pos;  // radians
        float wheel_L_ang_pos;   // radians
        float wheel_R_ang_pos;   // radians
        float wheel_L_ang_vel;   // radians per second - for differential drive
        float wheel_R_ang_vel;   // radians per second - for differential drive
        float wheel_FR_ang_vel;  // radians per second - for mecanum
        float wheel_FL_ang_vel;  // radians per second - for mecanum
        float wheel_RR_ang_vel;  // radians per second - for mecanum
        float wheel_RL_ang_vel;  // radians per second - for mecanum
        float robot_angular_pos; // radians
        float robot_angular_vel; // radians per second
        float robot_x_pos;       // meters
        float robot_y_pos;       // meters
        float robot_x_vel;       // meters per second
        float robot_y_vel;       // meters per second
    };
    union RosbotOdometry
    {
        Odometry odom;
        float buffor[14];
    };

    class RosbotKinematics
    {
    protected:
        const uint8_t POLARITY = 0b00111100;
        // const uint8_t POLARITY =0b11000011; // red wheels
        const float ROBOT_LENGTH = 0.197;
        const uint8_t ENCODER_CPR = 48;
        const float ROBOT_LENGTH_HALF = ROBOT_LENGTH / 2.0;
        const float DISTANCE_FRONT_TO_REAR_WHEEL = 0.11;
        const float WHEEL_SEPARATION_LENGTH = DISTANCE_FRONT_TO_REAR_WHEEL / 2;
        const float ROBOT_WIDTH = 0.206; // 0.22 0.195
        const float ROBOT_WIDTH_HALF = ROBOT_WIDTH / 2.0;
        const float DIAMETER_MODIFICATOR = 1.0; // 1.24, 1.09, 1.164
        const float TYRE_DEFLATION = 1.0;       // theoretical distance / real distance
        const float GEAR_RATIO = 34.014;
        const float WHEEL_DIAMETER = 0.097;
        const float WHEEL_RADIUS = WHEEL_DIAMETER / 2.0;

    public:
        RosbotKinematics();
        static RosbotKinematics *kinematicsType(int type);
        virtual ~RosbotKinematics();
        virtual void setRosbotSpeed(RosbotDrive &drive, RosbotSpeed &speed) = 0;
        virtual void updateRosbotOdometry(RosbotDrive &drive, RosbotOdometry &odom, float dtime) = 0;
        void resetRosbotOdometry(RosbotDrive &drive, RosbotOdometry &odom);
        virtual void calibrateOdometry(float diameter_modificator, float tyre_deflation);
        geometry_msgs::Twist getTwist(RosbotOdometry &odom);
        virtual void setOdomParams();
    };
    class DifferentialDrive : public RosbotKinematics
    {
    private:
        const float ROBOT_WIDTH = 0.215; // 0.22 0.195
        const float ROBOT_WIDTH_HALF = ROBOT_WIDTH / 2.0;
        const float DIAMETER_MODIFICATOR = 1.106; // 1.24, 1.09, 1.164
        const float TYRE_DEFLATION = 1.042;       // theoretical distance / real distance
        const float GEAR_RATIO = 34.014;
        // const float GEAR_RATIO = 20.4;    // red wheels
        const float WHEEL_DIAMETER = 0.085;
        const float WHEEL_RADIUS = WHEEL_DIAMETER / 2.0;

    public:
        DifferentialDrive(/* args */);
        ~DifferentialDrive();
        void setRosbotSpeed(RosbotDrive &drive, RosbotSpeed &speed);
        void setRosbotSpeed(RosbotDrive &drive, float linear_x, float angular_z);
        void updateRosbotOdometry(RosbotDrive &drive, RosbotOdometry &odom, float dtime);
    };

    class MecanumDrive : public RosbotKinematics
    {
    private:
        const float DISTANCE_FRONT_TO_REAR_WHEEL = 0.11;
        const float WHEEL_SEPARATION_LENGTH = DISTANCE_FRONT_TO_REAR_WHEEL / 2;
        const float ROBOT_WIDTH = 0.206; // 0.22 0.195
        const float ROBOT_WIDTH_HALF = ROBOT_WIDTH / 2.0;
        const float DIAMETER_MODIFICATOR = 1.0; // 1.24, 1.09, 1.164
        const float TYRE_DEFLATION = 1.0;       // theoretical distance / real distance
        const float GEAR_RATIO = 34.014;
        const float WHEEL_DIAMETER = 0.097;
        const float WHEEL_RADIUS = WHEEL_DIAMETER / 2.0;

    public:
        MecanumDrive(/* args */);
        ~MecanumDrive();
        void setRosbotSpeed(RosbotDrive &drive, RosbotSpeed &speed);
        void updateRosbotOdometry(RosbotDrive &drive, RosbotOdometry &odom, float dtime);
    };

} // namespace rosbot_kinematics
#endif /* __ROSBOT_KINEMATICS_H__ */