#include "rosbot_kinematics.h"

namespace rosbot_kinematics
{

    RosbotWheel custom_wheel_params{};

    RosbotKinematics::RosbotKinematics()
    {
    }

    RosbotKinematics::~RosbotKinematics()
    {
    }

    void RosbotKinematics::setOdomParams()
    {
        custom_wheel_params = {
            .radius = this->WHEEL_RADIUS,
            .diameter_modificator = this->DIAMETER_MODIFICATOR,
            .tyre_deflation = this->TYRE_DEFLATION,
            .gear_ratio = this->GEAR_RATIO,
            .encoder_cpr = this->ENCODER_CPR,
            .polarity = this->POLARITY};
    }

    void RosbotKinematics::resetRosbotOdometry(RosbotDrive &drive, RosbotOdometry &odom)
    {
        drive.enablePidReg(0);
        drive.resetDistance();
        odom.reset();
        drive.enablePidReg(1);
    }

    void RosbotKinematics::calibrateOdometry(float diameter_modificator, float tyre_deflation)
    {
        custom_wheel_params.diameter_modificator = diameter_modificator;
        custom_wheel_params.tyre_deflation = tyre_deflation;
    }

    RosbotKinematics *RosbotKinematics::kinematicsType(int type)
    {
        if (type == 0)
            return new DifferentialDrive;
        else if (type == 1)
            return new MecanumDrive;
        else
            return new DifferentialDrive;
    }

    geometry_msgs::Twist RosbotKinematics::getTwist(RosbotOdometry &odom)
    {
        geometry_msgs::Twist twist{};
        twist.linear.x = odom.robot_x_vel;
        twist.linear.y = odom.robot_y_vel;
        twist.angular.z = odom.robot_angular_vel;
        return twist;
    }

    //// diff drive -----------

    DifferentialDrive::DifferentialDrive()
    {
    }

    DifferentialDrive::~DifferentialDrive()
    {
    }

    void DifferentialDrive::updateRosbotOdometry(RosbotDrive &drive, RosbotOdometry &odom, float dtime)
    {
        float curr_wheel_R_ang_pos;
        float curr_wheel_L_ang_pos;
        odom.wheel_FR_ang_pos = drive.getAngularPos(MOTOR_FR);
        odom.wheel_FL_ang_pos = drive.getAngularPos(MOTOR_FL);
        odom.wheel_RR_ang_pos = drive.getAngularPos(MOTOR_RR);
        odom.wheel_RL_ang_pos = drive.getAngularPos(MOTOR_RL);
        curr_wheel_R_ang_pos = (odom.wheel_FR_ang_pos + odom.wheel_RR_ang_pos) / (2 * custom_wheel_params.tyre_deflation);
        curr_wheel_L_ang_pos = (odom.wheel_FL_ang_pos + odom.wheel_RL_ang_pos) / (2 * custom_wheel_params.tyre_deflation);
        odom.wheel_L_ang_vel = (curr_wheel_L_ang_pos - odom.wheel_L_ang_pos) / (dtime);
        odom.wheel_R_ang_vel = (curr_wheel_R_ang_pos - odom.wheel_R_ang_pos) / (dtime);
        odom.wheel_L_ang_pos = curr_wheel_L_ang_pos;
        odom.wheel_R_ang_pos = curr_wheel_R_ang_pos;
        odom.robot_angular_vel = (((odom.wheel_R_ang_pos - odom.wheel_L_ang_pos) * WHEEL_RADIUS / (ROBOT_WIDTH * custom_wheel_params.diameter_modificator)) - odom.robot_angular_pos) / dtime;
        odom.robot_angular_pos = (odom.wheel_R_ang_pos - odom.wheel_L_ang_pos) * WHEEL_RADIUS / (ROBOT_WIDTH * custom_wheel_params.diameter_modificator);
        float vel_x = (odom.wheel_L_ang_vel * WHEEL_RADIUS + odom.robot_angular_vel * ROBOT_WIDTH_HALF) * cos(odom.robot_angular_pos);
        float vel_y = (odom.wheel_L_ang_vel * WHEEL_RADIUS + odom.robot_angular_vel * ROBOT_WIDTH_HALF) * sin(odom.robot_angular_pos);
        odom.robot_x_vel = sqrt(vel_x * vel_x + vel_y * vel_y);
        odom.robot_y_vel = 0.0;
        odom.robot_x_pos = odom.robot_x_pos + odom.robot_x_vel * dtime;
        odom.robot_y_pos = odom.robot_y_pos + vel_y * dtime;
    }

    void DifferentialDrive::setRosbotSpeed(RosbotDrive &drive, RosbotSpeed &speed)
    {
        NewTargetSpeed new_speed;
        new_speed.mode = MPS;
        new_speed.speed[MOTOR_FL] = new_speed.speed[MOTOR_RL] = speed.lin_x - (speed.ang_z * ROBOT_WIDTH_HALF);
        new_speed.speed[MOTOR_FR] = new_speed.speed[MOTOR_RR] = speed.lin_x + (speed.ang_z * ROBOT_WIDTH_HALF);
        drive.updateTargetSpeed(new_speed);
    }

    void DifferentialDrive::setRosbotSpeed(RosbotDrive &drive, float linear_x, float angular_z)
    {
        NewTargetSpeed new_speed;
        new_speed.mode = MPS;
        new_speed.speed[MOTOR_FL] = new_speed.speed[MOTOR_RL] = linear_x - (angular_z * ROBOT_WIDTH_HALF);
        new_speed.speed[MOTOR_FR] = new_speed.speed[MOTOR_RR] = linear_x + (angular_z * ROBOT_WIDTH_HALF);
        drive.updateTargetSpeed(new_speed);
    }

    //// mecanum -----------

    MecanumDrive::MecanumDrive()
    {
    }

    MecanumDrive::~MecanumDrive()
    {
    }

    void MecanumDrive::setRosbotSpeed(RosbotDrive &drive, RosbotSpeed &speed)
    {
        NewTargetSpeed new_speed;
        new_speed.mode = MPS;
        new_speed.speed[MOTOR_FR] = (speed.lin_x + speed.lin_y + (ROBOT_WIDTH_HALF + WHEEL_SEPARATION_LENGTH) * speed.ang_z); //  # m/s
        new_speed.speed[MOTOR_FL] = (speed.lin_x - speed.lin_y - (ROBOT_WIDTH_HALF + WHEEL_SEPARATION_LENGTH) * speed.ang_z);
        new_speed.speed[MOTOR_RR] = (speed.lin_x - speed.lin_y + (ROBOT_WIDTH_HALF + WHEEL_SEPARATION_LENGTH) * speed.ang_z);
        new_speed.speed[MOTOR_RL] = (speed.lin_x + speed.lin_y - (ROBOT_WIDTH_HALF + WHEEL_SEPARATION_LENGTH) * speed.ang_z);
        drive.updateTargetSpeed(new_speed);
    }

    void MecanumDrive::updateRosbotOdometry(RosbotDrive &drive, RosbotOdometry &odom, float dtime)
    {
        float curr_wheel_FR_ang_pos;
        float curr_wheel_FL_ang_pos;
        float curr_wheel_RR_ang_pos;
        float curr_wheel_RL_ang_pos;

        curr_wheel_FR_ang_pos = drive.getAngularPos(MOTOR_FR);
        curr_wheel_FL_ang_pos = drive.getAngularPos(MOTOR_FL);
        curr_wheel_RR_ang_pos = drive.getAngularPos(MOTOR_RR);
        curr_wheel_RL_ang_pos = drive.getAngularPos(MOTOR_RL);

        odom.wheel_FR_ang_vel = (curr_wheel_FR_ang_pos - odom.wheel_FR_ang_pos) / (dtime);
        odom.wheel_FL_ang_vel = (curr_wheel_FL_ang_pos - odom.wheel_FL_ang_pos) / (dtime);
        odom.wheel_RR_ang_vel = (curr_wheel_RR_ang_pos - odom.wheel_RR_ang_pos) / (dtime);
        odom.wheel_RL_ang_vel = (curr_wheel_RL_ang_pos - odom.wheel_RL_ang_pos) / (dtime);

        odom.robot_x_vel = (odom.wheel_FR_ang_vel + odom.wheel_FL_ang_vel + odom.wheel_RR_ang_vel + odom.wheel_RL_ang_vel) * (WHEEL_RADIUS / 4);
        odom.robot_y_vel = (-odom.wheel_FL_ang_vel + odom.wheel_FR_ang_vel + odom.wheel_RL_ang_vel - odom.wheel_RR_ang_vel) * (WHEEL_RADIUS / 4);
        odom.robot_angular_vel = (-odom.wheel_FL_ang_vel + odom.wheel_FR_ang_vel - odom.wheel_RL_ang_vel + odom.wheel_RR_ang_vel) * (WHEEL_RADIUS / (4 * (ROBOT_WIDTH_HALF + WHEEL_SEPARATION_LENGTH)));

        double delta_heading = odom.robot_angular_vel / dtime; // [radians/s^2]
        odom.robot_angular_pos = odom.robot_angular_pos + delta_heading * dtime * dtime * 2;
        double delta_x = (odom.robot_x_vel * cos(odom.robot_angular_pos) - odom.robot_y_vel * sin(odom.robot_angular_pos)) / dtime; // [m]
        double delta_y = (odom.robot_x_vel * sin(odom.robot_angular_pos) + odom.robot_y_vel * cos(odom.robot_angular_pos)) / dtime; // [m]
        odom.robot_x_pos = odom.robot_x_pos + delta_x * dtime * dtime;
        odom.robot_y_pos = odom.robot_y_pos + delta_y * dtime * dtime;

        odom.wheel_FR_ang_pos = curr_wheel_FR_ang_pos;
        odom.wheel_FL_ang_pos = curr_wheel_FL_ang_pos;
        odom.wheel_RR_ang_pos = curr_wheel_RR_ang_pos;
        odom.wheel_RL_ang_pos = curr_wheel_RL_ang_pos;
    }

} // namespace rosbot_kinematics