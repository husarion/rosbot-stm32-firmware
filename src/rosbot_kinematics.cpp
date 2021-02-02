#include "rosbot_kinematics.h"

namespace rosbot_kinematics
{

    RosbotWheel custom_wheel_params = {
        .radius = WHEEL_RADIUS,
        .diameter_modificator = DIAMETER_MODIFICATOR,
        .tyre_deflation = TYRE_DEFLATION,
        .gear_ratio = GEAR_RATIO,
        .encoder_cpr = ENCODER_CPR,
        .polarity = POLARITY};

    RosbotKinematics::RosbotKinematics()
    {
    }

    RosbotKinematics::~RosbotKinematics()
    {
    }

    void RosbotKinematics::resetRosbotOdometry(RosbotDrive &drive, RosbotOdometry &odom)
    {
        drive.enablePidReg(0);
        memset(odom.buffor, 0, sizeof(odom.buffor));
        drive.resetDistance();
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
        twist.linear.x = odom.odom.robot_x_vel;
        twist.linear.y = odom.odom.robot_y_vel;
        twist.angular.z = odom.odom.robot_angular_vel;
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
        Odometry *iodom = &odom.odom;
        iodom->wheel_FR_ang_pos = drive.getAngularPos(MOTOR_FR);
        iodom->wheel_FL_ang_pos = drive.getAngularPos(MOTOR_FL);
        iodom->wheel_RR_ang_pos = drive.getAngularPos(MOTOR_RR);
        iodom->wheel_RL_ang_pos = drive.getAngularPos(MOTOR_RL);
        curr_wheel_R_ang_pos = (iodom->wheel_FR_ang_pos + iodom->wheel_RR_ang_pos) / (2 * custom_wheel_params.tyre_deflation);
        curr_wheel_L_ang_pos = (iodom->wheel_FL_ang_pos + iodom->wheel_RL_ang_pos) / (2 * custom_wheel_params.tyre_deflation);
        iodom->wheel_L_ang_vel = (curr_wheel_L_ang_pos - iodom->wheel_L_ang_pos) / (dtime);
        iodom->wheel_R_ang_vel = (curr_wheel_R_ang_pos - iodom->wheel_R_ang_pos) / (dtime);
        iodom->wheel_L_ang_pos = curr_wheel_L_ang_pos;
        iodom->wheel_R_ang_pos = curr_wheel_R_ang_pos;
        iodom->robot_angular_vel = (((iodom->wheel_R_ang_pos - iodom->wheel_L_ang_pos) * WHEEL_RADIUS / (ROBOT_WIDTH * custom_wheel_params.diameter_modificator)) - iodom->robot_angular_pos) / dtime;
        iodom->robot_angular_pos = (iodom->wheel_R_ang_pos - iodom->wheel_L_ang_pos) * WHEEL_RADIUS / (ROBOT_WIDTH * custom_wheel_params.diameter_modificator);
        float vel_x = (iodom->wheel_L_ang_vel * WHEEL_RADIUS + iodom->robot_angular_vel * ROBOT_WIDTH_HALF) * cos(iodom->robot_angular_pos);
        float vel_y = (iodom->wheel_L_ang_vel * WHEEL_RADIUS + iodom->robot_angular_vel * ROBOT_WIDTH_HALF) * sin(iodom->robot_angular_pos);
        iodom->robot_x_vel = sqrt(vel_x * vel_x + vel_y * vel_y);
        iodom->robot_y_vel = 0.0;
        iodom->robot_x_pos = iodom->robot_x_pos + iodom->robot_x_vel * dtime;
        iodom->robot_y_pos = iodom->robot_y_pos + vel_y * dtime;
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
        new_speed.speed[MOTOR_FR] = (speed.lin_x + speed.lin_y + (ROBOT_WIDTH_HALF + WHEEL_SEPARATION_LENGTH) * 2 * speed.ang_z); //  # m/s
        new_speed.speed[MOTOR_FL] = (speed.lin_x - speed.lin_y - (ROBOT_WIDTH_HALF + WHEEL_SEPARATION_LENGTH) * 2 * speed.ang_z);
        new_speed.speed[MOTOR_RR] = (speed.lin_x - speed.lin_y + (ROBOT_WIDTH_HALF + WHEEL_SEPARATION_LENGTH) * 2 * speed.ang_z);
        new_speed.speed[MOTOR_RL] = (speed.lin_x + speed.lin_y - (ROBOT_WIDTH_HALF + WHEEL_SEPARATION_LENGTH) * 2 * speed.ang_z);
        drive.updateTargetSpeed(new_speed);
    }

    void MecanumDrive::updateRosbotOdometry(RosbotDrive &drive, RosbotOdometry &odom, float dtime)
    {
        float curr_wheel_FR_ang_pos;
        float curr_wheel_FL_ang_pos;
        float curr_wheel_RR_ang_pos;
        float curr_wheel_RL_ang_pos;
        Odometry *iodom = &odom.odom;

        curr_wheel_FR_ang_pos = drive.getAngularPos(MOTOR_FR);
        curr_wheel_FL_ang_pos = drive.getAngularPos(MOTOR_FL);
        curr_wheel_RR_ang_pos = drive.getAngularPos(MOTOR_RR);
        curr_wheel_RL_ang_pos = drive.getAngularPos(MOTOR_RL);

        iodom->wheel_FR_ang_vel = (curr_wheel_FR_ang_pos - iodom->wheel_FR_ang_pos) / (dtime);
        iodom->wheel_FL_ang_vel = (curr_wheel_FL_ang_pos - iodom->wheel_FL_ang_pos) / (dtime);
        iodom->wheel_RR_ang_vel = (curr_wheel_RR_ang_pos - iodom->wheel_RR_ang_pos) / (dtime);
        iodom->wheel_RL_ang_vel = (curr_wheel_RL_ang_pos - iodom->wheel_RL_ang_pos) / (dtime);

        iodom->robot_x_vel = (iodom->wheel_FR_ang_vel + iodom->wheel_FL_ang_vel + iodom->wheel_RR_ang_vel + iodom->wheel_RL_ang_vel) * (WHEEL_RADIUS / 4);
        iodom->robot_y_vel = (-iodom->wheel_FL_ang_vel + iodom->wheel_FR_ang_vel + iodom->wheel_RL_ang_vel - iodom->wheel_RR_ang_vel) * (WHEEL_RADIUS / 4);
        iodom->robot_angular_vel = (-iodom->wheel_FL_ang_vel + iodom->wheel_FR_ang_vel - iodom->wheel_RL_ang_vel + iodom->wheel_RR_ang_vel) * (WHEEL_RADIUS / (8 * (ROBOT_WIDTH_HALF + WHEEL_SEPARATION_LENGTH)));

        double delta_heading = iodom->robot_angular_vel / dtime; // [radians/s^2]
        iodom->robot_angular_pos = iodom->robot_angular_pos + delta_heading * dtime * dtime *2;
        double delta_x = (iodom->robot_x_vel * cos(iodom->robot_angular_pos) - iodom->robot_y_vel * sin(iodom->robot_angular_pos)) / dtime; // [m]
        double delta_y = (iodom->robot_x_vel * sin(iodom->robot_angular_pos) + iodom->robot_y_vel * cos(iodom->robot_angular_pos)) / dtime; // [m]
        iodom->robot_x_pos = iodom->robot_x_pos + delta_x * dtime * dtime ;
        iodom->robot_y_pos = iodom->robot_y_pos + delta_y * dtime * dtime ;

        iodom->wheel_FR_ang_pos = curr_wheel_FR_ang_pos;
        iodom->wheel_FL_ang_pos = curr_wheel_FL_ang_pos;
        iodom->wheel_RR_ang_pos = curr_wheel_RR_ang_pos;
        iodom->wheel_RL_ang_pos = curr_wheel_RL_ang_pos;
    }

} // namespace rosbot_kinematics