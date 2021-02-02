/** @file RosbotDrive.h
 * Definitions of RosbotDrive class and accompany structures.
 */
#ifndef __ROSBOT_DRIVE_H__
#define __ROSBOT_DRIVE_H__

#include <mbed.h>
#include <DRV8848_STM.h>
#include <Encoder.h>
#include <RosbotRegulator.h>

/**
 * @brief Rosbot Motor Internal Number.
 * 
 * MOTOR1 and MOTOR2 tags identify driver 1 motor 1 and 2 outputs. 
 * MOTOR3 and MOTOR4 tags identify driver 2 motor 1 and 2 outputs. 
 */
enum RosbotMotNum : uint8_t
{
    MOTOR1 = 0, ///< Driver 1 motor 1
    MOTOR2 = 1, ///< Driver 1 motor 2
    MOTOR3 = 2, ///< Driver 2 motor 1
    MOTOR4 = 3  ///< Driver 2 motor 2
};

enum SpeedMode
{
    TICSKPS,
    MPS,
    DUTY_CYCLE
};

enum RosbotDriveStates
{
    UNINIT,
    HALT,
    IDLE,
    OPERATIONAL,
    FAULT
};

struct RosbotWheel
{
    float radius;
    float diameter_modificator;
    float tyre_deflation;
    float gear_ratio;
    uint32_t encoder_cpr; // conuts per revolution
    uint8_t polarity;     // LSB -> motor, MSB -> encoder
};

struct NewTargetSpeed
{
    float speed[4];
    SpeedMode mode;
};

struct PidDebugData
{
    float cspeed;
    float tspeed;
    float pidout;
    float error;
};

struct RosbotSpeed
{
    float lin_x;
    float lin_y;
    float ang_z;
};

//TODO: documentation
/**
 * @brief Rosbot Drive Module.
 *
 * This class represents the ROSbot drive module.
 */
class RosbotDrive : NonCopyable<RosbotDrive>
{
public:
    static const RosbotWheel DEFAULT_WHEEL_PARAMS; /**< Default ROSbot's wheels parameters. */

    static const RosbotRegulator_params DEFAULT_REGULATOR_PARAMS; /**< Default ROSbot regulator parameters. */

    static RosbotDrive &getInstance();

    void init(const RosbotWheel &wheel_params, const RosbotRegulator_params &params);

    void enable(bool en = true);

    void enablePidReg(bool en);

    bool isPidEnabled();

    void stop();

    void setupMotorSequence(RosbotMotNum first, RosbotMotNum second, RosbotMotNum third, RosbotMotNum fourth);

    float getSpeed(RosbotMotNum mot_num);

    float getSpeed(RosbotMotNum mot_num, SpeedMode mode);

    float getDistance(RosbotMotNum mot_num);

    float getAngularPos(RosbotMotNum mot_num);

    int32_t getEncoderTicks(RosbotMotNum mot_num);

    void resetDistance();

    void updateTargetSpeed(const NewTargetSpeed &new_speed);

    void updateWheelCoefficients(const RosbotWheel &params);

    void updatePidParams(const RosbotRegulator_params &params);

    void getPidParams(RosbotRegulator_params &params);

    // void getPidDebugData(PidDebugData * data, RosbotMotNum mot_num);

private:
    static RosbotDrive *_instance;

    RosbotDrive();

    void regulatorLoop();

    volatile RosbotDriveStates _state;
    volatile bool _regulator_output_enabled;
    volatile bool _regulator_loop_enabled;

    RosbotWheel _wheel_params;

    volatile float _tspeed_mps[4];
    volatile float _cspeed_mps[4];
    volatile int32_t _cdistance[4];
    uint8_t _motor_sequence[4];

    int _regulator_interval_ms;

    float _wheel_coefficient1;
    float _wheel_coefficient2;

    DRV8848 *_mot_driver[2];
    DRV8848::DRVMotor *_mot[4];
    Encoder *_encoder[4];
    RosbotRegulator *_regulator[4];

    Mutex rosbot_drive_mutex;
};

#endif /* __ROSBOT_DRIVE_H__ */