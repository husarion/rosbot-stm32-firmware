#include "RosbotDrive.h"
#include "RosbotRegulatorCMSIS.h"
#define PWM_DEFAULT_FREQ_HZ 18000UL /**< Default frequency for motors' pwms.*/

#define FOR(x) for(int i=0;i<x;i++)

static const DRV8848_Params_t DEFAULT_MDRV1_PARAMS{
    MOT1A_IN,
    MOT1B_IN,
    MOT1_PWM,
    MOT2A_IN,
    MOT2B_IN,
    MOT2_PWM,
    MOT12_FAULT,
    MOT12_SLEEP};

static const DRV8848_Params_t DEFAULT_MDRV2_PARAMS{
    MOT3A_IN,
    MOT3B_IN,
    MOT3_PWM,
    MOT4A_IN,
    MOT4B_IN,
    MOT4_PWM,
    MOT34_FAULT,
    MOT34_SLEEP};

const RosbotWheel RosbotDrive::DEFAULT_WHEEL_PARAMS = {
    .radius = 0.0425,
    .diameter_modificator = 1.0f,
    .tyre_deflation = 1.0f,
    .gear_ratio = 34.014,
    .encoder_cpr = 48,
    .polarity = 0b00111100};

// const RosbotRegulator_params RosbotDrive::DEFAULT_REGULATOR_PARAMS = {
//     .kp = 0.8,
//     .ki = 0.2,
//     .kd = 0.015,
//     .out_min = -1.0,
//     .out_max = 1.0,
//     .a_max = 1.5e-4,
//     .speed_max = 1.5,
//     .dt_ms = 10};

const RosbotRegulator_params RosbotDrive::DEFAULT_REGULATOR_PARAMS = {
    .kp = 0.8,
    .ki = 0.2,
    .kd = 0.015,
    .out_min = -0.8,
    .out_max = 0.8,
    .a_max = 1.5e-4,
    .speed_max = 1.0,
    .dt_ms = 10};

RosbotDrive * RosbotDrive::_instance = NULL;

/* static objects begin (memory optimizations) */
static Thread regulator_thread(osPriorityHigh);
static DRV8848 mot_driver1(&DEFAULT_MDRV1_PARAMS);
static DRV8848 mot_driver2(&DEFAULT_MDRV2_PARAMS);
static Encoder encoder1(ENCODER_1);
static Encoder encoder2(ENCODER_2);
static Encoder encoder3(ENCODER_3);
static Encoder encoder4(ENCODER_4);
/* static objects end (memory optimizations)*/

RosbotDrive::RosbotDrive()
: _state(UNINIT)
, _regulator_output_enabled(false)
, _regulator_loop_enabled(true)
, _tspeed_mps{0,0,0,0}
, _cspeed_mps{0,0,0,0}
, _cdistance{0,0,0,0}
, _motor_sequence{0,1,2,3}
, _mot_driver{NULL,NULL}
, _mot{NULL,NULL,NULL,NULL}
, _encoder{NULL,NULL,NULL,NULL}
{}

RosbotDrive & RosbotDrive::getInstance()
{
    if(_instance==NULL)
    {
        static RosbotDrive instance;
        _instance = &instance;
    }
    return *_instance;
}

void RosbotDrive::init(const RosbotWheel & wheel_params, const RosbotRegulator_params & reg_params)
{
    static bool initialized = false;
    if(initialized)
        return;
    
    _mot_driver[0] = &mot_driver1;
    _mot_driver[1] = &mot_driver2;
    _encoder[0] = &encoder1;
    _encoder[1] = &encoder2;
    _encoder[2] = &encoder3;
    _encoder[3] = &encoder4;

    FOR(2)
    {
        _mot[i]=_mot_driver[0]->getDCMotor((MotNum)i);
        _mot[i+2]=_mot_driver[1]->getDCMotor((MotNum)i);
    }

    // Use CMSIS PID regulator
    FOR(4) _regulator[i] = new RosbotRegulatorCMSIS(reg_params); //TODO change to static

    _regulator_interval_ms = reg_params.dt_ms;

    _wheel_coefficient1 =  2 * M_PI * wheel_params.radius / (wheel_params.gear_ratio * wheel_params.encoder_cpr * wheel_params.tyre_deflation);
    _wheel_coefficient2 =  2 * M_PI / (wheel_params.gear_ratio * wheel_params.encoder_cpr);

    FOR(4)
    {
        _mot[i]->setPolarity(wheel_params.polarity>>i & 1);
        _mot[i]->init(PWM_DEFAULT_FREQ_HZ);
        _mot[i]->setDriveMode(true);
        _encoder[i]->setPolarity(wheel_params.polarity>>(i+4) & 1);
        _encoder[i]->init();
    }
    
    _state=HALT;

    FOR(2) _mot_driver[i]->enable(true);

    initialized = true;

    regulator_thread.start(callback(this,&RosbotDrive::regulatorLoop));
}

void RosbotDrive::enable(bool en)
{
    if(_state == UNINIT)
        return;
    
    switch(_state)
    {
        case HALT:
            if(en)
                _state=OPERATIONAL;
            else
                _state=IDLE;

            FOR(2) _mot_driver[i]->enable(en);
            
            break;
        case IDLE:
            if(en)
            {
                _state=OPERATIONAL;
                FOR(2) _mot_driver[i]->enable(en);
            } 
            break;
        case OPERATIONAL:
            if(!en)
            {
                _state=IDLE;
                FOR(4) 
                {
                    _mot[i]->setPower(0);
                    _tspeed_mps[i]=0;
                    _regulator[i]->reset();
                }
                FOR(2) _mot_driver[i]->enable(en);
            } 
            break;
        default:
            break;
    }
}

void RosbotDrive::regulatorLoop()
{
    uint64_t sleepTime;
    int32_t distance;
    float factor1;
    int mot_num;
    while (1)
    {
        sleepTime = Kernel::get_ms_count() + _regulator_interval_ms;
        if (_regulator_loop_enabled) //TODO: change to mutex with fixed held time
        {

            factor1 = 1000.0 * _wheel_coefficient1 / _regulator_interval_ms;
            FOR(4)
            {
                distance = _encoder[i]->getCount();
                _cspeed_mps[i] = (float)(distance - _cdistance[i]) * factor1;
                _cdistance[i] = distance;
                
            }
            if ((_state == OPERATIONAL) && _regulator_output_enabled)
            {
                FOR(4)
                {
                    mot_num = _motor_sequence[i];
                    _mot[mot_num]->setPower(_regulator[mot_num]->updateState(_tspeed_mps[mot_num],_cspeed_mps[mot_num]));
                }
            }
        }
        ThisThread::sleep_until(sleepTime);
    }
}

void RosbotDrive::updateTargetSpeed(const NewTargetSpeed & new_speed)
{
    if(_state != OPERATIONAL)
        return;
    switch(new_speed.mode)
    {
        case DUTY_CYCLE:
            if(!_regulator_output_enabled)
                FOR(4) {_mot[i]->setPower(new_speed.speed[i]);}
            break;
        case MPS:
            if(_regulator_output_enabled)
                FOR(4) {_tspeed_mps[i]=new_speed.speed[i];}
            break;
        default:
            return;
    }
}

float RosbotDrive::getDistance(RosbotMotNum mot_num)
{
    return (float)_wheel_coefficient1 * _encoder[mot_num]->getCount();
}

float RosbotDrive::getAngularPos(RosbotMotNum mot_num)
{
    return (float)_wheel_coefficient2 * _encoder[mot_num]->getCount();
}

int32_t RosbotDrive::getEncoderTicks(RosbotMotNum mot_num)
{
    return _encoder[mot_num]->getCount();
}

float RosbotDrive::getSpeed(RosbotMotNum mot_num)
{
    return _cspeed_mps[mot_num];
}

void RosbotDrive::updateWheelCoefficients(const RosbotWheel & params)
{
    _regulator_loop_enabled = false;
    _wheel_coefficient1 =  2 * M_PI * params.radius / (params.gear_ratio * params.encoder_cpr * params.tyre_deflation);
    _wheel_coefficient2 =  2 * M_PI / (params.gear_ratio * params.encoder_cpr);
    _regulator_loop_enabled = true;
}

void RosbotDrive::updatePidParams(const RosbotRegulator_params & params)
{
    _regulator_loop_enabled = false;
        FOR(4) _regulator[i]->updateParams(params);
    _regulator_loop_enabled = true;
}

void RosbotDrive::getPidParams(RosbotRegulator_params & params)
{
    _regulator[0]->getParams(params);
}

void RosbotDrive::stop()
{
    switch(_state)
    {
        case IDLE:
            _state=HALT;
            FOR(2) _mot_driver[i]->enable(true);
            break;
        case OPERATIONAL:
            _state=HALT;
            FOR(4)  
            {
                _tspeed_mps[i]=0;
                _mot[i]->setPower(0);
            }
            break;
        default:
            break;
    }
}

void RosbotDrive::enablePidReg(bool en)
{
    _regulator_output_enabled = en;
}

bool RosbotDrive::isPidEnabled() 
{
    return _regulator_output_enabled;
}

// void RosbotDrive::getPidDebugData(PidDebugData_t * data, RosbotMotNum mot_num)
// {
//     // CriticalSectionLock lock;
//     data->cspeed=_cspeed_mps[mot_num];
//     data->tspeed=_tspeed_mps[mot_num];
//     data->error=_error[mot_num];
//     data->pidout=_pidout[mot_num];
// }

float RosbotDrive::getSpeed(RosbotMotNum mot_num, SpeedMode mode)
{
    switch(mode)
    {
        case TICSKPS:
            return _cspeed_mps[mot_num]/_wheel_coefficient1;
        case MPS:
            return _cspeed_mps[mot_num];
        case DUTY_CYCLE:
            return _mot[mot_num]->getDutyCycle();
        default:
            return 0.0;
    }
}

void RosbotDrive::resetDistance()
{
    bool tmp = _regulator_output_enabled;
    _regulator_loop_enabled = false;
    FOR(4) _mot[i]->setPower(0);
    FOR(4)
    {
        _encoder[i]->resetCount();
        _regulator[i]->reset();
        _tspeed_mps[i]=0;
        _cspeed_mps[i]=0;
        _cdistance[i]=0;
    }
    _regulator_loop_enabled = tmp;
}

void RosbotDrive::setupMotorSequence(RosbotMotNum first, RosbotMotNum second, RosbotMotNum third, RosbotMotNum fourth)
{
    bool tmp = _regulator_output_enabled;
    _regulator_output_enabled = false;
    _motor_sequence[0] = first;
    _motor_sequence[1] = second;
    _motor_sequence[2] = third;
    _motor_sequence[3] = fourth;
    _regulator_output_enabled = tmp;
}