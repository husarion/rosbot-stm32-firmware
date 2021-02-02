#ifndef __ROSBOT_SENSORS_H__
#define __ROSBOT_SENSORS_H__

#include <mbed.h>
#include <MultiDistanceSensor.h>

namespace rosbot_sensors{

float updateBatteryWatchdog();

class ServoManger : NonCopyable<ServoManger>
{

public:
    enum : int
    {
        SERVO_OUTPUT_1 = 0,
        SERVO_OUTPUT_2 = 1,
        SERVO_OUTPUT_3 = 2,
        SERVO_OUTPUT_4 = 3,
        SERVO_OUTPUT_5 = 4,
        SERVO_OUTPUT_6 = 5,
        VOLTAGE_5V = 0,
        VOLTAGE_6V = 1,
        VOLTAGE_7_4V = 2,
        VOLTAGE_8_6V = 3, 
    };

    ServoManger()
    : _servo{nullptr, nullptr, nullptr, nullptr, nullptr, nullptr}
    , _voltage_mode(0)
    , _enabled_outputs(0)
    , _servo_sel1(SERVO_SEL1,0)
    , _servo_sel2(SERVO_SEL2,0)
    , _servo_power(SERVO_POWER_ON,0)
    {}

    void enablePower(bool en = true)
    {
        _servo_power.write( en ? 1 : 0);
    }

    int getEnabledOutputs()
    {
        return _enabled_outputs;
    }

    PwmOut * getOutput(int output)
    {
        if(output < SERVO_OUTPUT_1 || output > SERVO_OUTPUT_6)
            return nullptr;

        return _servo[output];
    }

    bool setWidth(int output, int width_us)
    {
        if(output < SERVO_OUTPUT_1 || output > SERVO_OUTPUT_6)
            return false;

        if(_servo[output] == nullptr)
            return false;

        _servo[output]->pulsewidth_us(width_us);
        return true;
    }

    bool setPeriod(int output, int period_us)
    {
        if(output < SERVO_OUTPUT_1 || output > SERVO_OUTPUT_6)
            return false;

        if(_servo[output] == nullptr)
            return false;

        _servo[output]->period_us(period_us);
        return true;
    }

    void enableOutput(int output, bool en = true)
    {
        if(output < SERVO_OUTPUT_1 || output > SERVO_OUTPUT_6)
            return;
        
        if(en && _servo[output] == nullptr)
        {
            switch (output)
            {
            case SERVO_OUTPUT_1:
                _servo[output] = new PwmOut(SERVO1_PWM);
                break;
            case SERVO_OUTPUT_2:
                _servo[output] = new PwmOut(SERVO2_PWM);
                break;
            case SERVO_OUTPUT_3:
                _servo[output] = new PwmOut(SERVO3_PWM);
                break;
            case SERVO_OUTPUT_4:
                _servo[output] = new PwmOut(SERVO4_PWM);
                break;
            case SERVO_OUTPUT_5:
                _servo[output] = new PwmOut(SERVO5_PWM_ALT1);
                break;
            case SERVO_OUTPUT_6:
                _servo[output] = new PwmOut(SERVO6_PWM_ALT1);
                break;
            }
            _enabled_outputs++;
        }
        else if(_servo[output] != nullptr && !en)
        {
            delete _servo[output];
            _servo[output] = nullptr;
            _enabled_outputs--;
        }
    }

    void setPowerMode(int mode)
    {
        _servo_sel1.write(mode & 0x01L);
        _servo_sel2.write(mode & 0x02L);
    }

private:
    PwmOut *_servo[6];
    int _voltage_mode;
    int _enabled_outputs;
    DigitalOut _servo_sel1;
    DigitalOut _servo_sel2;
    DigitalOut _servo_power;
};

}

#endif /* __ROSBOT_SENSORS_H__ */