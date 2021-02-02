/** @file RosbotRegulator.h
 * Regulator abstract class.
 */
#ifndef __ROSBOT_REGULATOR_H__
#define __ROSBOT_REGULATOR_H__

struct RosbotRegulator_params
{
    float kp;
    float ki;
    float kd;
    float out_min;
    float out_max;
    float a_max;
    float speed_max;
    unsigned int dt_ms;
};

class RosbotRegulator
{
public:
    RosbotRegulator(const RosbotRegulator_params &params)
    : _params(params)
    {}

    virtual ~RosbotRegulator(){};

    virtual void updateParams(const RosbotRegulator_params &params)=0;

    virtual void getParams(RosbotRegulator_params &params)=0;

    virtual float updateState(float setpoint, float feedback)=0;

    virtual void reset()=0;

    virtual float getPidout()=0;

    virtual float getError()=0;

protected:
    RosbotRegulator_params _params;
};

#endif /* __ROSBOT_REGULATOR_H__ */