#ifndef __MULTI_DISTANCE_SENSOR_H__
#define __MULTI_DISTANCE_SENSOR_H__

#include <VL53L0X.h>

#define NUM_DISTANCE_SENSORS 4
#define DISTANCE_SENSORS_DEFAULT_I2C_FREQ 100000U

enum SensorSelector : uint8_t
{
    SENSOR_FR = 0,
    SENSOR_FL = 1,
    SENSOR_RR = 2,
    SENSOR_RL = 3
};

struct SensorsMeasurement
{
    float range[4];
    uint32_t timestamp;
    uint8_t status;
};

extern Mail<SensorsMeasurement, 5> distance_sensor_mail_box;
extern Mail<uint8_t, 5> distance_sensor_commands;

class MultiDistanceSensor : NonCopyable<MultiDistanceSensor>
{

public:
    enum : int
    {
        ERR_NONE = 0,
        ERR_BUSSY = 1,
        ERR_I2C_FAILURE = 2,
        ERR_NOT_READY = 3,
        ERR_NOT_INIT = 4
    };
    static MultiDistanceSensor & getInstance();    
    int init();
    
private:
    static MultiDistanceSensor * _instance;
    MultiDistanceSensor();
    ~MultiDistanceSensor();
    void runMeasurement();
    void start();
    void stop();
    int restart();
    void sensors_loop();
    void processOut();

    I2C * _i2c;
    DigitalInOut * _xshout[NUM_DISTANCE_SENSORS];
    VL53L0X * _sensor[NUM_DISTANCE_SENSORS];
    bool _is_active[NUM_DISTANCE_SENSORS];
    bool _initialized;
    bool _sensors_enabled;
    int _last_sensor_index;
    SensorsMeasurement _m;
    Thread _distance_sensor_thread;
};

#endif