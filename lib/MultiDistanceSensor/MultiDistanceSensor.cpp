#include "MultiDistanceSensor.h"

static const uint8_t DEFAULT_HW_ADDRESS = 0x29;

static const uint8_t SENSOR_HW_ADDRESS[]={
    DEFAULT_HW_ADDRESS + 1,
    DEFAULT_HW_ADDRESS + 2,
    DEFAULT_HW_ADDRESS + 3,
    DEFAULT_HW_ADDRESS + 4,
};

#if defined(TARGET_CORE2)

#define SENSOR_FR_XSHOUT_PIN SENS6_PIN1
#define SENSOR_FL_XSHOUT_PIN SENS6_PIN2 
#define SENSOR_RR_XSHOUT_PIN SENS6_PIN4
#define SENSOR_RL_XSHOUT_PIN SENS6_PIN3 
#define SENSORS_SDA_PIN SENS1_PIN4
#define SENSORS_SCL_PIN SENS1_PIN3

static DigitalInOut xshout[]={
DigitalInOut(SENSOR_FR_XSHOUT_PIN, PIN_OUTPUT, OpenDrainNoPull, 0),
DigitalInOut(SENSOR_FL_XSHOUT_PIN, PIN_OUTPUT, OpenDrainNoPull, 0),
DigitalInOut(SENSOR_RR_XSHOUT_PIN, PIN_OUTPUT, OpenDrainNoPull, 0),
DigitalInOut(SENSOR_RL_XSHOUT_PIN, PIN_OUTPUT, OpenDrainNoPull, 0)};

#else
    #error "Your target is not supported!"
#endif /* TARGET_CORE2 */

MultiDistanceSensor * MultiDistanceSensor::_instance = nullptr;

Mail<SensorsMeasurement, 5> distance_sensor_mail_box;
Mail<uint8_t, 5> distance_sensor_commands;

MultiDistanceSensor::MultiDistanceSensor()
:_i2c(nullptr)
,_xshout{nullptr, nullptr, nullptr, nullptr}
,_is_active{false,false,false,false}
,_initialized(false)
,_sensors_enabled(false)
,_last_sensor_index(-1)
{}

MultiDistanceSensor & MultiDistanceSensor::getInstance()
{
    if(_instance==NULL)
    {
        static MultiDistanceSensor instance;
        _instance = &instance;
    }
    return *_instance;
}

MultiDistanceSensor::~MultiDistanceSensor()
{
    stop();
    for(int i=0;i<4;i++) delete _sensor[i];
}

int MultiDistanceSensor::restart()
{
    int result=0;

    for(int i=0;i<4;i++)
    {
        _xshout[i]->write(0);
        _sensor[i]->setDefaultAddress();
        _is_active[i] = false;
    }

    ThisThread::sleep_for(10);
    
    _i2c->frequency(DISTANCE_SENSORS_DEFAULT_I2C_FREQ);

    for(int i=0;i<4;i++)
    {
        _sensor[i]->setTimeout(500);
        _xshout[i]->write(1);
        ThisThread::sleep_for(10);
        if(_sensor[i]->init())
        {
            _sensor[i]->setAddress(SENSOR_HW_ADDRESS[i]);
            _sensor[i]->setMeasurementTimingBudget(80);
            _is_active[i]=true;
            _last_sensor_index = i;
            result++;
        }
        else
        {
            _xshout[i]->write(0);
        }
    }

    return result;
}

void MultiDistanceSensor::stop()
{
    for(int i=0;i<NUM_DISTANCE_SENSORS;i++)
    {
        if(_is_active[i])
        {
            _sensor[i]->stopContinuous();
        }
    }
    _sensors_enabled = false;
}

void MultiDistanceSensor::start()
{
    for(int i=0;i<NUM_DISTANCE_SENSORS;i++)
    {
        if(_is_active[i])
        {
            // _sensor[i]->setTimeout(100);
            _sensor[i]->startContinuous(100);
        }
    }
    _sensors_enabled = true;
}

int MultiDistanceSensor::init()
{
    if(_initialized)
        return 0;
    
    _i2c = new I2C(SENSORS_SDA_PIN,SENSORS_SCL_PIN);        

    for(int i=0;i<NUM_DISTANCE_SENSORS;i++){
        _sensor[i] = new VL53L0X(*_i2c);
        _xshout[i] = &xshout[i]; 
    } 

    _initialized = true;

    int result;
    if((result = restart()) > 0)
        _distance_sensor_thread.start(callback(this,&MultiDistanceSensor::sensors_loop));
    return result;
}

void MultiDistanceSensor::sensors_loop()
{
    while (1)
    {
        osEvent evt = distance_sensor_commands.get(0);

        if(evt.status == osEventMail)
        {
            uint8_t * command = (uint8_t *)evt.value.p;
            switch(*command)
            {
                case 0:
                    stop();
                    break;
                case 1:
                    start();
                    break;
                case 2:
                    _i2c->abort_transfer();
                    restart();
                    break;
                default:
                    break;
            }
            distance_sensor_commands.free(command);
        }

        if (_sensors_enabled) runMeasurement();
        
        ThisThread::sleep_for(10);
    }
}

void MultiDistanceSensor::processOut()
{
    // static uint32_t spin_cnt = 0;

    // if(_m.status == ERR_NOT_INIT && spin_cnt++ % 50 != 0)
    //     return;

    if(_m.status != ERR_NOT_INIT && !distance_sensor_mail_box.full())
    {
        SensorsMeasurement * msg = distance_sensor_mail_box.alloc();
        if (msg == nullptr)
            return;
        
        memcpy(&msg->range,&_m.range,sizeof(_m.range));
        msg->timestamp = _m.timestamp;
        msg->status = _m.status;
        distance_sensor_mail_box.put(msg);
    }
}

void MultiDistanceSensor::runMeasurement()
{
    if(_last_sensor_index == -1)
    {
        _m.status = ERR_NOT_INIT;
        return processOut();
    }

    bool is_measurement_ready = (_sensor[_last_sensor_index]->readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07); 
    
    if (_sensor[_last_sensor_index]->last_status != 0)
    {
        _xshout[_last_sensor_index]->write(0);
        _is_active[_last_sensor_index] = false;

        _m.status = ERR_I2C_FAILURE;
        _m.timestamp = Kernel::get_ms_count();
        for(int i=0; i<NUM_DISTANCE_SENSORS; i++) _m.range[i] = -1.0f;         
        
        return processOut();
    }
    else if(!is_measurement_ready)
    {
        return;
    }
    else
    {
        _m.timestamp = Kernel::get_ms_count();
        _m.status = ERR_NONE;

        for(int i=0; i<NUM_DISTANCE_SENSORS; i++)
        {
            if(_is_active[i])
            {
                uint16_t range = _sensor[i]->readRangeContinuousMillimeters(false);
                if(_sensor[i]->last_status == 0)
                {
                    _m.range[i] = (float) range / 1000.0;
                }
                else
                {
                    _m.range[i] = -1.0;
                    _is_active[i] = false;
                    _xshout[i]->write(0);     
                    _m.status = ERR_I2C_FAILURE;
                }
            }
        }
        
        return processOut();
    }
}