#include "rosbot_sensors.h"

namespace rosbot_sensors{

#pragma region BATTERY_REGION

#define MEASUREMENT_SERIES 10
#define BATTERY_VOLTAGE_LOW 10.8

enum 
{
    BATTERY_LOW = 1,
    BATTERY_OK = 0
};

typedef struct BatteryData
{
    float voltage;
    float threshold;
    uint8_t status;
}BatteryData_t;

static BatteryData_t battery_data = { 0.0, BATTERY_VOLTAGE_LOW, BATTERY_OK}; 
static DigitalOut battery_led(LED1,1);
static Ticker battery_led_flipper;
static void readVoltageInternal();
static AnalogIn battery_adc(BAT_MEAS);

static void batteryLed()
{
    battery_led = !battery_led;
}

float updateBatteryWatchdog()
{
    static int index=0;
    battery_data.voltage = 3.3f * VIN_MEAS_CORRECTION * (UPPER_RESISTOR + LOWER_RESISTOR)/LOWER_RESISTOR * battery_adc.read();
    if(battery_data.threshold > battery_data.voltage && index < MEASUREMENT_SERIES) // low level
        index ++;
    else if(battery_data.threshold < battery_data.voltage && index > 0)
        index --;

    if(battery_data.status == BATTERY_OK && index == MEASUREMENT_SERIES)
    {
        battery_data.status = BATTERY_LOW;
        battery_led = 0;
        battery_led_flipper.attach(callback(batteryLed),0.4);
    }
    else if(battery_data.status == BATTERY_LOW && index == 0)
    {
        battery_data.status = BATTERY_OK;
        battery_led_flipper.detach();
        battery_led = 1;
    }
    return battery_data.voltage;
}

#pragma endregion BATTERY_REGION

}