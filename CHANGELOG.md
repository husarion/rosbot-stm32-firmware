# ROSbot firmware CHANGELOG

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/) and this project adheres to [Semantic Versioning](http://semver.org/).

## [0.1.0] - 2019-06-19
author: [byq77](https://github.com/byq77)

Initial log. Introducing new ROSbot firmware written using Mbed OS framework.

### Added 
  - VL53L0X sensors support
  - backward compatible ROS interface
  - battery voltage measurement
  - support for 4 wheel and 2 wheel ROSbot

## [0.2.0] - 2019-06-25
author: [byq77](https://github.com/byq77)

### Fixed
  - `rosserial-mbed.lib` v.1.1.1 fixes Buffer bug that would prevent usage larger number of publishers/subscribers

### Added
  - MPU9250 support. ROS interface consists of `/imu` topic with `geometry_msgs/QuaternionStamped` message.
  - Buttons support. ROS interface consists of `/buttons` topic with `std_msgs/UInt8`
    - `1` - BUTTON1 was pressed
    - `2` - BUTTON2 was pressed
  - Multipurpose service server `/config`. See `README.md` to learn more.

### Changed
  - VL53LX0 and MPU9250 are disabled on default. They can be enabled using `/config` service server.

## [0.3.0] - 2019-06-27
author: [byq77](https://github.com/byq77)

### Added
 - new commands:
   * `EJSM` - ENABLE JOINT STATES MESSAGES
   * `RODOM` - RESET ODOMETRY

### Fixed
 - fixed odometry for 4-wheeled robot
  
### Changed
- VL53L0X i2c interface frequency set at 400kHz
- BATTERY LOW indication set at 10.8V

## [0.4.0] - 2019-07-19
author: [byq77](https://github.com/byq77)

### Added
  - new command:
    * `RIMU` - RESET IMU
    * `EWCH` - ENABLE SPEED WATCHDOG 
  - speed watchdog (by default enabled), that sets speed to 0 if there is no speed update of `/cmd_vel` for 1s
  - support for EKF

### Changed
  - `imu` topic changed to `mpu9250` with message type `rosbot/Imu`

### Fixed
  - more frequent odometry update
  - odometry coefficient are slightly better adjusted
  - overall firmware's stability is improved
  - max target speed limited to 1.5m/s

## [0.4.1] - 2019-07-30
author: [byq77](https://github.com/byq77)

### Changed
- LPF set to 42 Hz and Gyro FSR set to 500dps for IMU.
- Mbed version changed from `5.12.4` to `5.13.1`

## [0.5.0] - 2019-08-01
author: [byq77](https://github.com/byq77)

### Changed
  - Improved DDR8848 driver implementation for CORE2. Motors' pwm frequency is now set to 21kHz with 1000 steps resolution. 
  - changed symbols C and C++ versions (following C11 and C++14 support in Mbed 5.13.x)

## [0.6.0] - 2019-08-14
author: [byq77](https://github.com/byq77)

### Added
  - support for WS2812B LEDs signalization
  - new command:
    * `SANI` - SET WS2812B LEDS ANIMATION

### Changes
  - some code refactoring and updates
  - `DIAMETER_MODIFICATOR` factor update 

## [0.7.0] - 2019-08-23
author: [byq77](https://github.com/byq77)

### Added 
  - `AnimationManager` class to control ws2812b's signalization

### Changed
  - Better rosserial interface for ws2812b signalization, now you can instantly change the animation and the color. 

## [0.7.1] - 2019-08-27
author: [byq77](https://github.com/byq77)

### Changes
- the support package name changed from `rosbot` to `rosbot_ekf`

## [0.7.2] - 2019-09-26
author: [byq77](https://github.com/byq77)

### Fixed
  - critical changes in CORE2 target definitions

## [0.8.0] - 2019-10-22
author: [byq77](https://github.com/byq77)

### Changed
  - mpu9250 i2c implementation changed to non-blocking
  - increased baudrate for Upboard
  - distance sensors enabled by default
  - mbed version changed to `5.14.1`

### Added  
  - firmware version is now printed as ros log

### Fixed
  - mpu9250 related bugs

## [0.9.0] - 2019-11-06
author: [byq77](https://github.com/byq77)

### Changed
  - Improved RosbotDrive module implementation
    * removed most of the unused and trash code,
    * regulator implementation moved to separate class that implements `RosbotRegulator` interface,
    * class api uses references instead pointers,
    * added motor output setup sequence
  - other small code and configuration improvements

### Added
  - Regulator now implements soft start and soft stop.
  - `CALI` command for wheel odometry calibration.s

## [0.9.1] - 2019-11-19
author: [byq77](https://github.com/byq77)

### Changed
  - Use direct access to encoder ticks count in `getDistance`, `getAngularPos` and `getEncoderTicks`.

### Fixed
  - Bug in `getPidout` (`RosbotRegulatorCmsis.h`).

## [0.9.2] - 2019-12-10
author: [byq77](https://github.com/byq77)

### Fixed
  - USART6 pins configuration. 

## [0.10.0] - 2020-12-20
author: [byq77](https://github.com/byq77)

### Changed
  - Topic names changed from global to relative.
  - New non-blocking implementation of VL53L0X library.
  - Small changes in implementation of MPU9250 library.
  - Improved accuracy of timestamps in ros messages.

### Added
  - `update_symbols.py` script for `c_cpp_properties.json` symbols update.

### Fixed
  - Fixed distance sensors behavior.
  - Fixed sensors error messages

### Added
  - Automatic distance sensors reinitialization in case of sensor malfunction/i2c error.

## [0.10.1] - 2020-03-10
author: [byq77](https://github.com/byq77)

### Fixed
  - MultiDistanceSensor implementation improved (support of new sensors hardware). 

### Changed  
  - It is possible now to initialize less than 4 proximity sensors, but the error message will be printed.

## [0.11.0] - 2020-05-19
author: [byq77](https://github.com/byq77)

### Added
  - Support for hServo outputs control.
  - New subscriber on `/cmd_ser` topic for servo output control. See `README` for more details.
  - New command `CSER` that allows servo outputs configuration. See `README` for more details.

## [0.12.0] - 2020-06-09
author: [byq77](https://github.com/byq77)

### Changed
  - Updated rosserial-mbed implementation (BufferedSerial lib changed to UARTSerial).
  - `/pose` and `/velocity` publication frequency increased to ~ 20Hz. 

## [0.13.0] - 2020-06-30
author: [byq77](https://github.com/byq77)

### Added
  - New command `CPID` that enables pid configuration. See `README` for more details.
  - New command `GPID` that returns current pid configuration. See `README` for more details.

### Changed
  - Updated mpu9250-mbed library.
  - New default PID regulator settings.

## [0.13.0] - 2020-07-07
author: [byq77](https://github.com/byq77)

### Fixed
  - Update MPU9250 readings only if there is more then 32 bytes in the fifo.
  - Fixed servo numeration in cmd_ser topic.

## [0.14.0] - 2020-11-13
author: [byq77](https://github.com/byq77)

### Added
  - support for BNO055 IMU sensor

## [0.14.1] - 2020-11-16
author: [byq77](https://github.com/byq77)

### Added
  - angular velocity and linear acceleration for BNO055 IMU sensor

## [0.14.2] - 2020-12-18
author: [byq77](https://github.com/byq77)

### Fixed
  - fixed bug with MPU9250 initialization (both sensors support)

## [0.14.3] - 2021-02-04
  - mecanum kinematics 
  - service for change kinematics (DIFF/MEC)
  - changed baud to "525000" for both Rosbot and Rosbot PRO
  - BUG found: resetting odometry not working 

## [0.14.4] - 2021-05-10

### Changed
  - Imu message changed to `sensors_msgs/Imu`
  - updated `core2-imu-driver` lib

### Fixed
  - fixed the bug in the odometry
  - reverted odometry params

## [0.14.5] 

TODO: add changes

## [0.15.0] - 2021-10-20

### Fixed
- Fixed acc bug for both imu sensors.
- Fixed the imu acc and gyro units to match units in the sensor_msgs/Imu message.
- Fixed acc vector orientation for mpu9250.

## [0.16.0] - 2022-06-17

### Fixed
- IMU driver update
- Fixed odometry for mecanum wheels

## [0.16.1] - 2022-06-17

### Fixed
- add JOINT_STATES_ENABLE flag 

## [0.16.2] - 2023-04-12

### Fixed
- repo dependencies
- Done some build troubleshooting for Python 3.10.x
- updated workflows
- fixed version print at startup