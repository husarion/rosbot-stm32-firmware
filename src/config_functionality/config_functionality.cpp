#include <config_functionality/config_functionality.hpp>

ConfigFunctionality::ConfigFunctionality() {
    _commands[SLED_COMMAND] = &ConfigFunctionality::setLed;
    _commands[EIMU_COMMAND] = &ConfigFunctionality::enableImu;
    _commands[EDSE_COMMAND] = &ConfigFunctionality::enableDistanceSensors;
    _commands[EJSM_COMMAND] = &ConfigFunctionality::enableJointStates;
    _commands[RODOM_COMMAND] = &ConfigFunctionality::resetOdom;
    _commands[EWCH_COMMAND] = &ConfigFunctionality::enableSpeedWatchdog;
    _commands[RIMU_COMMAND] = &ConfigFunctionality::resetImu;
    _commands[SANI_COMMAND] = &ConfigFunctionality::setAnimation;
    _commands[ETFM_COMMAND] = &ConfigFunctionality::enableTfMessages;
    _commands[CALI_COMMAND] = &ConfigFunctionality::calibrateOdometry;
    _commands[EMOT_COMMAND] = &ConfigFunctionality::enableMotors;
    _commands[CSER_COMMAND] = &ConfigFunctionality::configureServo;
    _commands[GPID_COMMAND] = &ConfigFunctionality::getPid;
    _commands[CPID_COMMAND] = &ConfigFunctionality::configurePid;
    _commands[SKIN_COMMAND] = &ConfigFunctionality::setKinematics;
    _commands[GKIN_COMMAND] = &ConfigFunctionality::getKinematics;
}

uint8_t ConfigFunctionality::enableTfMessages(const char *datain, const char **dataout) {
    int en;
    if (sscanf(datain, "%d", &en) == 1) {
        // tf_msgs_enabled = en ? true : false;
        // if (tf_msgs_enabled)
        // {rosbot_ekf
        //     initTfPublisher();
        // }
        return rosbot_ekf::Configuration::Response::SUCCESS;
    }
    return rosbot_ekf::Configuration::Response::FAILURE;
}

uint8_t ConfigFunctionality::configurePid(const char *datain, const char **dataout) {
    return pidCommandParser(datain) ? rosbot_ekf::Configuration::Response::SUCCESS : rosbot_ekf::Configuration::Response::FAILURE;
}

uint8_t ConfigFunctionality::getPid(const char *datain, const char **dataout) {
    RosbotRegulator_params params;
    RosbotDrive::getInstance().getPidParams(params);
    sprintf(this->_buffer, "%kp:%.3f ki:%.3f kd:%.3f out_max:%.3f out_min:%.3f a_max:%.3e speed_max: %.3f",
            params.kp, params.ki, params.kd, params.out_max, params.out_min, params.a_max, params.speed_max);
    *dataout = this->_buffer;
    return rosbot_ekf::Configuration::Response::SUCCESS;
}

uint8_t ConfigFunctionality::configureServo(const char *datain, const char **dataout) {
    return servoCommandParser(datain) ? rosbot_ekf::Configuration::Response::SUCCESS : rosbot_ekf::Configuration::Response::FAILURE;
}

uint8_t ConfigFunctionality::calibrateOdometry(const char *datain, const char **dataout) {
    float diameter_modificator, tyre_deflation;
    if (sscanf(datain, "%f %f", &diameter_modificator, &tyre_deflation) == 2) {
        rk->calibrateOdometry(diameter_modificator, tyre_deflation);
        RosbotDrive &drive = RosbotDrive::getInstance();
        drive.updateWheelCoefficients(rosbot_kinematics::custom_wheel_params);
        return rosbot_ekf::Configuration::Response::SUCCESS;
    }
    return rosbot_ekf::Configuration::Response::FAILURE;
}

uint8_t ConfigFunctionality::enableMotors(const char *datain, const char **dataout) {
    int en = atoi(datain);
    // if (en)
    //     nh.loginfo("Motors connected.");
    // else
    //     nh.loginfo("Motors disconnected.");
    RosbotDrive::getInstance().enable(en);
    return rosbot_ekf::Configuration::Response::SUCCESS;
}

// TODO change the implementation
uint8_t ConfigFunctionality::setAnimation(const char *datain, const char **dataout) {
#if USE_WS2812B_ANIMATION_MANAGER
    Color_t color;
    switch (datain[0]) {
        case 'O':
        case 'o':
            anim_manager->enableInterface(false);
            break;
        case 'F':
        case 'f':
            if (parseColorStr(&datain[2], &color))
                anim_manager->setFadingAnimation(&color);
            break;
        case 'B':
        case 'b':
            if (parseColorStr(&datain[2], &color))
                anim_manager->setBlinkingAnimation(&color);
            break;
        case 'R':
        case 'r':
            anim_manager->setRainbowAnimation();
            break;
        case 'S':
        case 's':
            if (parseColorStr(&datain[2], &color))
                anim_manager->setSolidColor(&color);
            break;
        default:
            return rosbot_ekf::Configuration::Response::FAILURE;
    }
#endif
    return rosbot_ekf::Configuration::Response::SUCCESS;
}

ConfigFunctionality::configuration_srv_fun_t ConfigFunctionality::findFunctionality(const char *command) {
    std::map<std::string, configuration_srv_fun_t>::iterator it = _commands.find(command);
    if (it != _commands.end())
        return it->second;
    else
        return NULL;
}

uint8_t ConfigFunctionality::resetImu(const char *datain, const char **dataout) {
    // TODO implement
    return rosbot_ekf::Configuration::Response::FAILURE;
}

uint8_t ConfigFunctionality::setMotorsAccelDeaccel(const char *datain, const char **dataout) {
    float accel, deaccel;
    // TODO implement
    return rosbot_ekf::Configuration::Response::FAILURE;
}

uint8_t ConfigFunctionality::resetOdom(const char *datain, const char **dataout) {
    RosbotDrive &drive = RosbotDrive::getInstance();
    rk->resetRosbotOdometry(drive, odometry);

    return rosbot_ekf::Configuration::Response::SUCCESS;
}

uint8_t ConfigFunctionality::setKinematics(const char *datain, const char **dataout) {
    if (strcmp(datain, "DIFF") == 0) {
        // nh.loginfo("Differential drive mode");
        rk = &diff_drive_kinematics;
        rk->setOdomParams();
        RosbotDrive &drive = RosbotDrive::getInstance();
        rk->resetRosbotOdometry(drive, odometry);
    } else if (strcmp(datain, "MEC") == 0) {
        // nh.loginfo("Mecanum drive mode");
        rk = &mecanum_drive_kinematics;
        rk->setOdomParams();
        RosbotDrive &drive = RosbotDrive::getInstance();
        rk->resetRosbotOdometry(drive, odometry);
    } else {
        // nh.logerror("Unrecognized data, use DIFF or MEC, provided:");
        // nh.logerror(datain);
        return rosbot_ekf::Configuration::Response::FAILURE;
    }
    return rosbot_ekf::Configuration::Response::SUCCESS;
}

uint8_t ConfigFunctionality::getKinematics(const char *datain, const char **dataout) {
    if (rk->getKinematicsType() == KINEMATICS_TYPE_DIFF_DRIVE)
        *dataout = "DIFF";
    else if (rk->getKinematicsType() == KINEMATICS_TYPE_MECANUM_DRIVE)
        *dataout = "MEC";
    return rosbot_ekf::Configuration::Response::SUCCESS;
}

uint8_t ConfigFunctionality::enableImu(const char *datain, const char **dataout) {
    int en;
    if (sscanf(datain, "%d", &en) == 1) {
        if (en) {
            imu_driver_ptr->start();
        } else {
            imu_driver_ptr->stop();
        }
        return rosbot_ekf::Configuration::Response::SUCCESS;
    }
    return rosbot_ekf::Configuration::Response::FAILURE;
}

uint8_t ConfigFunctionality::enableJointStates(const char *datain, const char **dataout) {
    int en;
    if (sscanf(datain, "%d", &en) == 1) {
        joint_states_enabled = (en == 0 ? false : true);
        return rosbot_ekf::Configuration::Response::SUCCESS;
    }
    return rosbot_ekf::Configuration::Response::FAILURE;
}

uint8_t ConfigFunctionality::enableDistanceSensors(const char *datain, const char **dataout) {
    int en;
    // if(sscanf(datain,"%d",&en) == 1)
    // {
    //     if(en == 0)
    //     {
    //         distance_sensors_enabled = false;
    //         for(int i=0;i<4;i++)
    //         {
    //             VL53L0X * sensor = distance_sensors->getSensor(i);
    //             sensor->stopContinuous();
    //         }
    //     }
    //     else
    //     {
    //         distance_sensors_enabled = true;
    //         for(int i=0;i<4;i++)
    //         {
    //             VL53L0X * sensor = distance_sensors->getSensor(i);
    //             sensor->setTimeout(50);
    //             sensor->startContinuous();
    //         }
    //     }
    //     return rosbot_ekf::Configuration::Response::SUCCESS;
    // }
    return rosbot_ekf::Configuration::Response::FAILURE;
}

uint8_t ConfigFunctionality::setLed(const char *datain, const char **dataout) {
    int led_num, led_state;
    if (sscanf(datain, "%d %d", &led_num, &led_state) == 2) {
        switch (led_num) {
            case 2:
                led2 = led_state;
                return rosbot_ekf::Configuration::Response::SUCCESS;
            case 3:
                led3 = led_state;
                return rosbot_ekf::Configuration::Response::SUCCESS;
            default:
                break;
        }
    }
    return rosbot_ekf::Configuration::Response::FAILURE;
}

uint8_t ConfigFunctionality::enableSpeedWatchdog(const char *datain, const char **dataout) {
    int en;
    if (sscanf(datain, "%d", &en) == 1) {
        is_speed_watchdog_enabled = (en == 0 ? false : true);
        return rosbot_ekf::Configuration::Response::SUCCESS;
    }
    return rosbot_ekf::Configuration::Response::FAILURE;
}

ConfigFunctionality *ConfigFunctionality::getInstance() {
    if (_instance == NULL) {
        _instance = new ConfigFunctionality();
    }
    return _instance;
}

// void responseCallback(const rosbot_ekf::Configuration::Request &req, rosbot_ekf::Configuration::Response &res)
// {
//     ConfigFunctionality *config_functionality = ConfigFunctionality::getInstance();
//     ConfigFunctionality::configuration_srv_fun_t fun = config_functionality->findFunctionality(req.command);
//     res.data = EMPTY_STRING;
//     if (fun != NULL)
//     {
//         // nh.loginfo("Command found!");
//         res.result = (config_functionality->*fun)(req.data, &res.data);
//     }
//     else
//     {
//         nh.loginfo("Command not found!");
//         res.result = rosbot_ekf::Configuration::Response::COMMAND_NOT_FOUND;
//     }
// }
