#pragma once

#include <boost/thread.hpp>
#include <chrono>
#include <cstring>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>              //Needed for I2C port
#include <sys/ioctl.h>          //Needed for I2C port
#include <linux/i2c-dev.h>      //Needed for I2C port

// extern "C" {
// #include "mcp23017.h"
// }
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

#include "vl53l0x_driver/vl53l0x.h"
// #include <rr_hw_interface/gpio/mcp23017_gpio.hpp>

#define FIELD_OF_VIEW 0.436332
#define MIN_RANGE 0.03
#define MAX_RANGE 1.2
#define VL53L0X_DEFAULT_ADDR 0x29
namespace rapyuta
{

/*
    vl53l0x interface which have trigger and status gpio class
*/
class Vl53l0x
{
public:
    Vl53l0x(ros::NodeHandle n, std::string topic_name)
            : _vl53l0xAddr(0x29)
    {
        _pSensor = &_sensor;
        _pub = n.advertise<vl53l0x_driver::vl53l0x>(topic_name, 10);

        std::string frame = "proximity_sensor";
        _msg.header.frame_id = frame;
        _msg.field_of_view = FIELD_OF_VIEW;
        _msg.min_range = MIN_RANGE;
        _msg.max_range = MAX_RANGE;
    };
    ~Vl53l0x() { 
    }

    int8_t init(uint8_t* addr_reg, int i2c_vl53l0x, std::string i2c_bus_path, uint32_t refSpadCount, uint8_t isApertureSpads, uint8_t VhvSettings,
            uint8_t PhaseCal)
    {
        // if(!_i2c.init(_config)){
        //     return -1;
        // }
        // _i2c.set(LOW);
        // _i2c.set(HIGH);

        addr_reg[1] = _vl53l0xAddr;

        if (ioctl(i2c_vl53l0x, I2C_SLAVE, VL53L0X_DEFAULT_ADDR) < 0)
        {
            printf("Failed to acquire bus access and/or talk to Vl53l0x.\n");
            return -1;
        }
        
        write(i2c_vl53l0x, addr_reg, 2);
        _pSensor->I2cDevAddr = _vl53l0xAddr;
        _pSensor->fd = VL53L0X_i2c_init((char*) i2c_bus_path.c_str(), _pSensor->I2cDevAddr);

        int8_t result = 0;
        result += VL53L0X_DataInit(_pSensor);
        result += VL53L0X_StaticInit(_pSensor);
        result += VL53L0X_PerformRefCalibration(_pSensor, &VhvSettings, &PhaseCal);
        result += VL53L0X_PerformRefSpadManagement(_pSensor, &refSpadCount, &isApertureSpads);

        return result;
    };

    int8_t sensorCalibration()
    {
        VL53L0X_Error Status = VL53L0X_ERROR_NONE;

        Status = VL53L0X_SetDeviceMode(_pSensor, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode

        // Enable/Disable Sigma and Signal check
        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_SetLimitCheckEnable(_pSensor,
                    VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
        }
        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_SetLimitCheckEnable(_pSensor,
                    VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
        }
                    
        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_SetLimitCheckValue(_pSensor,
                    VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                    (FixPoint1616_t)(0.25*65536));
        }           
        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_SetLimitCheckValue(_pSensor,
                    VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                    (FixPoint1616_t)(18*65536));            
        }
        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(_pSensor,
                    200000);
        }

        return (Status != VL53L0X_ERROR_NONE);
    }

    int checkDeviceConnection()
    {
        struct stat device_status;
        fstat(_pSensor->fd, &device_status);
        return device_status.st_nlink;
    }

    vl53l0x_driver::vl53l0x get()
    {

        VL53L0X_PerformSingleRangingMeasurement(_pSensor, &_sensorsRangingMeasurementData);
        _msg.proximity = float(_sensorsRangingMeasurementData.RangeMilliMeter) / 1000.0;
        _msg.header.stamp = ros::Time::now();
        return _msg;
    };

    float pub() { _pub.publish(_msg); }

    float get_and_pub()
    {
        get();
        pub();
    }

private:
    ros::Publisher _pub;
    int _vl53l0xAddr;
    VL53L0X_Dev_t _sensor;
    VL53L0X_Dev_t* _pSensor;
    VL53L0X_RangingMeasurementData_t _sensorsRangingMeasurementData;
    vl53l0x_driver::vl53l0x _msg;
};

} // namespace rapyuta