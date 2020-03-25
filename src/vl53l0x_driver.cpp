#include <boost/thread.hpp>
#include <chrono>
#include <cstring>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

#include "vl53l0x_driver/vl53l0x.h"

#include <vl53l0x_driver/vl53l0x_driver.hpp>

int i2c_bus_instance;
std::string i2c_bus_path;

uint32_t refSpadCount;
uint8_t isApertureSpads;
uint8_t VhvSettings;
uint8_t PhaseCal;

int i2c_vl53l0x;

int Sensor_Setup(rapyuta::Vl53l0x *sensors)
{
    /* multi sensors init START */
    uint8_t addr_reg[2] = {0};
    addr_reg[0] = VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS;
    i2c_vl53l0x = open(i2c_bus_path.c_str(), O_RDWR);
    if (i2c_vl53l0x < 0)
        return -1;
    ROS_INFO("Start sensor setup");

    
    if (sensors->init(addr_reg, i2c_vl53l0x, i2c_bus_path, refSpadCount, isApertureSpads, VhvSettings, PhaseCal) < 0) {
        ROS_FATAL("Setup Failure");
        return -1;
    }
    ROS_INFO("Setup Done");

    return 0;
}

void measure_and_publish(rapyuta::Vl53l0x* sensor)
{
    while (ros::ok() and sensor->checkDeviceConnection()) {
        sensor->get_and_pub();
        ros::spinOnce();
    }
    if (ros::ok()) {
        ros::shutdown();
    }
}

int get_i2c_bus(void)
{
    std::array<char, 128> buffer;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen("ls -l /dev/proximity_sensor | tail -c 2", "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    fgets(buffer.data(), buffer.size(), pipe.get());
    
    return buffer[0] - '0';
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vl53l0x_driver");
    ros::NodeHandle nh("~");

    int sensorNum = 0;
    nh.getParam("sensor_num", sensorNum);
    i2c_bus_instance = get_i2c_bus();
    
    ROS_INFO("i2c_bus_instance: %d", i2c_bus_instance);
    i2c_bus_path = "/dev/i2c-" + std::to_string(i2c_bus_instance);
    std::string topic_name = "sensor_data";
    rapyuta::Vl53l0x *sensors = new rapyuta::Vl53l0x(nh, topic_name);

    ROS_INFO("Completed init from the vlxdriver code");

    if (Sensor_Setup(sensors) == 0) {

        ROS_INFO("Start sensor calibration");
            if (sensors->sensorCalibration() < 0) {
                ROS_FATAL("Sensor Calibration Failure");
                return 0;
            }
        ROS_INFO("Finished sensor calibration");

        boost::thread(boost::bind(measure_and_publish, sensors));

        ROS_INFO("Start publishing sensor data");

        ros::waitForShutdown();
    } else {
        ROS_INFO("Sensor Setup failed");
    }

    VL53L0X_i2c_close();

    return 0;
}
