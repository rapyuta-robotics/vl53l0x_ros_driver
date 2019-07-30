#include <boost/thread.hpp>
#include <chrono>
#include <cstring>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>

extern "C" {
#include "mcp23017.h"
}
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

#include <rr_hw_interface/gpio/mcp23017_gpio.hpp>
#include <vl53l0x_driver/vl53l0x_driver.hpp>
#include "vl53l0x_driver/vl53l0x.h"

#define MCP_ADDRESS 0x20
#define VL53L0X_DEFAULT_ADDR 0x29

int i2c_bus_instance;
std::string i2c_bus_path;

uint32_t refSpadCount;
uint8_t isApertureSpads;
uint8_t VhvSettings;
uint8_t PhaseCal;
i2c* i2c_mcp23017;
i2c* i2c_vl53l0x;

int Sensor_Setup(std::vector<rapyuta::Vl53l0x<rapyuta::McpGpio, rapyuta::McpGpioBoardConfig>*> sensors)
{
    /* multi sensors init START */
    uint8_t addr_reg[2] = {0};
    addr_reg[0] = VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS;

    i2c_vl53l0x = libsoc_i2c_init(i2c_bus_instance, VL53L0X_DEFAULT_ADDR);
    if (i2c_vl53l0x == NULL)
       return -1;
    ROS_INFO("ready to start sensor setup");

    for (auto itr: sensors) {
      itr->init(addr_reg, i2c_vl53l0x, i2c_bus_path, 
       refSpadCount, isApertureSpads, VhvSettings, PhaseCal);
    } 

    libsoc_i2c_free(i2c_vl53l0x);
    /* multi sensors init END */
    ROS_INFO("Finished Sensor_Setup");
    return 0;
}

void measure_and_publish(rapyuta::Vl53l0x<rapyuta::McpGpio, rapyuta::McpGpioBoardConfig>* sensor)
{
    while (ros::ok() and sensor->checkDeviceConnection()) {
        sensor->get_and_pub();
        ros::spinOnce();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vl53l0x_driver");
    ros::NodeHandle nh("~");

    int sensorNum = 0;
    nh.getParam("sensor_num", sensorNum);
    nh.getParam("i2c_bus_instance", i2c_bus_instance);
    ROS_INFO("i2c_bus_instance: %d", i2c_bus_instance);
    i2c_bus_path = "/dev/i2c-" + std::to_string(i2c_bus_instance);
    std::string topic_name = "sensor_data_";

    rapyuta::McpGpioBoardConfig config(i2c_bus_instance, MCP_ADDRESS);

    std::vector<rapyuta::Vl53l0x<rapyuta::McpGpio, rapyuta::McpGpioBoardConfig>*> sensors;
    for (int i = 0; i < sensorNum; i++) {
        sensors.push_back(new rapyuta::Vl53l0x<rapyuta::McpGpio, rapyuta::McpGpioBoardConfig>(
          i, nh, topic_name+std::to_string(i+1), config));
    }

    ROS_INFO("Completed init from the vlxdriver code");

    if (Sensor_Setup(sensors) == 0) {
        for (auto itr: sensors) {
          itr->sensorCalibration();
        }
        ros::AsyncSpinner spinner(sensorNum);
        spinner.start();
        for (int i = 0; i < sensorNum; i++) {
            boost::thread(boost::bind(measure_and_publish, sensors[i]));
        }

        ros::waitForShutdown();
    } else{
        ROS_INFO("Sensor Setup failed");
    }

    VL53L0X_i2c_close();

    return (0);
}
