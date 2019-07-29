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
#include "vl53l0x_driver/vl53l0x.h"
#include "vl53l0x_platform.h"
#include <rr_hw_interface/gpio/mcp23017_gpio.hpp>

#define MCP_ADDRESS 0x20
#define VL53L0X_DEFAULT_ADDR 0x29
#define NUM_SENSORS 4
#define FIELD_OF_VIEW 0.436332
#define MIN_RANGE 0.03
#define MAX_RANGE 2.0

int VL53L0X_XSHUT_MCP23xx_IO[NUM_SENSORS];
int VL53L0X_ADDR[NUM_SENSORS];
VL53L0X_Dev_t Sensors[NUM_SENSORS];
VL53L0X_Dev_t* pSensors[NUM_SENSORS];
VL53L0X_RangingMeasurementData_t SensorsRangingMeasurementData[NUM_SENSORS];
vl53l0x_driver::vl53l0x sensor_msg_array[NUM_SENSORS];
int i2c_bus_instance;
std::string i2c_bus_path;
ros::Publisher sensor_pub_array[NUM_SENSORS];

void initialize()
{
    for (int i = 0; i < NUM_SENSORS; i++) {
        VL53L0X_XSHUT_MCP23xx_IO[i] = i;
        pSensors[i] = &Sensors[i];
        VL53L0X_ADDR[i] = (0x21 + i);
    }
}

uint32_t refSpadCount;
uint8_t isApertureSpads;
uint8_t VhvSettings;
uint8_t PhaseCal;
i2c* i2c_mcp23017;
i2c* i2c_vl53l0x;

int GPIO_Setup(rapyuta::McpGpio vlx1, rapyuta::McpGpio vlx2, rapyuta::McpGpio vlx3, rapyuta::McpGpio vlx4)
{
    ROS_INFO("Entered gpio setup");
    // i2c_mcp23017 = mcp23xx_init(i2c_bus_instance, MCP_ADDRESS);
    // if (i2c_mcp23017 == NULL)
    //   return -1;
    vlx1.pinmode(OUTPUT);
    ROS_INFO("gpio setup : First pinmode completed");
    vlx1.set(LOW);
    ROS_INFO("gpio setup : First pin set low completed");

    vlx2.pinmode(OUTPUT);
    ROS_INFO("gpio setup : second pinmode completed");
    vlx2.set(LOW);
    ROS_INFO("gpio setup : second pin set low completed");
    vlx3.pinmode(OUTPUT);
    vlx3.set(LOW);
    vlx4.pinmode(OUTPUT);
    vlx4.set(LOW);
    ROS_INFO("all pins gpio setup completed");
    // for (int i = 0; i < NUM_SENSORS; i++) {
    //   mcp_pinMode(i2c_mcp23017, VL53L0X_XSHUT_MCP23xx_IO[i], OUTPUT);
    //   mcp_digitalWrite(i2c_mcp23017, VL53L0X_XSHUT_MCP23xx_IO[i], LOW);
    // }
    ROS_INFO("Finished GPIO_Setup");
    return 0;
}

int Sensor_Setup(rapyuta::McpGpio vlx1, rapyuta::McpGpio vlx2, rapyuta::McpGpio vlx3, rapyuta::McpGpio vlx4)
{
    /* multi sensors init START */
    uint8_t addr_reg[2] = {0};
    addr_reg[0] = VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS;

    i2c_vl53l0x = libsoc_i2c_init(i2c_bus_instance, VL53L0X_DEFAULT_ADDR);
    if (i2c_vl53l0x == NULL)
         return -1;
    ROS_INFO("ready to start sensor setup");
    //mcp_digitalWrite(i2c_mcp23017, VL53L0X_XSHUT_MCP23xx_IO[0], HIGH);
    vlx1.set(1);
    addr_reg[1] = VL53L0X_ADDR[0];
    libsoc_i2c_write(i2c_vl53l0x, addr_reg, 2);
    ROS_INFO("CHECKPOINT1: Finished libsoc_i2c_write");
    pSensors[0]->I2cDevAddr = VL53L0X_ADDR[0];
    pSensors[0]->fd = VL53L0X_i2c_init((char*) i2c_bus_path.c_str(), pSensors[0]->I2cDevAddr);
    VL53L0X_DataInit(&Sensors[0]);
    VL53L0X_StaticInit(&Sensors[0]);
    VL53L0X_PerformRefCalibration(pSensors[0], &VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(pSensors[0], &refSpadCount, &isApertureSpads);
    
    ROS_INFO("first sensor setup completed");

    vlx2.set(HIGH);
    addr_reg[1] = VL53L0X_ADDR[1];
    libsoc_i2c_write(i2c_vl53l0x, addr_reg, 2);
    pSensors[1]->I2cDevAddr = VL53L0X_ADDR[1];
    pSensors[1]->fd = VL53L0X_i2c_init((char*) i2c_bus_path.c_str(), pSensors[1]->I2cDevAddr);
    VL53L0X_DataInit(&Sensors[1]);
    VL53L0X_StaticInit(&Sensors[1]);
    VL53L0X_PerformRefCalibration(pSensors[1], &VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(pSensors[1], &refSpadCount, &isApertureSpads);
    
    ROS_INFO("second sensor setup completed");

 
    vlx3.set(HIGH);
    addr_reg[1] = VL53L0X_ADDR[2];
    libsoc_i2c_write(i2c_vl53l0x, addr_reg, 2);
    pSensors[2]->I2cDevAddr = VL53L0X_ADDR[2];
    pSensors[2]->fd = VL53L0X_i2c_init((char*) i2c_bus_path.c_str(), pSensors[2]->I2cDevAddr);
    VL53L0X_DataInit(&Sensors[2]);
    VL53L0X_StaticInit(&Sensors[2]);
    VL53L0X_PerformRefCalibration(pSensors[2], &VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(pSensors[2], &refSpadCount, &isApertureSpads);
    
    ROS_INFO("third sensor setup completed");

 
    vlx4.set(HIGH);
    addr_reg[1] = VL53L0X_ADDR[3];
    libsoc_i2c_write(i2c_vl53l0x, addr_reg, 2);
    pSensors[3]->I2cDevAddr = VL53L0X_ADDR[3];
    pSensors[3]->fd = VL53L0X_i2c_init((char*) i2c_bus_path.c_str(), pSensors[3]->I2cDevAddr);
    VL53L0X_DataInit(&Sensors[3]);
    VL53L0X_StaticInit(&Sensors[3]);
    VL53L0X_PerformRefCalibration(pSensors[3], &VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(pSensors[3], &refSpadCount, &isApertureSpads);
    
    ROS_INFO("forth sensor setup completed");

 

    libsoc_i2c_free(i2c_vl53l0x);
    /* multi sensors init END */
    ROS_INFO("Finished Sensor_Setup");
    return 0;
}

void Sensor_Calibration(VL53L0X_Dev_t* pDevice)
{
    VL53L0X_SetDeviceMode(pDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    VL53L0X_SetLimitCheckEnable(pDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckEnable(pDevice, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckValue(pDevice, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1 * 65536));
    VL53L0X_SetLimitCheckValue(pDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60 * 65536));
    VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pDevice, 33000);
    VL53L0X_SetVcselPulsePeriod(pDevice, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    VL53L0X_SetVcselPulsePeriod(pDevice, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
}

int check_device_connection(int _file_descriptor)
{
    struct stat device_status;
    fstat(_file_descriptor, &device_status);
    return device_status.st_nlink;
}

void measure_and_publish(int sensor_num)
{
    while (ros::ok() and check_device_connection(pSensors[0]->fd)) {
        VL53L0X_PerformSingleRangingMeasurement(pSensors[sensor_num], &SensorsRangingMeasurementData[sensor_num]);
        sensor_msg_array[sensor_num].proximity = float(SensorsRangingMeasurementData[sensor_num].RangeMilliMeter) / 1000.0;
        sensor_msg_array[sensor_num].header.stamp = ros::Time::now();
        std::string frame = "sensor";
        sensor_msg_array[sensor_num].header.frame_id = frame + std::to_string(sensor_num + 1);
        sensor_msg_array[sensor_num].field_of_view = FIELD_OF_VIEW;
        sensor_msg_array[sensor_num].min_range = MIN_RANGE;
        sensor_msg_array[sensor_num].max_range = MAX_RANGE;
        sensor_pub_array[sensor_num].publish(sensor_msg_array[sensor_num]);
        ros::spinOnce();
    }
}

int main(int argc, char** argv)
{
    initialize();

    ros::init(argc, argv, "vl53l0x_driver");
    ros::NodeHandle nh;
    nh.getParam("/measure_proximity_node/i2c_bus_instance", i2c_bus_instance);
    ROS_INFO("i2c_bus_instance: %d", i2c_bus_instance);
    i2c_bus_path = "/dev/i2c-" + std::to_string(i2c_bus_instance);
    std::string name = "sensor_data_";
    std::string pin_name = "0";

    rapyuta::McpGpio vlx1(std::to_string(0), rapyuta::McpGpio::Type::RR_HW_INTERFACE_OUTPUT);
    rapyuta::McpGpio vlx2(std::to_string(1), rapyuta::McpGpio::Type::RR_HW_INTERFACE_OUTPUT);
    rapyuta::McpGpio vlx3(std::to_string(2), rapyuta::McpGpio::Type::RR_HW_INTERFACE_OUTPUT);
    rapyuta::McpGpio vlx4(std::to_string(3), rapyuta::McpGpio::Type::RR_HW_INTERFACE_OUTPUT);
    rapyuta::McpGpioBoardConfig config;
    ROS_INFO("i2c_bus_instance is %d",i2c_bus_instance);
    config.set_mcp_address(MCP_ADDRESS);
    config.set_i2c_bus_instance(i2c_bus_instance);
    vlx1.init(config);
    vlx2.init(config);
    vlx3.init(config);
    vlx4.init(config);

    ROS_INFO("Completed init from the vlxdriver code");

    if (GPIO_Setup(vlx1, vlx2, vlx3, vlx4) == 0) {
        if (Sensor_Setup(vlx1, vlx2, vlx3, vlx4) == 0) {
            for (int i = 0; i < NUM_SENSORS; i++) {
                std::string result = name + std::to_string(i + 1);
                sensor_pub_array[i] = nh.advertise<vl53l0x_driver::vl53l0x>(result, 10);
                Sensor_Calibration(pSensors[i]);
            }
            ros::AsyncSpinner spinner(NUM_SENSORS);
            spinner.start();
            for (int i = 0; i < NUM_SENSORS; i++) {
                boost::thread(boost::bind(measure_and_publish, i));
            }
            ros::waitForShutdown();
        } else
            ROS_INFO("Sensor Setup failed");
    } else
        ROS_INFO("GPIO Setup failed");

    VL53L0X_i2c_close();

    // Power-off the sensors
    // for (int i = 0; i < NUM_SENSORS; i++) {
    //   mcp_digitalWrite(i2c_mcp23017, VL53L0X_XSHUT_MCP23xx_IO[i], LOW);
    // }
    vlx1.set(LOW);
    vlx2.set(LOW);
    vlx3.set(LOW);
    vlx4.set(LOW);
    // mcp23xx_close(i2c_mcp23017);
    return (0);
}
