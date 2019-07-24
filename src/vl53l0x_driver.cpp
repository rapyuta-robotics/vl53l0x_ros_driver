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

#define MCP_ADDRESS 0x20
#define VL53L0X_DEFAULT_ADDR 0x29
#define NUM_SENSORS 4

int VL53L0X_XSHUT_MCP23xx_IO[NUM_SENSORS];
int VL53L0X_ADDR[NUM_SENSORS];
VL53L0X_Dev_t Sensors[NUM_SENSORS];
VL53L0X_Dev_t *pSensors[NUM_SENSORS];
VL53L0X_RangingMeasurementData_t SensorsRangingMeasurementData[NUM_SENSORS];
vl53l0x_driver::vl53l0x sensor_msg_array[NUM_SENSORS];
int i2c_bus_instance;
std::string i2c_bus_path;
ros::Publisher sensor_pub_array[NUM_SENSORS];

void initialize() {
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
i2c *i2c_mcp23017;
i2c *i2c_vl53l0x;

int GPIO_Setup() {
  i2c_mcp23017 = mcp23xx_init(i2c_bus_instance, MCP_ADDRESS);
  if (i2c_mcp23017 == NULL)
    return -1;

  for (int i = 0; i < NUM_SENSORS; i++) {
    mcp_pinMode(i2c_mcp23017, VL53L0X_XSHUT_MCP23xx_IO[i], OUTPUT);
    mcp_digitalWrite(i2c_mcp23017, VL53L0X_XSHUT_MCP23xx_IO[i], LOW);
  }
  ROS_INFO("Finished GPIO_Setup");
  return 0;
}

int Sensor_Setup() {
  /* multi sensors init START */
  uint8_t addr_reg[2] = {0};
  addr_reg[0] = VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS;

  i2c_vl53l0x = libsoc_i2c_init(i2c_bus_instance, VL53L0X_DEFAULT_ADDR);
  if (i2c_vl53l0x == NULL)
    return -1;
  for (int i = 0; i < NUM_SENSORS; i++) {
    mcp_digitalWrite(i2c_mcp23017, VL53L0X_XSHUT_MCP23xx_IO[i], HIGH);
    addr_reg[1] = VL53L0X_ADDR[i];
    libsoc_i2c_write(i2c_vl53l0x, addr_reg, 2);
    pSensors[i]->I2cDevAddr = VL53L0X_ADDR[i];
    pSensors[i]->fd =
        VL53L0X_i2c_init((char *)i2c_bus_path.c_str(), pSensors[i]->I2cDevAddr);
    VL53L0X_DataInit(&Sensors[i]);
    VL53L0X_StaticInit(&Sensors[i]);
    VL53L0X_PerformRefCalibration(pSensors[i], &VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(pSensors[i], &refSpadCount,
                                     &isApertureSpads);
  }

  libsoc_i2c_free(i2c_vl53l0x);
  /* multi sensors init END */
  ROS_INFO("Finished Sensor_Setup");
  return 0;
}

void Sensor_Calibration(VL53L0X_Dev_t *pDevice) {
  VL53L0X_SetDeviceMode(pDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING);
  VL53L0X_SetLimitCheckEnable(pDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                              1);
  VL53L0X_SetLimitCheckEnable(pDevice,
                              VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
  VL53L0X_SetLimitCheckValue(pDevice,
                             VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                             (FixPoint1616_t)(0.1 * 65536));
  VL53L0X_SetLimitCheckValue(pDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                             (FixPoint1616_t)(60 * 65536));
  VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pDevice, 33000);
  VL53L0X_SetVcselPulsePeriod(pDevice, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
  VL53L0X_SetVcselPulsePeriod(pDevice, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
}

int check_device_connection(int _file_descriptor) {
  struct stat device_status;
  fstat(_file_descriptor, &device_status);
  return device_status.st_nlink;
}

void thread1() {
  while (ros::ok()) {
    VL53L0X_PerformSingleRangingMeasurement(pSensors[0],
                                            &SensorsRangingMeasurementData[0]);
    sensor_msg_array[0].proximity =
        float(SensorsRangingMeasurementData[0].RangeMilliMeter) / 1000.0;
    sensor_msg_array[0].header.stamp = ros::Time::now();
    std::string frame = "sensor";
    sensor_msg_array[0].header.frame_id = frame + std::to_string(1);
    sensor_msg_array[0].field_of_view = 0.436332;
    sensor_msg_array[0].min_range = 0.03;
    sensor_msg_array[0].max_range = 2.0;
    sensor_pub_array[0].publish(sensor_msg_array[0]);
    ros::spinOnce();
  }
}

void thread2() {
  while (ros::ok()) {
    VL53L0X_PerformSingleRangingMeasurement(pSensors[1],
                                            &SensorsRangingMeasurementData[1]);
    sensor_msg_array[1].proximity =
        float(SensorsRangingMeasurementData[1].RangeMilliMeter) / 1000.0;
    sensor_msg_array[1].header.stamp = ros::Time::now();
    std::string frame = "sensor";
    sensor_msg_array[1].header.frame_id = frame + std::to_string(2);
    sensor_msg_array[1].field_of_view = 0.436332;
    sensor_msg_array[1].min_range = 0.03;
    sensor_msg_array[1].max_range = 2.0;
    sensor_pub_array[1].publish(sensor_msg_array[1]);
    ros::spinOnce();
  }
}

void thread3() {
  while (ros::ok()) {
    VL53L0X_PerformSingleRangingMeasurement(pSensors[2],
                                            &SensorsRangingMeasurementData[2]);
    sensor_msg_array[2].proximity =
        float(SensorsRangingMeasurementData[2].RangeMilliMeter) / 1000.0;
    sensor_msg_array[2].header.stamp = ros::Time::now();
    std::string frame = "sensor";
    sensor_msg_array[2].header.frame_id = frame + std::to_string(3);
    sensor_msg_array[2].field_of_view = 0.436332;
    sensor_msg_array[2].min_range = 0.03;
    sensor_msg_array[2].max_range = 2.0;
    sensor_pub_array[2].publish(sensor_msg_array[2]);
    ros::spinOnce();
  }
}

void thread4() {
  while (ros::ok()) {
    VL53L0X_PerformSingleRangingMeasurement(pSensors[3],
                                            &SensorsRangingMeasurementData[3]);
    sensor_msg_array[3].proximity =
        float(SensorsRangingMeasurementData[3].RangeMilliMeter) / 1000.0;
    sensor_msg_array[3].header.stamp = ros::Time::now();
    std::string frame = "frame_sensor";
    sensor_msg_array[3].header.frame_id = frame + std::to_string(4);
    sensor_msg_array[3].field_of_view = 0.436332;
    sensor_msg_array[3].min_range = 0.03;
    sensor_msg_array[3].max_range = 2.0;
    sensor_pub_array[3].publish(sensor_msg_array[3]);
    ros::spinOnce();
  }
}

int main(int argc, char **argv) {
  initialize();

  ros::init(argc, argv, "vl53l0x_driver");
  ros::NodeHandle nh;
  nh.getParam("/measure_proximity_node/i2c_bus_instance", i2c_bus_instance);
  ROS_INFO("i2c_bus_instance: %d", i2c_bus_instance);
  i2c_bus_path = "/dev/i2c-" + std::to_string(i2c_bus_instance);
  std::string name = "sensor_data_";

  if (GPIO_Setup() == 0) {
    if (Sensor_Setup() == 0) {
      for (int i = 0; i < NUM_SENSORS; i++) {
        std::string result = name + std::to_string(i + 1);
        sensor_pub_array[i] = nh.advertise<vl53l0x_driver::vl53l0x>(result, 10);
        Sensor_Calibration(pSensors[i]);
      }
      ros::AsyncSpinner spinner(4); // Use 4 threads
      spinner.start();
      boost::thread(boost::bind(thread1));
      boost::thread(boost::bind(thread2));
      boost::thread(boost::bind(thread3));
      boost::thread(boost::bind(thread4));
      ros::waitForShutdown();

    } else
      ROS_INFO("Sensor Setup failed");
  } else
    ROS_INFO("GPIO Setup failed");

  VL53L0X_i2c_close();

  // Power-off the sensors
  for (int i = 0; i < NUM_SENSORS; i++) {
    mcp_digitalWrite(i2c_mcp23017, VL53L0X_XSHUT_MCP23xx_IO[i], LOW);
  }
  mcp23xx_close(i2c_mcp23017);
  return (0);
}