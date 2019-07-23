#include <cstring>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <stdio.h>
#include <stdlib.h>
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

void initialize() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    VL53L0X_XSHUT_MCP23xx_IO[i] = i;
    pSensors[i] = &Sensors[i];
  }
  for (int i = 0, j = 0x21; i < NUM_SENSORS; j++, i++)
    VL53L0X_ADDR[i] = j;
}

uint32_t refSpadCount;
uint8_t isApertureSpads;
uint8_t VhvSettings;
uint8_t PhaseCal;
i2c *i2c_mcp23017;
i2c *i2c_vl53l0x;

void GPIO_Setup() {
  i2c_mcp23017 = mcp23xx_init(i2c_bus_instance, MCP_ADDRESS);
  if (i2c_mcp23017 == NULL)
    return;

  for (int i = 0; i < NUM_SENSORS; i++) {
    mcp_pinMode(i2c_mcp23017, VL53L0X_XSHUT_MCP23xx_IO[i], OUTPUT);
    mcp_digitalWrite(i2c_mcp23017, VL53L0X_XSHUT_MCP23xx_IO[i], LOW);
  }
}

void Sensor_Setup() {
  /* multi sensors init START */
  uint8_t addr_reg[2] = {0};
  addr_reg[0] = VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS;

  i2c_vl53l0x = libsoc_i2c_init(i2c_bus_instance, VL53L0X_DEFAULT_ADDR);

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

void Start_Ranging(int i) {

  VL53L0X_PerformSingleRangingMeasurement(pSensors[i],
                                          &SensorsRangingMeasurementData[i]);
  sensor_msg_array[i].proximity =
      float(SensorsRangingMeasurementData[i].RangeMilliMeter)/1000.0;
  sensor_msg_array[i].header.stamp = ros::Time::now();
  std::string frame = "frame_sensor";
  sensor_msg_array[i].header.frame_id = frame + std::to_string(i + 1);
  sensor_msg_array[i].field_of_view = 0.436332;
  sensor_msg_array[i].min_range = 0.03;
  sensor_msg_array[i].max_range = 2.0;
}

int main(int argc, char **argv) {
  initialize();

  ros::init(argc, argv, "vl53l0x_driver");
  ros::NodeHandle nh;
  nh.getParam("/measure_proximity_node/i2c_bus_instance", i2c_bus_instance);
  ROS_INFO("i2c_bus_instance: %d", i2c_bus_instance);
  i2c_bus_path = "/dev/i2c-" + std::to_string(i2c_bus_instance);

  ros::Publisher sensor_pub_array[NUM_SENSORS];
  std::string name = "sensor_data_";
  for (int i = 0; i < NUM_SENSORS; i++) {
    std::string result = name + std::to_string(i + 1);
    sensor_pub_array[i] = nh.advertise<vl53l0x_driver::vl53l0x>(result, 1000);
  }

  GPIO_Setup();
  Sensor_Setup();

  // Step 4 :: Calibration Sensor and Get Sensor Value
  for (int i = 0; i < NUM_SENSORS; i++) {
    Sensor_Calibration(pSensors[i]);
  }
  while (ros::ok()) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      Start_Ranging(i);
      sensor_pub_array[i].publish(sensor_msg_array[i]);
      ros::spinOnce();
    }
  }

  VL53L0X_i2c_close();

  // Power-off the sensors
  for (int i = 0; i < NUM_SENSORS; i++) {
    mcp_digitalWrite(i2c_mcp23017, VL53L0X_XSHUT_MCP23xx_IO[i], LOW);
  }
  mcp23xx_close(i2c_mcp23017);
  return (0);
}