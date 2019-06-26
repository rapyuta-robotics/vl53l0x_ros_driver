#include <ros/ros.h>
#include "std_msgs/Int16.h"
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

extern "C" {  
    #include "mcp23017.h"  
}  

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

#include "vl53l0x_driver/getSensorData.h"
#include "vl53l0x_driver/vl53l0x.h"
 
#define MCP_ADDRESS 0x20
#define I2C_BUS_PATH   "/dev/i2c-8"
#define I2C_BUS_INSTANCE   8

#define VL53L0X_1_XSHUT_MCP23xx_IO  0
#define VL53L0X_2_XSHUT_MCP23xx_IO  1
#define VL53L0X_3_XSHUT_MCP23xx_IO  2
#define VL53L0X_4_XSHUT_MCP23xx_IO  3

#define VL53L0X_DEFAULT_ADDR  0x29
#define VL53L0X_1_ADDR  0x21
#define VL53L0X_2_ADDR  0x22
#define VL53L0X_3_ADDR  0x23
#define VL53L0X_4_ADDR  0x24

VL53L0X_Dev_t SensorOne;
VL53L0X_Dev_t SensorTwo;
VL53L0X_Dev_t SensorThree;
VL53L0X_Dev_t SensorFour;

VL53L0X_Dev_t *pSensorOne = &SensorOne;
VL53L0X_Dev_t *pSensorTwo = &SensorTwo;
VL53L0X_Dev_t *pSensorThree = &SensorThree;
VL53L0X_Dev_t *pSensorFour = &SensorFour;

VL53L0X_RangingMeasurementData_t    SensorOneRangingMeasurementData;
VL53L0X_RangingMeasurementData_t    SensorTwoRangingMeasurementData;
VL53L0X_RangingMeasurementData_t    SensorThreeRangingMeasurementData;
VL53L0X_RangingMeasurementData_t    SensorFourRangingMeasurementData;

uint32_t refSpadCount;
uint8_t isApertureSpads;
uint8_t VhvSettings;
uint8_t PhaseCal;

vl53l0x_driver::vl53l0x sensorData;
i2c *i2c_mcp23017;
i2c *i2c_vl53l0x;

bool getSensorData(vl53l0x_driver::getSensorData::Request &req, vl53l0x_driver::getSensorData::Response &res) {
    res.One = SensorOneRangingMeasurementData.RangeMilliMeter;
    res.Two = SensorTwoRangingMeasurementData.RangeMilliMeter;
    res.Three = SensorThreeRangingMeasurementData.RangeMilliMeter;
    res.Four = SensorFourRangingMeasurementData.RangeMilliMeter;
    return true;
}

void GPIO_Setup() {
    i2c_mcp23017 = mcp23xx_init(I2C_BUS_INSTANCE, MCP_ADDRESS);
	if (i2c_mcp23017 == NULL)
		return;

    mcp_pinMode(i2c_mcp23017, VL53L0X_1_XSHUT_MCP23xx_IO, OUTPUT);
    mcp_pinMode(i2c_mcp23017, VL53L0X_2_XSHUT_MCP23xx_IO, OUTPUT);
    mcp_pinMode(i2c_mcp23017, VL53L0X_3_XSHUT_MCP23xx_IO, OUTPUT);
    mcp_pinMode(i2c_mcp23017, VL53L0X_4_XSHUT_MCP23xx_IO, OUTPUT);

    mcp_digitalWrite(i2c_mcp23017, VL53L0X_1_XSHUT_MCP23xx_IO, LOW);
    mcp_digitalWrite(i2c_mcp23017, VL53L0X_2_XSHUT_MCP23xx_IO, LOW);
    mcp_digitalWrite(i2c_mcp23017, VL53L0X_3_XSHUT_MCP23xx_IO, LOW);
    mcp_digitalWrite(i2c_mcp23017, VL53L0X_4_XSHUT_MCP23xx_IO, LOW);
}

void Sensor_Setup() {
/* multi sensors init START */
    uint8_t addr_reg[2] = {0};
    addr_reg[0] = VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS;

    i2c_vl53l0x = libsoc_i2c_init(I2C_BUS_INSTANCE, VL53L0X_DEFAULT_ADDR);

    mcp_digitalWrite(i2c_mcp23017, VL53L0X_1_XSHUT_MCP23xx_IO, HIGH);
    addr_reg[1] = VL53L0X_1_ADDR;
    libsoc_i2c_write(i2c_vl53l0x, addr_reg, 2);
    pSensorOne->I2cDevAddr      = VL53L0X_1_ADDR;
    pSensorOne->fd = VL53L0X_i2c_init((char*)I2C_BUS_PATH, pSensorOne->I2cDevAddr);
    VL53L0X_DataInit(&SensorOne);
    VL53L0X_StaticInit(&SensorOne);
    VL53L0X_PerformRefCalibration(pSensorOne, &VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(pSensorOne, &refSpadCount, &isApertureSpads);

    mcp_digitalWrite(i2c_mcp23017, VL53L0X_2_XSHUT_MCP23xx_IO, HIGH);
    addr_reg[1] = VL53L0X_2_ADDR;
    libsoc_i2c_write(i2c_vl53l0x, addr_reg, 2);
    //usleep(sleeptime);
    pSensorTwo->I2cDevAddr      = VL53L0X_2_ADDR;
    pSensorTwo->fd = VL53L0X_i2c_init((char*)I2C_BUS_PATH, pSensorTwo->I2cDevAddr);
    VL53L0X_DataInit(&SensorTwo);
    VL53L0X_StaticInit(&SensorTwo);
    VL53L0X_PerformRefCalibration(pSensorTwo, &VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(pSensorTwo, &refSpadCount, &isApertureSpads);

    mcp_digitalWrite(i2c_mcp23017, VL53L0X_3_XSHUT_MCP23xx_IO, HIGH);
    addr_reg[1] = VL53L0X_3_ADDR;
    libsoc_i2c_write(i2c_vl53l0x, addr_reg, 2);
    pSensorThree->I2cDevAddr      = VL53L0X_3_ADDR;
    pSensorThree->fd = VL53L0X_i2c_init((char*)I2C_BUS_PATH, pSensorThree->I2cDevAddr);
    VL53L0X_DataInit(&SensorThree);
    VL53L0X_StaticInit(&SensorThree);
    VL53L0X_PerformRefCalibration(pSensorThree, &VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(pSensorThree, &refSpadCount, &isApertureSpads);

    mcp_digitalWrite(i2c_mcp23017, VL53L0X_4_XSHUT_MCP23xx_IO, HIGH);
    addr_reg[1] = VL53L0X_4_ADDR;
    libsoc_i2c_write(i2c_vl53l0x, addr_reg, 2);
    pSensorFour->I2cDevAddr = VL53L0X_4_ADDR;
    pSensorFour->fd = VL53L0X_i2c_init((char*)I2C_BUS_PATH, pSensorFour->I2cDevAddr);
    VL53L0X_DataInit(&SensorFour);
    VL53L0X_StaticInit(&SensorFour);
    VL53L0X_PerformRefCalibration(pSensorFour, &VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(pSensorFour, &refSpadCount, &isApertureSpads);
   
    libsoc_i2c_free(i2c_vl53l0x);
/* multi sensors init END */
}

void Sensor_Calibration(VL53L0X_Dev_t *pDevice) {
    VL53L0X_SetDeviceMode(pDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    VL53L0X_SetLimitCheckEnable(pDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckEnable(pDevice, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckValue(pDevice, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1 * 65536));
    VL53L0X_SetLimitCheckValue(pDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60 * 65536));
    VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pDevice, 33000);
    VL53L0X_SetVcselPulsePeriod(pDevice, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    VL53L0X_SetVcselPulsePeriod(pDevice, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
}

void Start_Ranging() {
    VL53L0X_PerformSingleRangingMeasurement(pSensorOne,	&SensorOneRangingMeasurementData);
    VL53L0X_PerformSingleRangingMeasurement(pSensorTwo,	&SensorTwoRangingMeasurementData);
    VL53L0X_PerformSingleRangingMeasurement(pSensorThree, &SensorThreeRangingMeasurementData);
    VL53L0X_PerformSingleRangingMeasurement(pSensorFour, &SensorFourRangingMeasurementData);

    sensorData.One = SensorOneRangingMeasurementData.RangeMilliMeter;
    sensorData.Two = SensorTwoRangingMeasurementData.RangeMilliMeter;
    sensorData.Three = SensorThreeRangingMeasurementData.RangeMilliMeter;
    sensorData.Four = SensorFourRangingMeasurementData.RangeMilliMeter;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vl53l0x_driver");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<vl53l0x_driver::vl53l0x>("/vl53l0x_sensors_data", 1);

    GPIO_Setup();
    Sensor_Setup();

    //Step 4 :: Calibration Sensor and Get Sensor Value 
    Sensor_Calibration(pSensorOne);
    Sensor_Calibration(pSensorTwo);
    Sensor_Calibration(pSensorThree);
    Sensor_Calibration(pSensorFour);

    while (ros::ok()) {
        Start_Ranging();
        pub.publish(sensorData);
        ros::spinOnce();
    }

    VL53L0X_i2c_close();

    // Power-off the sensors
    mcp_digitalWrite(i2c_mcp23017, VL53L0X_1_XSHUT_MCP23xx_IO, LOW);
    mcp_digitalWrite(i2c_mcp23017, VL53L0X_2_XSHUT_MCP23xx_IO, LOW);
    mcp_digitalWrite(i2c_mcp23017, VL53L0X_3_XSHUT_MCP23xx_IO, LOW);
    mcp_digitalWrite(i2c_mcp23017, VL53L0X_4_XSHUT_MCP23xx_IO, LOW);
    mcp23xx_close(i2c_mcp23017);
    return (0);
}

