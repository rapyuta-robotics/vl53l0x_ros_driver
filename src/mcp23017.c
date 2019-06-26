#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

#include "mcp23017.h"

/**
 * Bit number associated to a give Pin
 */
static uint8_t bitForPin(uint8_t pin) {
	return pin%8;
}

/**
 * Register address, port dependent, for a given PIN
 */
static uint8_t regForPin(uint8_t pin, uint8_t portAaddr, uint8_t portBaddr){
	return(pin < 8) ?portAaddr:portBaddr;
}

/**
 * Reads a given register
 */
static uint8_t readRegister(i2c *i2c, uint8_t regAddr){
	// Read the register
	uint8_t res;

    libsoc_i2c_write(i2c, &regAddr, 1);
	if (libsoc_i2c_read(i2c, &res, 1))
		return 0;
	return res;
}

/**
 * Writes a given register
 */
static void writeRegister(i2c *i2c, uint8_t regAddr, uint8_t regValue){
	// Write the register
    uint8_t buf[2] = {0};
    buf[0] = regAddr;
    buf[1] = regValue;
    libsoc_i2c_write(i2c, buf, 2);
}

/**
 * Helper to update a single bit of an A/B register.
 * - Reads the current register value
 * - Writes the new register value
 */
static void updateRegisterBit(i2c *i2c, uint8_t pin, uint8_t pValue, uint8_t portAaddr, uint8_t portBaddr) {
	uint8_t regValue;
	uint8_t regAddr = regForPin(pin, portAaddr, portBaddr);
	uint8_t bit = bitForPin(pin);

	regValue = readRegister(i2c, regAddr);
	// set the value for the particular bit
	bitWrite(regValue, bit, pValue);
	writeRegister(i2c, regAddr, regValue);
}

////////////////////////////////////////////////////////////////////////////////
// PUBLIC routines

i2c* mcp23xx_init(uint8_t i2c_bus, uint8_t i2c_address) {
	i2c *i2c = libsoc_i2c_init(i2c_bus, i2c_address);
	if (i2c == NULL) {
		perror("libsoc_i2c_init failed");
		return i2c;
	}
    mcp_begin(i2c);
	return i2c;
}

void mcp23xx_close(i2c* i2c) {
    libsoc_i2c_free(i2c);
}

void mcp_begin(i2c *i2c) {
	// set defaults!
	// all inputs on port A and B
	writeRegister(i2c, MCP23017_IODIRA, 0xFF);
	writeRegister(i2c, MCP23017_IODIRB, 0xFF);
}

void mcp_pinMode(i2c *i2c, uint8_t p, uint8_t d) {
	updateRegisterBit(i2c, p, (d == INPUT), MCP23017_IODIRA, MCP23017_IODIRB);
}

uint8_t mcp_readGPIO(i2c *i2c, uint8_t b) {
	// read the current GPIO output latches
    return readRegister(i2c, (b == 0)?MCP23017_GPIOA:MCP23017_GPIOB);
}

void mcp_writeGPIOAB(i2c *i2c, uint16_t writeValue) {
        uint8_t val[2] = {0};

        val[0] = writeValue & 0xff;
        val[1] = (writeValue >> 8);
        libsoc_i2c_write(i2c, val, 2);
}

uint16_t mcp_readGPIOAB(i2c *i2c) {
    uint16_t readValue;
    uint8_t val[2] = {0};
    val[0] = MCP23017_GPIOA;
	// read the current GPIO output latches
    libsoc_i2c_write(i2c, val, 1);
	if (libsoc_i2c_read(i2c, val, 2))
		return 0;
   
	//readValue = wiringPiI2CReadReg16(deviceFd, MCP23017_GPIOA);
	readValue = (val[1] & 0xFF00) << 8 | ((val[0] & 0x00FF));

	return readValue;
}

void mcp_digitalWrite(i2c *i2c, uint8_t pin, uint8_t d) {
	uint8_t gpio;
	uint8_t bit = bitForPin(pin);


	// read the current GPIO output latches
	uint8_t regAddr = regForPin(pin, MCP23017_OLATA, MCP23017_OLATB);
	gpio = readRegister(i2c, regAddr);

	// set the pin and direction
	bitWrite(gpio, bit, d);

	// write the new GPIO
	regAddr = regForPin(pin, MCP23017_GPIOA, MCP23017_GPIOB);
	writeRegister(i2c, regAddr, gpio);
}

void mcp_pullUp(i2c *i2c, uint8_t p, uint8_t d) {
	updateRegisterBit(i2c, p, d, MCP23017_GPPUA, MCP23017_GPPUB);
}

uint8_t mcp_digitalRead(i2c *i2c, uint8_t pin) {
	uint8_t bit = bitForPin(pin);
	uint8_t regAddr = regForPin(pin, MCP23017_GPIOA, MCP23017_GPIOB);
	return (readRegister(i2c, regAddr) >> bit) & 0x1;
}

/**
 * Configures the interrupt system. both port A and B are assigned the same configuration.
 * Mirroring will OR both INTA and INTB pins.
 * Opendrain will set the INT pin to value or open drain.
 * polarity will set LOW or HIGH on interrupt.
 * Default values after Power On Reset are: (false,flase, LOW)
 * If you are connecting the INTA/B pin to arduino 2/3, you should configure the interupt handling as FALLING with
 * the default configuration.
 */
void mcp_setupInterrupts(i2c *i2c, uint8_t mirroring, uint8_t openDrain, uint8_t polarity){
	// configure the port A
	uint8_t ioconfValue = readRegister(i2c, MCP23017_IOCONA);
	bitWrite(ioconfValue, 6, mirroring);
	bitWrite(ioconfValue, 2, openDrain);
	bitWrite(ioconfValue, 1, polarity);
	writeRegister(i2c, MCP23017_IOCONA, ioconfValue);

	// Configure the port B
	ioconfValue = readRegister(i2c, MCP23017_IOCONB);
	bitWrite(ioconfValue, 6, mirroring);
	bitWrite(ioconfValue, 2, openDrain);
	bitWrite(ioconfValue, 1, polarity);
	writeRegister(i2c, MCP23017_IOCONB, ioconfValue);
}

/**
 * Set's up a pin for interrupt. uses arduino MODEs: CHANGE, FALLING, RISING.
 *
 * Note that the interrupt condition finishes when you read the information about the port / value
 * that caused the interrupt or you read the port itself. Check the datasheet can be confusing.
 *
 */
void mcp_setupInterruptPin(i2c *i2c, uint8_t pin, uint8_t mode) {

	// set the pin interrupt control (0 means change, 1 means compare against given value);
	updateRegisterBit(i2c, pin, (mode != CHANGE), MCP23017_INTCONA, MCP23017_INTCONB);
	// if the mode is not CHANGE, we need to set up a default value, different value triggers interrupt

	// In a RISING interrupt the default value is 0, interrupt is triggered when the pin goes to 1.
	// In a FALLING interrupt the default value is 1, interrupt is triggered when pin goes to 0.
	updateRegisterBit(i2c, pin, (mode == FALLING), MCP23017_DEFVALA, MCP23017_DEFVALB);

	// enable the pin for interrupt
	updateRegisterBit(i2c, pin, HIGH, MCP23017_GPINTENA, MCP23017_GPINTENB);

}

uint8_t mcp_getLastInterruptPin(i2c *i2c){
	uint8_t intf, i;

	// try port A
	intf=readRegister(i2c, MCP23017_INTFA);
	for(i = 0; i < 8; i++) if (bitRead(intf, i)) return i;

	// try port B
	intf = readRegister(i2c, MCP23017_INTFB);
	for(i = 0; i < 8; i++) if (bitRead(intf, i)) return i + 8;

	return MCP23017_INT_ERR;
}

uint8_t mcp_getLastInterruptPinValue(i2c *i2c){
	uint8_t intPin = 0; /* TODO */
	if(intPin != MCP23017_INT_ERR){
		uint8_t intcapreg = regForPin(intPin, MCP23017_INTCAPA, MCP23017_INTCAPB);
		uint8_t bit = bitForPin(intPin);
		return (readRegister(i2c, intcapreg)>>bit) & (0x01);
	}

	return MCP23017_INT_ERR;
}
