
/**
  * @note 		STILL IN DEVELOPMENT
  * @file 		max3713.cpp
  * @author 	Simon Burkhardt github.com/mnemocron
  * @copyright 	MIT license
  * @date 		14 Jan 2018
  * @brief 		Object oriented C++ library for the MAX7313 port expander for STM32 HAL.
  * @details
  * @see 		github.com/mnemocron
  * @see 		https://datasheets.maximintegrated.com/en/ds/MAX7313.pdf
  * @see 		https://forum.arduino.cc/index.php?topic=9682.0
  *
  * @todo 		documentation of individual methods
  */

#include "max7313.hpp"
#include "main.h"

MAX7313::MAX7313(I2C_HandleTypeDef *wireIface, uint16_t address){
	this->wireIface = wireIface;
	this->devAddress = address;
	this->ioconfig[0] = 0xFF;			// default input / 1=IN / 0=OUT
	this->ioconfig[1] = 0xFF;			// default input / 1=IN / 0=OUT
	this->conf = 0x00;
	for(uint8_t i=0; i<sizeof(this->intensity); i++){
		this->intensity[i] = 0xff;
	}
}

uint8_t MAX7313::begin(){
	this->writeRegister(MAX7313_PORTS_CONF_00_07, this->ioconfig[0]);
	this->writeRegister(MAX7313_PORTS_CONF_08_15, this->ioconfig[1]);
	this->writeRegister(MAX7313_BLINK_PHASE_0_00_07, 0xff);
	this->writeRegister(MAX7313_BLINK_PHASE_0_08_15, 0xff);
	this->writeRegister(MAX7313_CONFIGURATION,       this->conf);
	this->writeRegister(MAX7313_OUT_INT_MA_16,       0xff);
	return 0;
}

/**
  * @important 		must be called before begin();
  */
uint8_t MAX7313::enableInterrupt(){
	this->conf |= 0x08;
	/** @Todo There are two possibilities to implement this method:
		*				- call this method before begin() => interrupts can only be enabled but not disabled
		*				- call this method anytime => must execute this->begin() inside inside here 
		*								=> possible to enable/disable interrupts on the fly 
		*/
	//this->begin();
	this->writeRegister(MAX7313_CONFIGURATION,       this->conf);
	return 0;
}

uint8_t MAX7313::writeRegister(uint8_t reg, uint8_t val){
	uint8_t buf[2];
	buf[0] = reg;
	buf[1] = val;
	HAL_I2C_Master_Transmit(this->wireIface, this->devAddress, &buf[0], 2, 10);
	/** @TODO: check for transmission error if(Transmit != HAL_OK) return error; */
	return 0;
}

uint8_t MAX7313::readRegister(uint8_t reg, uint8_t *val){
	uint8_t buf[2];
	HAL_I2C_Master_Transmit(this->wireIface, this->devAddress, &reg, 1, 10);
	HAL_I2C_Master_Receive(this->wireIface, this->devAddress, val, 1, 10);
	/** @TODO: check for transmission error if(Transmit != HAL_OK) return error; */
	return 0;
}

MAX7313Output::MAX7313Output(MAX7313 *chip, uint8_t port, uint8_t polarity=0){
	this->chip = chip;
	this->port = port;
	(this->port > 7) ? this->chip->ioconfig[1] &= ~(1<<(this->port%8)) : this->chip->ioconfig[0] &= ~(1<<(this->port));
	this->polarity = polarity;
	(this->polarity == 1) ? this->setIntensity(0xf) : this->setIntensity(0x0);
	if(port % 2)	// odd  numbered ports 1, 3, 5 ...
		this->regmask = 0x0F;
	else					// even numbered ports 0, 2, 3 ...
		this->regmask = 0xF0;
	switch(port){
		case 0:
		case 1:
			this->regmask = 0x0F;
			this->ioreg = MAX7313_OUT_INT_01_00;
			break;
		case 2:
		case 3:
			this->ioreg = MAX7313_OUT_INT_03_02;
			break;
		case 4:
		case 5:
			this->ioreg = MAX7313_OUT_INT_05_04;
			break;
		case 6:
		case 7:
			this->ioreg = MAX7313_OUT_INT_07_06;
			break;
		case 8:
		case 9:
			this->ioreg = MAX7313_OUT_INT_09_08;
			break;
		case 10:
		case 11:
			this->ioreg = MAX7313_OUT_INT_11_10;
			break;
		case 12:
		case 13:
			this->ioreg = MAX7313_OUT_INT_13_12;
			break;
		case 14:
		case 15:
			this->ioreg = MAX7313_OUT_INT_15_14;
			break;
		default:
			this->ioreg = MAX7313_NO_PORT;
			break;
	}
}

MAX7313Input::MAX7313Input(MAX7313 *chip, uint8_t port){
	this->chip = chip;
	this->port = port;
	(this->port > 7) ? this->chip->ioconfig[1] |= (1<<(this->port%8)) : this->chip->ioconfig[0] |= (1<<(this->port));
	this->regmask = (1<<(port%8));
	if(port < 7)
		this->ioreg = MAX7313_READ_IN_00_07;
	else if(port < 16)
		this->ioreg = MAX7313_READ_IN_08_15;
	else
		ioreg = MAX7313_NO_PORT;
}

uint8_t MAX7313Output::setIntensity(uint8_t intensity){
	if(intensity > 0x0F)
		intensity = 0x0F;
	//if(intensity < 0)			// unnecessary because unsigned int is never < 0
	//	intensity = 0;
	this->chip->intensity[this->port] = intensity;
	uint8_t val;
	/*										 [odd] [even]
	register in max7313:		xxxx xxxx
	intensity[even]         0000 xxxx										mask-> 0x0F
	intensity[odd]          xxxx 0000		shift->  <<4		mask-> 0xF0
	*/
	
	if(!polarity){			// polarity = 0
		if(this->port % 2) 		//  odd (1,3,5) => this + this-1  [MSB nibble] + [LSB nibble]
			val = 0xF0-((this->chip->intensity[this->port]<<4)&0xF0) + 0x0F-(this->chip->intensity[this->port-1]&0x0F);
		else									// even (0,2,4) => this+1 + this
			val = 0xF0-((this->chip->intensity[this->port+1]<<4)&0xF0) + 0x0F-(this->chip->intensity[this->port]&0x0F);
	} else {
		if(this->port % 2) 		//  odd (1,3,5) => this + this-1
			val = ((this->chip->intensity[this->port]<<4)&0xF0) + (this->chip->intensity[this->port-1]&0x0F);
		else									// even (0,2,4) => this+1 + this
			val = ((this->chip->intensity[this->port+1]<<4)&0xF0) + (this->chip->intensity[this->port]&0x0F);
	}
	
	this->chip->writeRegister(this->ioreg, val);
	return 0;
}

uint8_t MAX7313Input::read(){
	uint8_t ret = 0;
	this->chip->readRegister(this->ioreg, &ret);
	
	/** @Todo I haven't figured out how to return a bitmasked value
	  * but since the return value in this case is only either 0 or 1 it does not matter.
	  * the if statement below is a good solution.
	  */
	ret = ret&this->regmask;
	//ret = ret>>(this->port);
	if(ret) return 1;
	return 0;
}

