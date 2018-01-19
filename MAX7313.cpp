
/**
  * @note 		STILL IN DEVELOPMENT
  * @file 		max3713.cpp
  * @author 	Simon Burkhardt github.com/mnemocron
  * @copyright 	MIT license
  * @date 		19 Jan 2018
  * @brief 		Object oriented C++ library for the MAX7313 port expander for STM32 HAL.
  * @details 	
  * @see 		github.com/mnemocron
  * @see 		https://datasheets.maximintegrated.com/en/ds/MAX7313.pdf
  * @see 		https://forum.arduino.cc/index.php?topic=9682.0
  */

#include "max7313.h"
#include "main.h"

/**
  * @brief 		MAX7313 constructor
  * @param 		*wireIface a pointer to a HAL I2C_HandleTypeDef
  * @param 		address of the chip on the I2C bus
  */
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

/**
  * @brief 		Initalizer function
  * @return 	0 on success, 1 on I2C transmission error
  * @pre 		all Inputs & Outputs must be declared before calling begin()
  */
uint8_t MAX7313::begin(){
	uint8_t ret = 0;
	ret += this->write8(MAX7313_PORTS_CONF_00_07,    this->ioconfig[0]);
	ret += this->write8(MAX7313_PORTS_CONF_08_15,    this->ioconfig[1]);
	ret += this->write8(MAX7313_BLINK_PHASE_0_00_07, 0xff);
	ret += this->write8(MAX7313_BLINK_PHASE_0_08_15, 0xff);
	ret += this->write8(MAX7313_CONFIGURATION,       this->conf);
	ret += this->write8(MAX7313_OUT_INT_MA_16,       0xff);
	if(ret)
		return 1;
	return 0;
}

/**
  * @brief 		enables data change interrupt on !INT/O16 pin
  * @return 	0 on success, 1 on I2C transmission error
  * @see 		MAX7313 datasheet p.7 / p.13 - 15
  * @important 	The MAX7313 signals a data change interrupt with a falling edge on the !INT/O16 pin
  *           	and will stay in LOW state as long as the interrput flag is not reset by reading input registers.
  *           	You can use MAX7313::clearInterrupt()
  */
uint8_t MAX7313::enableInterrupt(){
	this->conf |= 0x08;
	if(this->write8(MAX7313_CONFIGURATION, this->conf))
		return 1;
	if(this->clearInterrupt())
		return 1;
	return 0;
}

/**
  * @brief 		disables data change interrupt on !INT/O16 pin
  * @return 	0 on success, 1 on I2C transmission error
  * @see 		MAX7313 datasheet p.7 / p.13 - 15
  */
uint8_t MAX7313::disableInterrupt(){
	this->conf &= 0xF7;			// clear bit 3
	if(this->write8(MAX7313_CONFIGURATION, this->conf))
		return 1;
	return 0;
}

/**
  * @brief 		clear/validate the data change interrupt
  * @details 	This method is used to clear an interrupt condition (!INT/O16 pin = LOW)
  * 			by reading the input registers. The MAX7313 will pull the !INT/O16 pin HIGH
  * 			and will be armed for another interrupt event.
  * @return 	0 on success, 1 on I2C transmission error, 2 if interrupts are not enabled
  * @see 		https://github.com/mnemocron/MAX7313/issues/1
  */
uint8_t MAX7313::clearInterrupt(){
	if(this->conf & 0x08){
		uint8_t devnull = 0;
		if(this->read8(MAX7313_READ_IN_00_07, &devnull))
			return 1;
		if(this->read8(MAX7313_READ_IN_08_15, &devnull))
			return 1;
	}
	return 0;
}

/**
  * @brief 		writes a single value into a MAX7313 register
  * @param 		reg, the destination register's address
  * @param 		val, the value for the destination register
  */
uint8_t MAX7313::write8(uint8_t reg, uint8_t val){
	/**
	  * @note 	This is a glue function,
	  * 		You can change the hardware-specific functions here,
	  * 		if you want to use this library with another compiler (eg. Arduino)
	  * @note 	This method writes one byte to the register address reg
	  */
	if(HAL_I2C_Mem_Write(this->wireIface, this->devAddress, reg, 1, &val, 1, 10) != HAL_OK) 
		return 1;
	return 0;
}

/**
  * @brief 		reads a single value from a MAX7313 register
  * @param 		reg, the destination register's address
  * @param 		val, pointer to the location where the value shall be stored
  * @return 	0 on success, 1 on transmission error
  */
uint8_t MAX7313::read8(uint8_t reg, uint8_t *val){
	/**
	  * @note 	This is a glue function,
	  * 		You can change the hardware-specific functions here,
	  * 		if you want to use this library with another compiler (eg. Arduino)
	  * @note 	This method reads one byte from the register at address reg
	  */
	if(HAL_I2C_Mem_Read(this->wireIface, this->devAddress, reg, 1, val, 1, 10) != HAL_OK) 
		return 1;
	return 0;
}

/**
  * @brief 		Constructor for a MAX7313 PWM Output
  * @param 		chip, MAX7313 class object to which to assign the output
  * @param 		port, the port number on the chip (0-15)
  * @param 		polarity, set the output polarity 0 = active high / 1 = active low
  */
MAX7313Output::MAX7313Output(MAX7313 *chip, uint8_t port, uint8_t polarity=0){
	this->chip = chip;
	/* is this  */
	if(port < 16 && port > 0)		// check port range
		this->port = port;
	else
		this->port = 16;
	/** @todo 	correct port range error handling => what happens if port = 16 is sent to MAX7313
	  * 		port must not be 0 either, because of actual possible I/Os on port 0 */

	(this->port > 7) ? this->chip->ioconfig[1] &= ~(1<<(this->port%8)) : this->chip->ioconfig[0] &= ~(1<<(this->port));
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
	this->polarity = polarity;
	(this->polarity == 1) ? this->setIntensity(0xf) : this->setIntensity(0x0);
	/** @todo  	check for transmission error in setIntensity */
	/** 		how to handle/return errors in constructor ? */
}

/**
  * @brief 		Constructor for a MAX7313 Input
  * @param 		chip, MAX7313 class object to which to assign the input
  * @param 		port, the port number on the chip (0-15)
  */
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

/**
  * @brief 		set the intensity (brightness) on a MAX7313 Output
  * @param 		intensity (0-15)
  * @return 	0 on success, 1 on I2C transmission error
  */
uint8_t MAX7313Output::setIntensity(uint8_t intensity){
	if(intensity > 0x0F)
		intensity = 0x0F;
	//if(intensity < 0)			// unnecessary because unsigned int is never < 0
	//	intensity = 0;
	this->chip->intensity[this->port] = intensity;
	uint8_t val;
	/*                   		[odd] [even]
	register in max7313:		xxxx xxxx
	intensity[even]      		0000 xxxx					mask-> 0x0F
	intensity[odd]       		xxxx 0000	shift->  <<4	mask-> 0xF0
	*/
	// distinguish between active-high and active-low setting -> invert intensity value accordingly
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
	
	if(this->chip->write8(this->ioreg, val))
		return 1;
	return 0;
}

/**
  * @brief 		read the digital input value form a MAX7313 Input
  * @return 	0 = digital LOW, 1 = digital HIGH
  * @todo 		how to signal transmission error? => parameter = pointer to return value / return value = transmission success/error
  */
uint8_t MAX7313Input::read(){
	uint8_t ret = 0;
	this->chip->read8(this->ioreg, &ret);
	/** I haven't figured out how to correctly shift and bitmask the return value
	  * but since the return value in this case is only a 1-bit value it is not of great importance
	  * the if statement below works for 1-bit values	  */
	ret = ret&this->regmask;
	//ret = ret>>(this->port);
	if(ret) return 1;
	return 0;
}
