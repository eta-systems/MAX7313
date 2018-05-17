
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
  *
  * @todo 		documentation of individual methods
  */

#ifndef _MAX7313_H
#define _MAX7313_H

/**
  * @note 		tested using STM32F373
  */
#ifndef STM32F3XX_H
#include "stm32f3xx_hal.h"
#endif

#ifndef STM32F3XX_HAL_I2C_H
#include "stm32f3xx_hal_i2c.h"
#endif

/**
  * @note 		datasheet p.13 table 2. Register Address Map
  * @see 		https://datasheets.maximintegrated.com/en/ds/MAX7313.pdf
  */
#define MAX7313_READ_IN_00_07       	0x00
#define MAX7313_READ_IN_08_15       	0x01
#define MAX7313_BLINK_PHASE_0_00_07 	0x02
#define MAX7313_BLINK_PHASE_0_08_15 	0x03
#define MAX7313_PORTS_CONF_00_07    	0x06
#define MAX7313_PORTS_CONF_08_15    	0x07
#define MAX7313_BLINK_PHASE_1_00_07 	0x0A
#define MAX7313_BLINK_PHASE_1_08_15 	0x0B
#define MAX7313_OUT_INT_MA_16       	0x0E
#define MAX7313_CONFIGURATION       	0x0F
#define MAX7313_OUT_INT_01_00       	0x10
#define MAX7313_OUT_INT_03_02       	0x11
#define MAX7313_OUT_INT_05_04       	0x12
#define MAX7313_OUT_INT_07_06       	0x13
#define MAX7313_OUT_INT_09_08       	0x14
#define MAX7313_OUT_INT_11_10       	0x15
#define MAX7313_OUT_INT_13_12       	0x16
#define MAX7313_OUT_INT_15_14       	0x17
#define MAX7313_NO_PORT    	        	0x88 		// @todo check that this address is not within the address space of MAX7313 OR add check for NO_PORT befor writing to i2c bus

class MAX7313
{
	private:
		uint16_t devAddress;
		I2C_HandleTypeDef *wireIface;
		uint8_t conf;
	public:
		uint8_t intensity[16];
		uint8_t ioconfig[2];
		uint8_t begin();
	
		MAX7313(I2C_HandleTypeDef *wireIface, uint16_t address);
		uint8_t write8(uint8_t reg, uint8_t val);
		uint8_t read8(uint8_t reg, uint8_t *val);
		uint8_t enableInterrupt();
		uint8_t disableInterrupt();
		uint8_t clearInterrupt();
};

class MAX7313Output
{
	private:
		MAX7313 *chip;
		uint8_t port;
		uint8_t ioreg;
		uint8_t regmask;
		uint8_t polarity;
	public:
		MAX7313Output(MAX7313 *chip, uint8_t port, uint8_t polarity);
		uint8_t setIntensity(uint8_t intensity);
};

class MAX7313Input
{
	private:
		MAX7313 *chip;
		uint8_t port;
		uint8_t ioreg;
		uint8_t regmask;
	public:
		MAX7313Input(MAX7313 *chip, uint8_t port);
		uint8_t read();
};

#endif
