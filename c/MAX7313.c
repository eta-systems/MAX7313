
/**
  * @note 		STILL IN DEVELOPMENT
  * @file 		max3713.c
  * @author 	Simon Burkhardt github.com/mnemocron
  * @copyright 	MIT license
  * @date 		17.05.2018
  * @brief 		C library for the MAX7313 port expander for STM32 HAL.
  * @details 	
  * @see 		https://datasheets.maximintegrated.com/en/ds/MAX7313.pdf
  * @see 		https://forum.arduino.cc/index.php?topic=9682.0
  */

#include "max7313.h"
#include "main.h"

/**
  * @brief 		writes a single value into a MAX7313 register
  * @param 		*chip, pointer to the MAX7313 typedef struct
  * @param 		reg, the destination register's address
  * @param 		val, the value for the destination register
  */
uint8_t MAX7313_Write8(MAX7313 *chip, uint8_t reg, uint8_t val){
	if(HAL_I2C_Mem_Write(chip->wireIface, chip->devAddress, reg, 1, &val, 1, 10) != HAL_OK) 
		return 1;
	return 0;
}

/**
  * @brief 		reads a single value from a MAX7313 register
  * @param 		*chip, pointer to the MAX7313 typedef struct
  * @param 		reg, the destination register's address
  * @param 		val, pointer to the location where the value shall be stored
  * @return 	0 on success, 1 on transmission error
  */
uint8_t MAX7313_Read8(MAX7313 *chip, uint8_t reg, uint8_t *val){
	if(HAL_I2C_Mem_Read(chip->wireIface, chip->devAddress, reg, 1, val, 1, 10) != HAL_OK) 
		return 1;
	return 0;
}

/**
  * @brief 		initializes a MAX7313 struct with the default values
  * @return 	MAX7313 containing the default register values
  */
MAX7313 new_MAX7313(void){
	MAX7313 chip;
	chip.ioconfig[0] = 0xFF;			// default input / 1=IN / 0=OUT
	chip.ioconfig[1] = 0xFF;			// default input / 1=IN / 0=OUT
	chip.conf = 0x00;
	for(uint8_t i=0; i<sizeof(chip.intensity); i++){
		chip.intensity[i] = 0xff;
	}
	return chip;
}

/**
  * @brief 		Initalizer function
  * @param 		*chip, pointer to the MAX7313 typedef struct
  * @param 		*wireIface a pointer to a HAL I2C_HandleTypeDef
  * @param 		address of the chip on the I2C bus
  * @return 	0 on success, 1 on I2C transmission error
  * @pre 		all Inputs & Outputs must be declared before calling begin()
  */
uint8_t MAX7313_Init(MAX7313 *chip, I2C_HandleTypeDef *wireIface, uint16_t address){
	chip->wireIface = wireIface;
	chip->devAddress = address;
	uint8_t ret = 0;
	ret += MAX7313_Write8(chip, MAX7313_PORTS_CONF_00_07,    chip->ioconfig[0]);
	ret += MAX7313_Write8(chip, MAX7313_PORTS_CONF_08_15,    chip->ioconfig[1]);
	ret += MAX7313_Write8(chip, MAX7313_BLINK_PHASE_0_00_07, 0xff);
	ret += MAX7313_Write8(chip, MAX7313_BLINK_PHASE_0_08_15, 0xff);
	ret += MAX7313_Write8(chip, MAX7313_CONFIGURATION,       chip->conf);
	ret += MAX7313_Write8(chip, MAX7313_OUT_INT_MA_16,       0xff);
	if(ret)
		return 1;
	return 0;
}

/**
  * @brief 		Set the desired Pin mode on a MAX7313 I/O port
  * @param 		*chip, pointer to the MAX7313 typedef struct
  * @param 		port, the port to modify
  * @param 		mode, the desired mode [PORT_INPUT | PORT_OUTPUT]
  */
void MAX7313_Pin_Mode(MAX7313 *chip, uint8_t port, uint8_t mode){
	if(mode == PORT_OUTPUT){
		if(port > 7)
			chip->ioconfig[1] &= ~(1<<(port%8));
		else
			chip->ioconfig[0] &= ~(1<<(port));
	} else {
		if(port > 7)
			chip->ioconfig[1] |= (1<<(port%8));
		else
			chip->ioconfig[0] |= (1<<(port));
	}
}

/**
  * @brief 		set the intensity (brightness) on a MAX7313 Output
  * @param 		*chip, pointer to the MAX7313 typedef struct
  * @param 		port, the port to modify
  * @param 		intensity (0-15)
  * @return 	0 on success, 1 on I2C transmission error
  */
uint8_t MAX7313_Pin_Write(MAX7313 *chip, uint8_t port, uint8_t intensity){
	if(intensity > 0x0F)
		intensity = 0x0F;
	chip->intensity[port] = intensity;
	uint8_t val;
	if(port % 2) 		//  odd (1,3,5) => this + this-1
		val = ((chip->intensity[port]<<4)&0xF0) + (chip->intensity[port-1]&0x0F);
	else									// even (0,2,4) => this+1 + this
		val = ((chip->intensity[port+1]<<4)&0xF0) + (chip->intensity[port]&0x0F);
	if(MAX7313_Write8(chip, __max7313_get_output_reg(port), val))
		return 1;
	return 0;
}

/**
  * @brief 		read the digital input value form a MAX7313 Input
  * @param 		*chip, pointer to the MAX7313 typedef struct
  * @param 		port, the port to read from
  * @return 	0 = digital LOW, 1 = digital HIGH
  * @todo 		how to signal transmission error? => parameter = pointer to return value / return value = transmission success/error
  */
uint8_t MAX7313_Pin_Read(MAX7313 *chip, uint8_t port){
	uint8_t ret = 0;
	MAX7313_Read8(chip, __max7313_get_input_reg(port), &ret);
	ret = ret & __max7313_get_regmask(port);
	if(ret) 
		return 1;
	return 0;
}

/**
  * @brief 		enables data change interrupt on !INT/O16 pin
  * @param 		*chip, pointer to the MAX7313 typedef struct
  * @return 	0 on success, 1 on I2C transmission error
  * @see 		MAX7313 datasheet p.7 / p.13 - 15
  * @important 	The MAX7313 signals a data change interrupt with a falling edge on the !INT/O16 pin
  *           	and will stay in LOW state as long as the interrput flag is not reset by reading input registers.
  *           	You can use MAX7313::clearInterrupt()
  */
uint8_t MAX7313_Interrupt_Enable(MAX7313 *chip){
	chip->conf |= 0x08;
	if(MAX7313_Write8(chip, MAX7313_CONFIGURATION, chip->conf))
		return 1;
	if(MAX7313_Interrupt_Clear(chip))
		return 1;
	return 0;
}

/**
  * @brief 		disables data change interrupt on !INT/O16 pin
  * @param 		*chip, pointer to the MAX7313 typedef struct
  * @return 	0 on success, 1 on I2C transmission error
  * @see 		MAX7313 datasheet p.7 / p.13 - 15
  */
uint8_t MAX7313_Interrupt_Disable(MAX7313 *chip){
	chip->conf &= 0xF7;			// clear bit 3
	if(MAX7313_Write8(chip, MAX7313_CONFIGURATION, chip->conf))
		return 1;
	return 0;
}

/**
  * @brief 		clear/validate the data change interrupt
  * @details 	This method is used to clear an interrupt condition (!INT/O16 pin = LOW)
  * 			by reading the input registers. The MAX7313 will pull the !INT/O16 pin HIGH
  * 			and will be armed for another interrupt event.
  * @param 		*chip, pointer to the MAX7313 typedef struct
  * @return 	0 on success, 1 on I2C transmission error, 2 if interrupts are not enabled
  * @see 		https://github.com/mnemocron/MAX7313/issues/1
  */
uint8_t MAX7313_Interrupt_Clear(MAX7313 *chip){
	if(chip->conf & 0x08){
		uint8_t devnull = 0;
		if(MAX7313_Read8(chip, MAX7313_READ_IN_00_07, &devnull))
			return 1;
		if(MAX7313_Read8(chip, MAX7313_READ_IN_08_15, &devnull))
			return 1;
	}
	return 0;
}


