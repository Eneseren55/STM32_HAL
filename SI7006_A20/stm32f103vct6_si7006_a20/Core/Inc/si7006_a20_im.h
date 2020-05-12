/*
 * si7006_a20_im.h
 *
 *  Created on: May 8, 2020
 *      Author: Baris
 */

#ifndef INC_SI7006_A20_IM_H_
#define INC_SI7006_A20_IM_H_

#include "i2c.h"

#define deviceAddress 						   (0x40 << 1)
#define humidityRegister 						0xE5
#define temperatureRegister 					0xE3
#define temperatureRegisterFromRHMeasurement	0xE0 // Read Temperature Value from Previous RH Measurement
#define errorValue								0xFF

uint16_t 	readSI7006_A20_IM(uint8_t Command, I2C_HandleTypeDef *);
float 		readTemperature(I2C_HandleTypeDef *);
float 		readHumidity(I2C_HandleTypeDef *);
void 		readTemperatureAndHumidity(float values[2], I2C_HandleTypeDef *);
uint16_t    convert_to_uint16(uint8_t bytes[]);

#endif /* INC_SI7006_A20_IM_H_ */
