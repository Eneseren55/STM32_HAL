/*
 * si7006_a20_im.c
 *
 *  Created on: May 8, 2020
 *      Author: Baris
 */

#include "si7006_a20_im.h"


uint16_t convert_to_uint16(uint8_t bytes[])
{
  return (uint16_t)((bytes[0]<<8) | bytes[1]);
}

uint16_t readSI7006_A20_IM(uint8_t Command, I2C_HandleTypeDef * hi2c)
{
	//if(HAL_I2C_IsDeviceReady(&hi2c, deviceAddress, 1, 10) == HAL_OK)
	//{
	  uint8_t buffer[2] = {0,0};

	  if(HAL_OK != HAL_I2C_Master_Transmit(hi2c, deviceAddress, &Command, 1, 10000))
	    return errorValue;

	  if(HAL_OK != HAL_I2C_Master_Receive(hi2c, deviceAddress, buffer, 2, 10000))
	    return errorValue;
	//}
	  return convert_to_uint16(buffer);
}

float readTemperature(I2C_HandleTypeDef * hi2c)
{
    uint16_t rawTemperature = readSI7006_A20_IM(temperatureRegister,hi2c);
    if(rawTemperature == errorValue)
    	return errorValue;
    float temperature = (float)(((175.72 * rawTemperature) / 65536.0) - 46.85);
    return temperature;

}
float readHumidity(I2C_HandleTypeDef * hi2c)
{
    uint16_t rawHumidity = readSI7006_A20_IM(humidityRegister,hi2c);
    if(rawHumidity == errorValue)
    	return errorValue;
    float humidity = (float)(((125.0 * rawHumidity) / 65536.0) - 6.0);
    if(humidity < 0)
      return 0;
    else if(humidity > 100)
      return 100;
    else
      return (float)humidity;

    return humidity;
}


void readTemperatureAndHumidity(float values[2], I2C_HandleTypeDef * hi2c)
{
	float humidity = readHumidity(hi2c);
	values[0] = humidity;
    uint16_t rawTemperature = readSI7006_A20_IM(temperatureRegisterFromRHMeasurement,hi2c);
    if(rawTemperature == errorValue){
    	values[1] = errorValue;
    	return;
    }
    float temperature = (float)(((175.72 * rawTemperature) / 65536.0) - 46.85);
    values[1] = temperature;

}
