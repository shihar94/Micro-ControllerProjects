#include "mpu6050.h"

HAL_StatusTypeDef MPU6050_ReadRegister(MPU6050 *dev,uint8_t reg ,uint8_t *data){
	return HAL_I2C_Mem_Read(dev->i2cHandle, MPU6050_ADDRESS, reg , I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU6050_ReadRegisters(MPU6050 *dev,uint8_t reg ,uint8_t *data,uint8_t length){
	return HAL_I2C_Mem_Read(dev->i2cHandle, MPU6050_ADDRESS, reg , I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU6050_WriteRegister(MPU6050 *dev,uint8_t reg, uint8_t *data){
	return HAL_I2C_Mem_Write(dev->i2cHandle, MPU6050_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}


uint8_t MPU6050_Initialise(MPU6050 *dev,I2C_HandleTypeDef *i2cHandle){
	dev->i2cHandle = i2cHandle;
	dev->acc_mps2[0] = 0.0f;
	dev->acc_mps2[1] = 0.0f;
	dev->acc_mps2[2] = 0.0f;

	dev->tempc       = 0.0f;

	dev->gyro_data[0] = 0.0f;
	dev->gyro_data[1] = 0.0f;
	dev->gyro_data[2] = 0.0f;
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	uint8_t regData;
	status = MPU6050_ReadRegister(dev, WHO_AM_I_MPU6050, &regData);
	errNum +=(status !=HAL_OK);
	if(regData !=MPU6050_BASE_ADDRESS){
		return 255;
	}


	regData = 0x07;
	status = MPU6050_WriteRegister(dev,SMPLRT_DIV , regData);
	errNum +=(status !=HAL_OK);
	regData = 0x00;
	//status = MPU6050_WriteRegister(dev,CONFIG , &regData);
	//errNum +=(status !=HAL_OK);

	status = MPU6050_WriteRegister(dev,ACCEL_CONFIG , &regData);
	errNum +=(status !=HAL_OK);

	regData = 0x08;
	status = MPU6050_WriteRegister(dev,GYRO_CONFIG ,&regData);
	errNum +=(status !=HAL_OK);

	regData = 0x01;
	status = MPU6050_WriteRegister(dev,PWR_MGMT_1 , &regData);
	errNum +=(status !=HAL_OK);

	return errNum;
}

HAL_StatusTypeDef MPU6050_ReadTemperature(MPU6050 *dev){
	uint8_t regData[2];

	HAL_StatusTypeDef status = MPU6050_ReadRegisters(dev, TEMP_OUT_H, regData, 2);
	uint16_t tempRaw = ((regData[0]<<8)| regData[1]);
	dev->tempc = (tempRaw + 12412.0) / 340.0;
	return status;
}


HAL_StatusTypeDef MPU6050_ReadAccelerations(MPU6050 *dev){
	int8_t regData[14];
	HAL_StatusTypeDef status = MPU6050_ReadRegisters(dev, ACCEL_XOUT_H, regData, 6);
	int16_t accX = (regData[0]<<8) | regData[1];
	int16_t accY = (regData[2]<<8) | regData[3];
	int16_t accZ = (regData[4]<<8) | regData[5];

	int16_t temp = (regData[6]<<8) | regData[7];

	int16_t gyrX = (regData[8]<<8) | regData[9];
	int16_t gyrY = (regData[10]<<8) | regData[11];
	int16_t gyrZ = (regData[12]<<8) | regData[13];

	dev->acc_mps2[0] = ((float)accX)/16384.0;
	dev->acc_mps2[1] = ((float)accY)/16384.0;
	dev->acc_mps2[2] = ((float)accZ)/16384.0;

	dev->tempc = (((float)temp) + 12412.0) / 340.0;

	dev->gyro_data[0] = ((float)gyrX)/ 131.0;
	dev->gyro_data[1] = ((float)gyrY)/ 131.0;
	dev->gyro_data[2] = ((float)gyrZ)/ 131.0;


	return status;
}

