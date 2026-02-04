/*
 * mpu6050.cpp
 *
 *  Created on: Dec 3, 2025
 *      Author: KiteBuilder
 */
    
#include <cstddef>
#include <cstdint>
#include <cstdlib>

#include <mpu6050.h>
#include "times.h"

using namespace std;

//******************************************************************************
//  MPU6050_Interface methods
//******************************************************************************

/**
  * @brief Init MPU6050 Gyro sensor
  * @param
  * desiredLpf : mpu60505 inner gyro LPF filter frequency in Hz
  * desiredRateHz : desired gyro rate in Hz
  * desiredGyroScale : gyro scale range
  * desiredAccScale : accel scale range
  * @retval None
  */
void MPU6050_Interface::mpu6050_Init(uint8_t desiredLpf, uint16_t desiredRateHz, uint8_t desiredGyroScale, uint8_t desiredAccScale)
{
    //initialize antialiasing LPF filter
    float loopTime_sec = 1.0f / (float)desiredRateHz;
    for (uint32_t i = 0; i < XYZ_AXIS_COUNT; i++)
    {
        gyroLPFFilter[i].FilterInit(LPF_CUT_OFF_FREQ, loopTime_sec);
        accelLPFFilter[i].FilterInit(LPF_CUT_OFF_FREQ, loopTime_sec);
    }    

    p_config = mpu6050_ChooseGyroConfig(desiredLpf, desiredRateHz);

    // Device Reset
    mpu6050_WriteByte(MPU_RA_PWR_MGMT_1, BIT_H_RESET);
    delayMs(150);

    mpu6050_WriteByte(MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
    delayMs(150);

    // Clock Source PPL with Z axis gyro reference
    mpu6050_WriteByte(MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
    delayUs(15);

    mpu6050_WriteByte(MPU_RA_PWR_MGMT_2, 0x00);
    delayUs(15);

    // Accel Sample Rate 1kHz
    // Gyroscope Output Rate =  1kHz when the DLPF is enabled
    mpu6050_WriteByte(MPU_RA_SMPLRT_DIV, p_config->gyroConfigValues[1]);
    delayUs(15);

    // Gyro +/- 2000 DPS Full Scale
    mpu6050_WriteByte(MPU_RA_GYRO_CONFIG, desiredGyroScale << 3);
    delayUs(15);

    gyroSensRange = gyro_sens_table[desiredGyroScale];

    // Accel +/- 16 G Full Scale
    mpu6050_WriteByte(MPU_RA_ACCEL_CONFIG, desiredAccScale << 3);
    delayUs(15);

    accSensRange = (float)acc_sens_table[desiredAccScale];

    // Accel and Gyro DLPF Setting
    mpu6050_WriteByte(MPU_RA_CONFIG, p_config->gyroConfigValues[0]);
    delayUs(1);

    mpu6050_GyroRead();
}

/**
  * @brief Read MPU6050 gyro data 
  * @param None
  * @retval return true if read was successful
  */
bool MPU6050_Interface::mpu6050_GyroRead(void)
{
    uint8_t data[6];

    HAL_StatusTypeDef status = mpu6050_ReadBuf(MPU_RA_GYRO_XOUT_H, data, 6);
    if (status != HAL_OK) 
    {
        gyroADCRaw[X] = 0.0f;
        gyroADCRaw[Y] = 0.0f;
        gyroADCRaw[Z] = 0.0f;
        return false;
    }
    // big endian data
    gyroADCRaw[X] = (float)get_int16_val_bigend(data, 0) - gyroZeroOffset[X];
    gyroADCRaw[Y] = (float)get_int16_val_bigend(data, 1) - gyroZeroOffset[Y];
    gyroADCRaw[Z] = (float)get_int16_val_bigend(data, 2) - gyroZeroOffset[Z];

    return true;
}

/**
  * @brief Read Accel, Gyro and Temperature data as one data flow
  * @param None
  * @retval return true if read was successful
  */
 bool MPU6050_Interface::mpu6050_ReadAccTempGyro(void)
 {
    uint8_t data[6 + 2 + 6];

    HAL_StatusTypeDef status = mpu6050_ReadBuf(MPU_RA_ACCEL_XOUT_H, data, 6 + 2 + 6);
    if (status != HAL_OK) 
    {
        gyroADCRaw[X] = 0.0f;
        gyroADCRaw[Y] = 0.0f;
        gyroADCRaw[Z] = 0.0f;

        accADCRaw[X] = 0.0f;
        accADCRaw[Y] = 0.0f;
        accADCRaw[Z] = 0.0f;
        
        tempC = 0.0f;
        return false;
    }
    //big endian data
    accADCRaw[X] = (float)get_int16_val_bigend(data, 0) - accZeroOffset[X];
    accADCRaw[Y] = (float)get_int16_val_bigend(data, 1) - accZeroOffset[Y];
    accADCRaw[Z] = (float)get_int16_val_bigend(data, 2) - accZeroOffset[Z];

    tempC = (float)get_int16_val_bigend(data, 3);

    gyroADCRaw[X] = (float)get_int16_val_bigend(data, 4) - gyroZeroOffset[X];
    gyroADCRaw[Y] = (float)get_int16_val_bigend(data, 5) - gyroZeroOffset[Y];
    gyroADCRaw[Z] = (float)get_int16_val_bigend(data, 6) - gyroZeroOffset[Z];

    //LPF PT1 filter for data
    for (uint32_t i = 0; i < XYZ_AXIS_COUNT; i++)
    {
        if (gyroLPFFilter[i].isFirstLoad())
        {
            gyroLPFFilter[i].FilterSetVal(gyroADCRaw[i]);
        }
        gyroADCFilt[i] = gyroLPFFilter[i].FilterApply(gyroADCRaw[i]);

        if (accelLPFFilter[i].isFirstLoad())
        {
            accelLPFFilter[i].FilterSetVal(accADCRaw[i]);
        }
        accADCFilt[i] = accelLPFFilter[i].FilterApply(accADCRaw[i]);
    }

    return true;
 }

/**
  * @brief Return MPU6050 gyro data with applied offsets and scaled
  * @param 
  * data : pointer to the gyro data array
  * @retval None
  */
void MPU6050_Interface::mpu6050_GetGyroData(float *data)
{
    data[X] = gyroADCRaw[X] / gyroSensRange;
    data[Y] = gyroADCRaw[Y] / gyroSensRange;
    data[Z] = gyroADCRaw[Z] / gyroSensRange;
}

/**
  * @brief Return MPU6050 accel data with applied offsets and scaled
  * @param
  * data : pointer to the accel data array
  * @retval None
  */
void MPU6050_Interface::mpu6050_GetAccData(float *data)
{
    data[X] = accADCRaw[X] / accSensRange;
    data[Y] = accADCRaw[Y] / accSensRange;
    data[Z] = (accADCRaw[Z] + accSensRange) / accSensRange;
}

/**
  * @brief Return MPU6050 normilized temperature data
  * @param 
  * data : pointer to the temperature variable
  * @retval None
  */
void MPU6050_Interface::mpu6050_GetTempC(float *data)
{
    *data = (tempC/ 340) + 36.53 - TEMPERATURE_ERROR;
}

/**
  * @brief Return MPU6050 filtered gyro data with applied offsets and scaled
  * @param 
  * data : pointer to the gyro data array
  * @retval None
  */
void MPU6050_Interface::mpu6050_GetGyroDataFilt(float *data)
{
    data[X] = gyroADCFilt[X] / gyroSensRange;
    data[Y] = gyroADCFilt[Y] / gyroSensRange;
    data[Z] = gyroADCFilt[Z] / gyroSensRange;
}

/**
  * @brief Return MPU6050 filtered accel data with applied offsets and scaled
  * @param
  * data : pointer to the accel data array
  * @retval None
  */
void MPU6050_Interface::mpu6050_GetAccDataFilt(float *data)
{
    data[X] = accADCFilt[X] / accSensRange;
    data[Y] = accADCFilt[Y] / accSensRange;
    data[Z] = (accADCFilt[Z] + accSensRange) / accSensRange;
}

/**
  * @brief Chose MPU6050 configuration from the mpuGyroConfigs array
  * @param 
  * desiredLpf : mpu60505 inner gyro LPF filter frequency in Hz
  * desiredRateHz : desired gyro rate in Hz
  * @retval gyroFilterAndRateConfig_t * : pointer to the configuration structure
  */
const gyroFilterAndRateConfig_t * MPU6050_Interface::mpu6050_ChooseGyroConfig(uint8_t desiredLpf, uint16_t desiredRateHz)
{
    const gyroFilterAndRateConfig_t * config;
    uint32_t cfg_size = sizeof(mpuGyroConfigs) / sizeof(mpuGyroConfigs[0]);

    for (uint32_t i = 0;  i < cfg_size; i++)
    {
        if (mpuGyroConfigs[i].gyroLpf == desiredLpf && mpuGyroConfigs[i].gyroRateHz == desiredRateHz)
        {
            config = &mpuGyroConfigs[i];
        }
    }    

    return config;
}

//******************************************************************************
// MPU6050_I2C methods
//******************************************************************************

/**
  * @brief Write one byte
  * @param 
  * reg - mpu6050 register address
  * data -  data byte
  * @retval None
  */
HAL_StatusTypeDef MPU6050_I2C::mpu6050_WriteByte(uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(p_hi2c, MPU6050_I2C_ADDR, reg, 1, &data, 1, HAL_MAX_DELAY);
}

/**
  * @brief Write bytes sequence
  * @param
  * reg - mpu6050 register address
  * p_data -  data bufer pointer
  * length - amount of data bytes
  * @retval None
  */
HAL_StatusTypeDef MPU6050_I2C::mpu6050_WriteBuf(uint8_t reg, uint8_t *p_data, size_t length)
{
    return HAL_I2C_Mem_Write(p_hi2c, MPU6050_I2C_ADDR, reg, 1, p_data, length, HAL_MAX_DELAY);
}

/**
  * @brief Read one byte
  * @param
  * reg - mpu6050 register address
  * p_data -  pointer to the data byte
  * @retval None
  */
HAL_StatusTypeDef MPU6050_I2C::mpu6050_ReadByte(uint8_t reg, uint8_t * p_data)
{
    return HAL_I2C_Mem_Read(p_hi2c, MPU6050_I2C_ADDR, reg, 1, p_data, 1, HAL_MAX_DELAY);
}

/**
  * @brief Read bytes sequence
  * @param
  * reg - mpu6050 register address
  * p_data -  data bufer pointer
  * length - amount of data bytes
  * @retval None
  */
HAL_StatusTypeDef MPU6050_I2C::mpu6050_ReadBuf(uint8_t reg, uint8_t * p_data, size_t length)
{
    return HAL_I2C_Mem_Read(p_hi2c, MPU6050_I2C_ADDR, reg, 1, p_data, length, HAL_MAX_DELAY);
}

//******************************************************************************
//  MPU6050_Calibrate methods
//******************************************************************************
/**
  * @brief MPU6050_Calibration class constructor
  * @param 
  * p_gyroData - pointer to XYZ gyro array 
  * p_gyroOffset - pointer to XYZ accel array 
  * p_accData - pointer to array with gyro offsets
  * p_accOffset - pointer to array with accel offsets
  * samples - number of sumples that should be taken for the calibration process
  * @retval None
  */
MPU6050_Calibration::MPU6050_Calibration(const float *p_gyroData, float *p_gyroOffset, const float *p_accData, float *p_accOffset, uint16_t samples)
{
    gyroData = p_gyroData;
    accData = p_accData;
    gyroZeroOffset = p_gyroOffset;
    accZeroOffset = p_accOffset;
    n_samples = samples;

    if (gyroData == NULL || accData == NULL || gyroZeroOffset == NULL || accZeroOffset == NULL)
    {
        return;
    }    

    if (n_samples != 0)
    {
        f_CalibEn = true;
    }    
}

/**
  * @brief MPU6050 sensor calibration routine 
  * @param None
  * @retval None
  */
void MPU6050_Calibration::mpu6050_Calibrate()
{
    if (!f_CalibEn)
    {
        return;
    }

    //Accumulate gyro and accel data for n_samples
    for (uint32_t i = 0; i < XYZ_AXIS_COUNT; i++)
    {
        gyroSum[i] += gyroData[i];
        accSum[i] += accData[i];
    }

    ++samples_cnt;

    if (samples_cnt >= n_samples)
    {
        //Calculate average gyro and accel data for n_samples
        for (uint32_t i = 0; i < XYZ_AXIS_COUNT; i++)
        {
            gyroAvg[i] = gyroSum[i] / (float)n_samples;
            accAvg[i] = accSum[i] / (float)n_samples;
        }

        // Clear all auxillary counters and arrays for the next iteration
        samples_cnt = 0;

        for (uint32_t i = 0; i < XYZ_AXIS_COUNT; i++)
        {
            gyroSum[i] = 0;
            accSum[i] = 0;
        }

        if (state == CALIBRATE_STEP1)
        {
            mpu6050_CalibrateStep1();
        } 
        else if (state == CALIBRATE_STEP2)
        {
            mpu6050_CalibrateStep2();
        }
    }
}

 /**
  * @brief MPU6050 sensor calibration Step1
  * @param None
  * @retval None
  */
 void MPU6050_Calibration::mpu6050_CalibrateStep1()
 {
    //take average values as an offset values 
    for (uint32_t i = 0; i < XYZ_AXIS_COUNT; i++)
    {
        gyroZeroOffset[i] =  gyroAvg[i]; 
        accZeroOffset[i] =  accAvg[i]; 
    }
    state =  CALIBRATE_STEP2;
 }

  /**
  * @brief MPU6050 sensor calibration Step2
  * @param None
  * @retval None
  */
 void MPU6050_Calibration::mpu6050_CalibrateStep2()
 {
    //take offset corrections
    for (uint32_t i = 0; i < XYZ_AXIS_COUNT; i++)
    {
        gyroZeroOffset[i] +=  gyroAvg[i]; 
        accZeroOffset[i] +=  accAvg[i]; 
    }
    state =  CALIBRATE_FINISHED;
    f_CalibEn = false;
 }