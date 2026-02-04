/*
 * mpu6050.hpp
 *
 *  Created on: Dec 03, 2025
 *      Author: KiteBuilder
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include <stm32f4xx_hal.h>
#include "mpu_commands.h"
#include "Filters.h"

//******************************************************************************
// MPU6050 calibration class to calibrate gyro and accel measurements
//******************************************************************************
class MPU6050_Calibration {

    enum calib_states {CALIBRATE_STEP1 = 0, CALIBRATE_STEP2, CALIBRATE_FINISHED};

    bool f_CalibEn = false;
    calib_states state = CALIBRATE_STEP1;    
    uint16_t n_samples = 0; //should be gotten in constructor
    uint16_t samples_cnt = 0;

    //all pointers should be gotten in constructor
    const float *gyroData = NULL;   //pointer to XYZ gyro array 
    const float *accData = NULL;    //pointer to XYZ accel array
    float *gyroZeroOffset = NULL;       //pointer to array with gyro zero offsets
    float *accZeroOffset = NULL;        //pointer to array with accell zero offsets

    float gyroSum[XYZ_AXIS_COUNT] = {0.0f, 0.0f, 0.0f};
    float gyroAvg[XYZ_AXIS_COUNT] = {0.0f, 0.0f, 0.0f};

    float accSum[XYZ_AXIS_COUNT] = {0.0f, 0.0f, 0.0f};
    float accAvg[XYZ_AXIS_COUNT] = {0.0f, 0.0f, 0.0f};

    void mpu6050_CalibrateStep1();
    void mpu6050_CalibrateStep2();

public:
    void mpu6050_Calibrate();

    bool mpu6050_IsCalibrate()
    {
        return f_CalibEn;
    }

    MPU6050_Calibration(const float *p_gyroData, float *p_gyroOffset, const float *p_accData, float *p_accOffset, uint16_t samples);
    ~MPU6050_Calibration() = default;
};

//******************************************************************************
// MPU6050 interface virtual class
//******************************************************************************
class MPU6050_Interface : public MPU6050_Calibration{
    
    #define CALIBRATION_SAMPLES     1000
    #define TEMPERATURE_ERROR       2.5f
    #define LPF_CUT_OFF_FREQ        250 //Hz

    virtual HAL_StatusTypeDef mpu6050_WriteByte(uint8_t reg, uint8_t data) = 0;
    virtual HAL_StatusTypeDef mpu6050_WriteBuf(uint8_t reg, uint8_t * data, size_t length) = 0;
    virtual HAL_StatusTypeDef mpu6050_ReadByte(uint8_t reg, uint8_t * data) = 0;
    virtual HAL_StatusTypeDef mpu6050_ReadBuf(uint8_t reg, uint8_t * data, size_t length) = 0;

    const gyroFilterAndRateConfig_t *p_config;
    //Raw Gyro and Accel data
    float gyroADCRaw[XYZ_AXIS_COUNT] = {0.0f, 0.0f, 0.0f};
    float accADCRaw[XYZ_AXIS_COUNT] = {0.0f, 0.0f, 0.0f};
    //Gyro and Accel zero offsets
    float gyroZeroOffset[XYZ_AXIS_COUNT] = {0.0f, 0.0f, 0.0f};
    float accZeroOffset[XYZ_AXIS_COUNT] = {0.0f, 0.0f, 0.0f};

    float tempC = 0.0f;
    //Gyro and Accel sensor ranges that depends on full scale range
    float gyroSensRange = 0.0f;
    float accSensRange = 0.0f;

    //Anti aliasing PT1 LPF filters
    PT1Filter gyroLPFFilter[XYZ_AXIS_COUNT];
    PT1Filter accelLPFFilter[XYZ_AXIS_COUNT];
    //Filtered Gyro and Accel data
    float gyroADCFilt[XYZ_AXIS_COUNT] = {0.0f, 0.0f, 0.0f};
    float accADCFilt[XYZ_AXIS_COUNT] = {0.0f, 0.0f, 0.0f};    

public:
    void mpu6050_Init(uint8_t desiredLpf, uint16_t desiredRateHz, uint8_t desiredGyroScale, uint8_t desiredAccScale);
    bool mpu6050_GyroRead(void);
    bool mpu6050_ReadAccTempGyro(void);
    void mpu6050_GetGyroData(float *data);    
    void mpu6050_GetAccData(float *data);
    void mpu6050_GetTempC(float *data);

    void mpu6050_GetGyroDataFilt(float *data);    
    void mpu6050_GetAccDataFilt(float *data);    

    const gyroFilterAndRateConfig_t* mpu6050_ChooseGyroConfig(uint8_t desiredLpf, uint16_t desiredRateHz);

    MPU6050_Interface() : MPU6050_Calibration(gyroADCRaw, gyroZeroOffset, accADCRaw, accZeroOffset, CALIBRATION_SAMPLES)
    {

    }

    ~MPU6050_Interface() = default;
};

//******************************************************************************
//
//******************************************************************************
class MPU6050_I2C : public MPU6050_Interface {

    I2C_HandleTypeDef *p_hi2c;

    static MPU6050_I2C *p_instance;

    HAL_StatusTypeDef mpu6050_WriteByte(uint8_t reg, uint8_t data);
    HAL_StatusTypeDef mpu6050_WriteBuf(uint8_t reg, uint8_t * data, size_t length);
    HAL_StatusTypeDef mpu6050_ReadByte(uint8_t reg, uint8_t * data);
    HAL_StatusTypeDef mpu6050_ReadBuf(uint8_t reg, uint8_t * data, size_t length);

    MPU6050_I2C(I2C_HandleTypeDef *p_handler)
    {
        p_hi2c = p_handler;
    }

public:
    MPU6050_I2C(const MPU6050_I2C &obj) = delete;

    static MPU6050_I2C* getInstance()
    {
        return p_instance;
    }

    ~MPU6050_I2C() = default;
};

#endif /* MPU6050_H_*/