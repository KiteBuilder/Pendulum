#ifndef POWER_H_
#define POWER_H_

#include <stm32f4xx_hal.h>

#include "Filters.h"
#include "times.h"

#define R_SHUNT 0.00025f  // omh
#define R_DIV1 22000      // omh
#define R_DIV2 2200       // omh
#define V_REF 3.3f        // Volt
#define MV_TO_AMP 11.75f  // mv/A
#define ADC_RANGE 4095    // 2^12 - 1
#define VOLT_SCALE (V_REF / ADC_RANGE)

#define ADC_AVERAGE_N_SAMPLES 20

#define VBATT_LPF_FREQ 0.5f  // Hz
#define IBATT_LPF_FREQ 0.5f  // Hz

#define AS_TO_MAH 0.2778f

struct average_t
{
    float val = 0;
    float buf[ADC_AVERAGE_N_SAMPLES] = {0};
    uint8_t cnt = 0;

    void clear()
    {
        val = 0;
        cnt = 0;

        for (uint32_t i = 0; i < ADC_AVERAGE_N_SAMPLES; i++) 
        {
            buf[i] = 0;
        }
    }
};

class Power
{
    ADC_HandleTypeDef* p_hadc;

    average_t iBat;
    average_t vBat;

    uint16_t dma_buff[2];

    // filters
    float delta_time = 0.0f;
    timeUs_t previousTimeUs = 0;
    PT1Filter vBatFilter;
    PT1Filter iBatFilter;

    // Voltage and current values
    float iBatFilt = 0.0f;
    float vBatFilt = 0.0f;
    float iBatRaw = 0.0f;
    float vBatRaw = 0.0f;
    float drawn_mAh = 0.0f;

    void addToAverage(average_t& avrg, float value);
    void generateAverageVar(average_t& avrg);
    float convertToVolt(float val);
    float convertToAmp(float val);
    float adcToVoltage(float val);

public:
    void StartADC(timeUs_t currentTimeUs);
    void HandleADC(void);

    float GetIBat(void) { return iBatRaw; }

    float GetVBat(void) { return vBatRaw; }

    float GetIBatFilt(void) { return iBatFilt; }

    float GetVBatFilt(void) { return vBatFilt; }

    float GetEBat(void) { return drawn_mAh; }

    Power(ADC_HandleTypeDef* hadc);
    ~Power() = default;
};

#endif /*POWER_H_*/