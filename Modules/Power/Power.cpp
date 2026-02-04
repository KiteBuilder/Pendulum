#include "Power.h"

#include <stdint.h>
#include <stdio.h>

/**
 * @brief Constructor
 * @param 
 * hadc : pointer to the ADC descriptor
 * @retval None
 */
Power::Power(ADC_HandleTypeDef *hadc) : p_hadc(hadc)
{

}

/**
 * @brief Start ADC conversions
 * @param 
 * currentTimeUs : current timestamp in us
 * @retval None
 */
void Power::StartADC(timeUs_t currentTimeUs) 
{ 
    // calculate time since last update
    delta_time = US2S(currentTimeUs - previousTimeUs);
    previousTimeUs = currentTimeUs;

    HAL_ADC_Start_DMA(p_hadc, (uint32_t*)dma_buff, 2); 
}

/**
 * @brief Handle ADC(Power) measurements
 * @param None
 * @retval None
 */
void Power::HandleADC(void)
{
    HAL_ADC_Stop_DMA(p_hadc);

    addToAverage(iBat, (float)dma_buff[0]);
    addToAverage(vBat, (float)dma_buff[1]);

    generateAverageVar(iBat);
    generateAverageVar(vBat);

    iBatRaw = convertToAmp(adcToVoltage(iBat.val));
    vBatRaw = convertToVolt(adcToVoltage(vBat.val));

    // update the filtered voltage and currents
    if (vBatFilter.isFirstLoad())
    {
        vBatFilter.FilterSetVal(vBatRaw);
    }
    vBatFilt = vBatFilter.FilterApply(vBatRaw, delta_time, VBATT_LPF_FREQ);

    if (iBatFilter.isFirstLoad())
    {
        iBatFilter.FilterSetVal(iBatRaw);
    }
    iBatFilt = iBatFilter.FilterApply(iBatRaw, delta_time, IBATT_LPF_FREQ);

    drawn_mAh += iBatRaw * delta_time * AS_TO_MAH;
}

/**
 * @brief Add value to the buffer
 * @param 
 * avrg : pointer to average_t structure (buffer for values)
 * value : measured value
 * @retval None
 */
void Power::addToAverage(average_t& avrg, float value)
{
    if (avrg.cnt >= ADC_AVERAGE_N_SAMPLES) 
    {
        for (int i = 0; i < ADC_AVERAGE_N_SAMPLES; i++) 
        {
            value += avrg.buf[i];
        }

        value /= (ADC_AVERAGE_N_SAMPLES + 1);
        avrg.cnt = 0;
    }

    avrg.buf[avrg.cnt++] = value;
}

/**
 * @brief Calculate average value
 * @param
 * avrg : pointer to average_t structure (buffer for values)
 * @retval None
 */
void Power::generateAverageVar(average_t& avrg)
{
    float val = 0;
    uint8_t avSize = avrg.cnt;

    if (avSize == 0) 
    {
        return;
    }

    for (int i = 0; i < avSize; i++) 
    {
        val += avrg.buf[i];
    }

    avrg.val = val / avSize;
}

/**
 * @brief Convert ADC value to measured voltage
 * @param 
 * val : ADC value in Volts
 * @retval None
 */
float Power::convertToVolt(float val)
{
    return ((R_DIV1 + R_DIV2) * val) / R_DIV2;
}

/**
 * @brief Convert ADC value to measured amperage
 * @param 
 * val : ADC value in Volts
 * @retval None
 */
float Power::convertToAmp(float val)
{
    return (val * 1000) / MV_TO_AMP;
}

/**
 * @brief Convert ADC measured value to voltage
 * @param 
 * val_adc : ADC measured value
 * @retval None
 */
float Power::adcToVoltage(float val_adc)
{
    return val_adc * VOLT_SCALE;
}
