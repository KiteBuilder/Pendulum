#include <cstdint>
#include <cstddef>
#include <cmath>
#include "Dshot.h"
#include "times.h"

/**
  * @brief Dshot class constructor
  * @param None
  * @retval None
  */
Dshot::Dshot(TIM_HandleTypeDef *htim, uint32_t channel) : p_htim(htim), tim_channel(channel)
{

}

/**
  * @brief Dshot class constructor
  * @param dshot_freq_hz : DSHOT one bit frequency in Hz
  * @retval None
  */
void Dshot::Initialize(uint32_t dshot_freq_hz)
{
    uint32_t freq = HAL_RCC_GetPCLK1Freq() << 1;
    //uint16_t  prescaler = freq / (DSHOT_MOTOR_BITLENGTH * dshot_freq_hz);
    uint16_t  prescaler = lrintf((float)freq / (DSHOT_MOTOR_BITLENGTH * dshot_freq_hz) + 0.01f);

    __HAL_TIM_SET_PRESCALER(p_htim, prescaler - 1);
    __HAL_TIM_SET_AUTORELOAD(p_htim, DSHOT_MOTOR_BITLENGTH - 1);
}

/**
  * @brief Send throttle value to ESC
  * @param 
  * throttle : 0 - 2000 throttle value, would be transfered to value from 48 to 2047, 2000 steps
  * f_telemtry : true - telemetry request, fale - no telemetry
  * @retval None
  */
void Dshot::SendThrottle(uint16_t throttle, bool f_telemetry)
{
    throttle += DSHOT_MIN_THROTTLE;
    if (throttle > DSHOT_MAX_THROTTLE)
    {
        throttle = DSHOT_MAX_THROTTLE;
    }

    uint16_t packet = assemblePacket(throttle, f_telemetry);
    loadDmaBuffer(packet);

    HAL_TIM_PWM_Start_DMA(p_htim, tim_channel, dmaBuffer, DSHOT_DMA_BUFFER_SIZE);
}

/**
  * @brief Send command to ESC
  * @param 
  * cmd : command value from dshotCommands_e enum list
  * f_telemtry : true - telemetry request, fale - no telemetry
  * @retval None
  */
void Dshot::SendCommand(dshotCommands_e cmd, bool f_telemetry)
{
    uint16_t packet = assemblePacket(cmd, f_telemetry);
    loadDmaBuffer(packet);

    HAL_TIM_PWM_Start_DMA(p_htim, tim_channel, dmaBuffer, DSHOT_DMA_BUFFER_SIZE);
}

/**
  * @brief Load DMA buffer with a packet data 
  * @param packet : bit field where MSB should be transferred first
  * @retval None
  */
void Dshot::loadDmaBuffer(uint16_t packet)
{
    for (uint32_t i = 0; i < DSHOT_PACKET_SIZE; i++) 
    {
        dmaBuffer[i] = (packet & 0x8000) ? DSHOT_MOTOR_BIT_1 : DSHOT_MOTOR_BIT_0;  // MSB first
        packet <<= 1;
    }

    dmaBuffer[16] = 0;
    dmaBuffer[17] = 0;
}

/**
  * @brief Prepare and assemble Dshot DMA packet
  * @param 
  * value : throttle value from DSHOT_MIN_THROTTLE to DSHOT_MAX_THROTTLE
  * f_telemetry : true - telemetry request, fale - no telemetry
  * @retval None
  */
uint16_t Dshot::assemblePacket(uint16_t value, bool f_telemetry)
{
    uint16_t packet = (value << 1) | (f_telemetry ? 1 : 0);

    // Compute checksum
    uint16_t check_sum = 0;
    uint16_t data = packet;

    for (int i = 0; i < 3; i++) {
        check_sum ^=  data & 0x000F;   // xor data by nibbles
        data >>= 4;
    }

    check_sum &= 0x000F;

    // Apply checksum
    packet = (packet << 4) | check_sum;

    return packet;
    
}