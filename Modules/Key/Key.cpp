/*
 * Key.cpp
 *
 *  Created on: Feb 9, 2026
 *      Author: Kitebuilder
 */

#include <cstdint>
#include "Key.h"

/**
  * @brief Key initialize routine
  * @param 
  *        gpio: pointer to GPIO_TypeDef structure
  *        pin:  gpio pin, should be adjusted as an input with pull-up
  *        p_handler: pointer to the key handler routine or NULL
  * @retval None
  */
void Key::Key_Init(GPIO_TypeDef* p_gpio, uint16_t pin, key_active_level_e level, p_key_handler p_handler)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    key_state = RELEASED;
    tremble_cnt = 0;
    guard_cnt = 0;
    gpio =  p_gpio;
    this->pin =  pin;
    this->level = level;
    this->p_handler = p_handler;

    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = (level == LO_LEVEL) ? GPIO_PULLUP : GPIO_PULLDOWN;
    HAL_GPIO_Init(gpio, &GPIO_InitStruct);
}

/**
  * @brief Check keys state
  * @param None
  * @retval None
  */
void Key::Key_CheckState()
{
    if (guard_cnt == 0) //during GUARD_DELAY key can't be detected as pressed or released
    {
        if ( HAL_GPIO_ReadPin(gpio, pin) ==  (level == LO_LEVEL ? GPIO_PIN_RESET : GPIO_PIN_SET) )
        {
            if (key_state == RELEASED)
            {
                if (++tremble_cnt == TREMBLE_THRESHOLD) //key should be detected as pressed this TREMBLE_THRESHOLD times to say that it's pressed
                {
                    tremble_cnt = 0;
                    key_state = PRESSED;
                    guard_cnt = GUARD_DELAY;
                    if (p_handler != NULL)
                    {
                        p_handler(key_state);
                    }
                }
            }
        }
        else
        {
            if (key_state == PRESSED)
            {
                if(++tremble_cnt == TREMBLE_THRESHOLD) //key should be detected as released this TREMBLE_THRESHOLD times to say that it's pressed
                {
                    tremble_cnt = 0;
                    key_state = RELEASED;
                    guard_cnt = GUARD_DELAY;
                    if (p_handler != NULL)
                    {
                        p_handler(key_state);
                    }
                }
            }
            else
            {
                tremble_cnt = 0;
            }
        }
    }
    else
    {
        --guard_cnt;
    }
}

/**
  * @brief Instant check the key state
  * @param None
  * @retval key_state_e
  */
key_state_e Key::Key_InstantCheck()
{
    if ( HAL_GPIO_ReadPin(gpio, pin) ==  (level == LO_LEVEL ? GPIO_PIN_RESET : GPIO_PIN_SET) )
    {
        return PRESSED;
    }
    else
    {
        return RELEASED;
    }
}
