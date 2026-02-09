/*
 * Key.h
 *
 *  Created on: Feb 9, 2026
 *      Author: Kitebuilder
 */

#ifndef KEY_H_
#define KEY_H_

#include <stm32f4xx_hal.h>

typedef enum _key_state_ {RELEASED = 0, PRESSED = 1} key_state_e;

typedef enum _key_active_level_ {LO_LEVEL = 0, HI_LEVEL = 1} key_active_level_e;

#define GUARD_DELAY   10 //Depends on the polling period. The guard time designates the time period during which the button can't be pressed or released
#define TREMBLE_THRESHOLD 3 //Key can be detected as pressed or released if it detected in this state TREMBLE_THRESHOLD times

typedef void (*p_key_handler)(key_state_e);

class Key{
    key_state_e key_state;
    uint8_t tremble_cnt;
    uint8_t guard_cnt;

    GPIO_TypeDef* gpio;
    uint16_t pin;
    key_active_level_e level;

    p_key_handler p_handler;

public:

    void Key_Init(GPIO_TypeDef*, uint16_t, key_active_level_e, p_key_handler);
    void Key_CheckState();
    key_state_e Key_InstantCheck();

    Key() = default;
    ~Key() = default;    
};


#endif /* KEY_H_ */
