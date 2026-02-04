#ifndef DEF_PORTS_H_
#define DEF_PORTS_H_

//--------------------------------------------------------------------------------
//LEDs

#define GPIO_LED_STROBE             GPIOC
#define PIN_LED_STROBE              GPIO_PIN_13
#define LED_STROBE_SET(flag)        HAL_GPIO_WritePin(GPIO_LED_STROBE, PIN_LED_STROBE, flag)

#define GPIO_STROBE_OUT             GPIOA
#define PIN_STROBE_OUT              GPIO_PIN_6


//--------------------------------------------------------------------------------

#endif /* DEF_PORTS_H_ */
