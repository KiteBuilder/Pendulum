#ifndef APP_H_
#define APP_H_

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void initialization(void);
void exec(void);

#define DEBUG_ENABLED

#ifdef __cplusplus
}
#endif

#endif /* APP_H_ */
