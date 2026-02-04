#ifndef ATOMIC_H_
#define ATOMIC_H_

// cleanup BASEPRI restore function, with global memory barrier
static inline void __basepriRestoreMem(uint8_t *val)
{
    __set_BASEPRI(*val);
}

// set BASEPRI_MAX function, with global memory barrier, returns true
static inline uint8_t __basepriSetMemRetVal(uint8_t prio)
{
    __set_BASEPRI_MAX(prio);
    return 1;
}


#define ATOMIC_BLOCK(prio) for ( uint8_t __basepri_save __attribute__((__cleanup__(__basepriRestoreMem))) = __get_BASEPRI(), \
                                     __ToDo = __basepriSetMemRetVal((prio) << (8U - __NVIC_PRIO_BITS)); __ToDo ; __ToDo = 0 )

#define NVIC_PRIO_MAX                       1
#define NVIC_PRIO_I2C_ER                    2
#define NVIC_PRIO_I2C_EV                    2
#define NVIC_PRIO_TIMER                     3
#define NVIC_PRIO_TIMER_DMA                 3
#define NVIC_PRIO_SDIO                      3
#define NVIC_PRIO_USB                       5
#define NVIC_PRIO_SERIALUART                5
#define NVIC_PRIO_VCP                       7

#endif /* ATOMIC_H_ */
