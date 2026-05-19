#include <cstdio>
#include <cmath>
#include "app.h"
#include "times.h"
#include "atomic.h"
#include "Scheduler.h"
#include "def_ports.h"
#include "model_config.h"
#include "SSD1306.h"

#define SSD1306_INCLUDE_FONT_6x8
#define SSD1306_INCLUDE_FONT_11x18
#include "SSD1306_Fonts.h"

#include "mpu6050.h"
#include "Kalman.h"
#include "Dshot.h"
#include "Power.h"
#include "Key.h"
#include "PID.h"
#include "mixer.h"
#include "NOR_Flash.h"

using namespace std;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;
extern DMA_HandleTypeDef hdma_i2c3_tx;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
extern ADC_HandleTypeDef hadc1;

#ifdef SSD1306_INCLUDE_FONT_6x8
SSD1306_FontDef Font_6x8 = {6, 8, Font6x8, 32, 126};
#endif

#ifdef SSD1306_INCLUDE_FONT_11x18
SSD1306_FontDef Font_11x18 = {11, 18, Font11x18, 32, 126};
#endif

SSD1306_I2C_Display *SSD1306_I2C_Display::p_instance = new SSD1306_I2C_Display(&hi2c3);
SSD1306_I2C_Display *p_i2c_display = SSD1306_I2C_Display::getInstance();

MPU6050_I2C *MPU6050_I2C::p_instance = new MPU6050_I2C(&hi2c1);
MPU6050_I2C *p_i2c_mpu6050 = MPU6050_I2C::getInstance();

float gyroData[XYZ_AXIS_COUNT] = {0.0f, 0.0f, 0.0f};
float accData[XYZ_AXIS_COUNT] = {0.0f, 0.0f, 0.0f};

Kalman rollKalman, pitchKalman;
float rollAngle = 0.0f, pitchAngle = 0.0f;
timeDelta_t previousTimeUs = 0;

Dshot motorLeft(&htim3, TIM_CHANNEL_4);
Dshot motorRight(&htim3, TIM_CHANNEL_3);

uint16_t throttle_right = 0;
uint16_t throttle_left = 0;

bool f_need_arm = true;
#define ARM_PERIOD_US  (2500 * 1000)
uint32_t arm_delay_us = 0;

Power power(&hadc1);

#ifdef DEBUG_ENABLED
    extern UART_HandleTypeDef huart1;
    uint8_t rxByte;
#endif

Key start_stop_key;
static void StartStop_Key_Handler(key_state_e);
bool f_start = false;

PID pid;
Mixer mixer;

ConfigStore pid_config;

static void update_lcd_display(void);
static void update_gyro(void);
static void kalman_setup(void);
static void kalman_loop(timeDelta_t dT);

#if 0
static void motors_arm(void);
static void motors_disarm(void);
#endif

static void taskSTROBE(timeUs_t);
static void taskGYRO(timeUs_t);
static void taskPID(timeUs_t);
static void taskDISPLAY(timeUs_t);
static void taskPOWER(timeUs_t);
static void taskDEBUG(timeUs_t);
static void taskKEY(timeUs_t);


//Task array, consists of all scheduled tasks
task_t tasks[TASK_COUNT] = {
        [TASK_STROBE] = {
                .name = "LED_STROBE",
                .taskHandler = taskSTROBE,
                .taskPeriod = TASK_PERIOD_US(1000), //1000 microseconds, 1KHz
                .taskEnabled = false,
        },

        [TASK_GYRO] = {
                .name = "GYRO_SENSOR",
                .taskHandler = taskGYRO,
                .taskPeriod = TASK_PERIOD_HZ(GYRO_RATE_HZ), //1000 microseconds, 1000Hz
                .taskEnabled = false,
        },

        [TASK_PID_CONTROL] = {
                .name = "PID_CONTROLLER",
                .taskHandler = taskPID,
                .taskPeriod = TASK_PERIOD_HZ(PID_LOOP_HZ), //2000 microseconds, 500Hz
                .taskEnabled = false,
        },

        [TASK_DISPLAY] = {
                .name = "LCD_DISPLAY",
                .taskHandler = taskDISPLAY,
                .taskPeriod = TASK_PERIOD_MS(250), //250 milliseconds, 4Hz
                .taskEnabled = false,
        },
        [TASK_POWER] = {
                .name = "POWER_CONSUMPTION",
                .taskHandler = taskPOWER,
                .taskPeriod = TASK_PERIOD_HZ(50), //20 milliseconds, 50Hz
                .taskEnabled = false,
        },        
#ifdef DEBUG_ENABLED
        [TASK_DEBUG] = {
                .name = "DEBUG",
                .taskHandler = taskDEBUG,
                .taskPeriod = TASK_PERIOD_HZ(50), //50Hz, 20ms
                .taskEnabled = false,
        },
#endif
        [TASK_KEY] = {
                .name = "KEY_POLLING",
                .taskHandler = taskKEY,
                .taskPeriod = TASK_PERIOD_MS(10), //10 milliseconds, 100Hz
                .taskEnabled = false,
        }, 
};

TasksQueue taskQueue(tasks, TASK_COUNT);

/**
  * @brief
  * @param None
  * @retval None
  */
void initialization(void)
{
    ATOMIC_BLOCK(NVIC_PRIO_MAX)
    {
        usTicks = SystemCoreClock / 1000000;
    }

	HAL_Delay(100);

	taskQueue.taskEnable(TASK_STROBE);
    taskQueue.taskEnable(TASK_GYRO);
    taskQueue.taskEnable(TASK_PID_CONTROL);
    taskQueue.taskEnable(TASK_DISPLAY);
    taskQueue.taskEnable(TASK_POWER);
#ifdef DEBUG_ENABLED
    HAL_UART_Receive_IT(&huart2, &rxByte, 1);
    taskQueue.taskEnable(TASK_DEBUG);
#endif
    taskQueue.taskEnable(TASK_KEY);

    start_stop_key.Key_Init(GPIOA, GPIO_PIN_0, LO_LEVEL, &StartStop_Key_Handler);

    p_i2c_display->ssd1306_Init();

    p_i2c_mpu6050->mpu6050_Init(GYRO_LPF_TYPE, GYRO_RATE_HZ, GYRO_SCALE, ACC_SCALE);

    motorLeft.Initialize(MOTOR_DSHOT300_HZ);
    motorRight.Initialize(MOTOR_DSHOT300_HZ);

    config_t cfg = pid_config.getConfig();

    if (cfg.id != pid_config.getID())
    {
        cfg.id = pid_config.getID();
        cfg.pid_P = PID_P;
        cfg.pid_I = PID_I;
        cfg.pid_D = PID_D;
        pid_config.setConfig(cfg);
    }

    pid.Initialize(cfg.pid_P, cfg.pid_I, cfg.pid_D, PID_LOOP_HZ, PIDSUM_MAX);
    mixer.Initialize(0, PIDSUM_MAX, DSHOT_MIN, DSHOT_MAX, DSHOT_MID);
}

/**
  * @brief
  * @param None
  * @retval None
  */
void exec(void)
{
    taskQueue.scheduler();
}

/**
  * @brief
  * @param
  * @retval
  */
static void taskSTROBE(timeUs_t currentTimeUs)
{
    static uint32_t strbCnt = 0;

    UNUSED(currentTimeUs);
    switch (strbCnt)
    {
        case 0:
            LED_STROBE_SET(GPIO_PIN_RESET);
            break;

        case 50:
            LED_STROBE_SET(GPIO_PIN_SET);
            break;

        case 1000:
            strbCnt = 0;
            return;
    }

    ++strbCnt;
}

/**
  * @brief
  * @param
  * @retval
  */
static void taskGYRO(timeUs_t currentTimeUs)
{
    update_gyro();

    if (f_need_arm)
    { 
        motorLeft.SendThrottle(0, false);
        motorRight.SendThrottle(0, false);
    }

    HAL_GPIO_TogglePin(GPIO_STROBE_OUT, PIN_STROBE_OUT);
}

/**
  * @brief
  * @param
  * @retval
  */
static void taskPID(timeUs_t currentTimeUs)
{
    timeDelta_t dT = currentTimeUs - previousTimeUs;
    previousTimeUs = currentTimeUs;    

    if (!p_i2c_mpu6050->mpu6050_IsCalibrate())
    {
        kalman_loop(dT);
    }

    if (f_need_arm)
    { 
        arm_delay_us += dT;
        if (arm_delay_us >= ARM_PERIOD_US)
        {
            f_need_arm = false;
            arm_delay_us = 0;
        }
    }
    else
    {
        float pidsum = pid.Compute(SET_POINT, rollAngle, US2S(dT));
        mixer.Compute(pidsum, throttle_left, throttle_right);

        if (f_start)
        {
            motorLeft.SendThrottle(throttle_left, false);
            motorRight.SendThrottle(throttle_right, false);
        }
        else
        {
            motorLeft.SendThrottle(0, false);
            motorRight.SendThrottle(0, false);
        }
    }
}

/**
  * @brief
  * @param
  * @retval
  */
static void taskDISPLAY(timeUs_t currentTimeUs)
{
    update_lcd_display();
}

/**
  * @brief
  * @param
  * @retval
  */
static void taskPOWER(timeUs_t currentTimeUs)
{
    power.StartADC(currentTimeUs);
}

#ifdef DEBUG_ENABLED
//*****************************************************************************
//Here part for sending debug info with the UART
//*****************************************************************************
#define DLE 0x10
#define ETX 0x03
#define ID 0x01

#define DEBUG_PACK_SIZE 128
uint8_t debugBuff[DEBUG_PACK_SIZE];

#define PACK_SIZE 16

#pragma pack(push, 1)
typedef union
{
    float flt;
    uint8_t bt[sizeof(float)];
} byte_float_t;
#pragma pack(pop)

byte_float_t debugPack[PACK_SIZE];

bool f_TxReady = true;
bool f_RxReady = false;

/**
  * @brief Debug task sends selected amount of values by the serial interface
  *        and uses simple protocol that consists of next fields:
  *        |DLE|ID|DATA|DLE|ETX| where DLE symbol in the DATA field
  *        should be doubled
  * @param
  * @retval
  */
static void taskDEBUG(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (f_RxReady == true)
    {
        f_RxReady = false;

        if (rxByte == 'R')
        {
            //TODO
        }
    }

    debugPack[0].flt = power.GetIBat();
    debugPack[1].flt = power.GetVBat();
    debugPack[2].flt = power.GetEBat();
    debugPack[3].flt = SET_POINT;
    debugPack[4].flt = rollAngle;
    debugPack[5].flt = pid.Get_Error();
    debugPack[6].flt = pid.Get_P();
    debugPack[7].flt = pid.Get_I();
    debugPack[8].flt = pid.Get_D();
    debugPack[9].flt = throttle_left * 1.0f;
    debugPack[10].flt = throttle_right * 1.0f;
    debugPack[11].flt = 0;
    debugPack[12].flt = 0;
    debugPack[13].flt = 0;
    debugPack[14].flt = 0;
    debugPack[15].flt = 0;

    uint32_t n = 0;
    debugBuff[n++] = DLE;
    debugBuff[n++] = ID;

    for (uint32_t i = 0; i < PACK_SIZE; i++)
    {
        for (uint32_t j = 0; j < sizeof(byte_float_t); j++)
        {
           debugBuff[n++] = debugPack[i].bt[j];

           if (debugPack[i].bt[j] == DLE)
           {
               debugBuff[n++] = DLE;
           }
        }
    }

    debugBuff[n++] = DLE;
    debugBuff[n++] = ETX;

    do
    {
        continue;
    }while(!f_TxReady);

    f_TxReady = false;
    HAL_UART_Transmit_IT(&huart2, debugBuff, n);
}

/**
  * @brief 
  * @param
  * @retval
  */
static void taskKEY(timeUs_t currentTimeUs)
{
    start_stop_key.Key_CheckState();
}

/**
  * @brief
  * @param
  * @retval
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
        f_TxReady = true;
    }
}

/**
  * @brief
  * @param
  * @retval
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
        f_RxReady = true;
        HAL_UART_Receive_IT(huart, &rxByte, 1);
    }
}

#endif

/**
  * @brief 
  * @param None
  * @retval None
  */
static void update_lcd_display(void)
{
    char str_buff[256];
    float tempC;
    uint32_t data[2];
    char sign[2];
    
    p_i2c_mpu6050->mpu6050_GetTempC(&tempC);

    p_i2c_display->ssd1306_Fill(Black);

    p_i2c_display->ssd1306_SetCursor(5, 0);
    data[0] = (uint32_t)(tempC * 10);
    //sprintf(str_buff, "tC = %4.1f", tempC);
    sprintf(str_buff, "tC = %2lu.%1lu", data[0] / 10, data[0] % 10);
    p_i2c_display->ssd1306_WriteString(str_buff, Font_11x18, White);

    if (!p_i2c_mpu6050->mpu6050_IsCalibrate())
    {
        if (!f_start)
        {
            p_i2c_display->ssd1306_SetCursor(2, 25);
            data[0] = (uint32_t)(fabs(rollAngle) * 10);
            sign[0] = (rollAngle < 0.0f) ? '-' : ' ';
            data[1] = (uint32_t)(fabs(pitchAngle) * 10);
            sign[1] = (pitchAngle < 0.0f) ? '-' : ' ';
            //sprintf(str_buff, "R=%6.1f P=%6.1f", rollAngle, pitchAngle);
            sprintf(str_buff, "R=%c%3lu.%1lu P=%c%3lu.%1lu", sign[0], data[0] / 10, data[0] % 10, sign[1], data[1] / 10, data[1] % 10); 
            p_i2c_display->ssd1306_WriteString(str_buff, Font_6x8, White);

            p_i2c_display->ssd1306_SetCursor(2, 40);
            data[0] = (uint32_t)(power.GetVBat() * 10);
            data[1] = (uint32_t)(power.GetIBat() * 10);
            //sprintf(str_buff, "V=%4.1f I=%4.1f", power.GetVBat(), power.GetIBat());
            sprintf(str_buff, "V=%2lu.%1lu I=%2lu.%1lu", data[0] / 10, data[0] % 10, data[1] / 10, data[1] % 10);
            p_i2c_display->ssd1306_WriteString(str_buff, Font_6x8, White);

            p_i2c_display->ssd1306_SetCursor(2, 55);
            data[0] = (uint32_t)(power.GetEBat() * 10);
            //sprintf(str_buff, "E=%6.1fmAh", power.GetEBat());
            sprintf(str_buff, "E=%4lu.%1lu", data[0] / 10, data[0] % 10);
            p_i2c_display->ssd1306_WriteString(str_buff, Font_6x8, White);
        }
        else
        {
            p_i2c_display->ssd1306_SetCursor(2, 25);
            data[0] = (uint32_t)(fabs(rollAngle) * 10);
            sign[0] = (rollAngle < 0.0f) ? '-' : ' ';
            //sprintf(str_buff, "R=%6.1f", rollAngle);
            sprintf(str_buff, "R=%c%3lu.%1lu", sign[0], data[0] / 10, data[0] % 10);
            p_i2c_display->ssd1306_WriteString(str_buff, Font_6x8, White);

            p_i2c_display->ssd1306_SetCursor(2, 40);
            data[0] = (uint32_t)(fabs(pid.Get_P()) * 10);
            sign[0] = (pid.Get_P() < 0.0f) ? '-' : ' ';            
            //sprintf(str_buff, "P=%8.3f", pid.Get_P());
            sprintf(str_buff, "P=%c%4lu.%1lu", sign[0], data[0] / 10, data[0] % 10);
            p_i2c_display->ssd1306_WriteString(str_buff, Font_6x8, White);

            p_i2c_display->ssd1306_SetCursor(2, 55);
            data[0] = (uint32_t)(fabs(pid.Get_D()) * 10);
            sign[0] = (pid.Get_D() < 0.0f) ? '-' : ' ';
            //sprintf(str_buff, "D=%8.3f", pid.Get_D());
            sprintf(str_buff, "D=%c%4lu.%1lu", sign[0], data[0] / 10, data[0] % 10);
            p_i2c_display->ssd1306_WriteString(str_buff, Font_6x8, White);
        }
    }
    else
    {
        p_i2c_display->ssd1306_SetCursor(2, 25);
        sprintf(str_buff, "...Gyro Calibrate...");
        p_i2c_display->ssd1306_WriteString(str_buff, Font_6x8, White);        
    }

    p_i2c_display->ssd1306_UpdateScreenDMA();
}

/**
  * @brief 
  * @param None
  * @retval None
  */
static void update_gyro(void)
{
    p_i2c_mpu6050->mpu6050_ReadAccTempGyro();

    if (p_i2c_mpu6050->mpu6050_IsCalibrate())
    {
        p_i2c_mpu6050->mpu6050_Calibrate();

        if (!p_i2c_mpu6050->mpu6050_IsCalibrate())
        {
            kalman_setup();
        }
    }
}

/**
  * @brief
  * @param None
  * @retval None
  */
static void kalman_setup(void)
{
    p_i2c_mpu6050->mpu6050_GetGyroDataFilt(gyroData);
    p_i2c_mpu6050->mpu6050_GetAccDataFilt(accData);

    float roll = atan2(accData[Y], accData[Z]) * (180 / M_PI);
    float pitch = atan2 (-accData[X], sqrt(accData[Y] * accData[Y] + accData[Z] * accData[Z])) * (180 / M_PI);

    rollKalman.setAngle(roll); // Set starting angle
    pitchKalman.setAngle(pitch);

    previousTimeUs = micros();
}

/**
  * @brief 
  * @param None
  * @retval None
  */
static void kalman_loop(timeDelta_t dT)
{
    p_i2c_mpu6050->mpu6050_GetGyroDataFilt(gyroData);
    p_i2c_mpu6050->mpu6050_GetAccDataFilt(accData);

    float roll = atan2(accData[Y], accData[Z]) * (180 / M_PI);
    float pitch = atan2 (-accData[X], sqrt(accData[Y] * accData[Y] + accData[Z] * accData[Z])) * (180 / M_PI);
    
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && rollAngle > 90) || (roll > 90 && rollAngle < -90)) 
    {
        rollKalman.setAngle(roll);
        rollAngle = roll;
    } 
    else
    {
        rollAngle = rollKalman.getAngle(roll, gyroData[X], US2S(dT)); // Calculate the angle using a Kalman filter
    }

    if (abs(roll) > 90)
    {
        gyroData[Y] = -gyroData[Y]; // Invert rate, so it fits the restriced accelerometer reading
    }

    pitchAngle = pitchKalman.getAngle(pitch, gyroData[Y], US2S(dT));    
}

#if 0
/**
  * @brief Motors arming process
  * @param None
  * @retval None
  */
static void motors_arm(void)
{
    for (uint32_t i = 0;  i < 1500; i++)
    {
        motorLeft.SendThrottle(0, false);
        motorRight.SendThrottle(0, false);
        delayUs(1000);
    }
}

/**
  * @brief Motors disarm
  * @param None
  * @retval None
  */
static void motors_disarm(void)
{
    motorLeft.SendCommand(DSHOT_CMD_MOTOR_STOP, false);
    motorRight.SendCommand(DSHOT_CMD_MOTOR_STOP, false);
}
#endif

/**
  * @brief 
  * @param None
  * @retval None
  */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    p_i2c_display->ssd1306_UpdateScreenDMA();
}

/**
  * @brief 
  * @param None
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    power.HandleADC();
}

/**
  * @brief 
  * @param None
  * @retval None
  */
static void StartStop_Key_Handler(key_state_e state)
{
    if (state == PRESSED)
    {
        f_start = (!f_start) ? true : false;
    }
}
