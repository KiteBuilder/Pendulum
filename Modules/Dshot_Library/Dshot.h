#ifndef DSHOT_H_
#define DSHOT_H_

#include "stm32f4xx_hal.h"

typedef enum {
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEACON1,
    DSHOT_CMD_BEACON2,
    DSHOT_CMD_BEACON3,
    DSHOT_CMD_BEACON4,
    DSHOT_CMD_BEACON5,
    DSHOT_CMD_ESC_INFO, // V2 includes settings
    DSHOT_CMD_SPIN_DIRECTION_1,
    DSHOT_CMD_SPIN_DIRECTION_2,
    DSHOT_CMD_3D_MODE_OFF,
    DSHOT_CMD_3D_MODE_ON,
    DSHOT_CMD_SETTINGS_REQUEST, // Currently not implemented
    DSHOT_CMD_SAVE_SETTINGS,
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
    DSHOT_CMD_LED0_ON, // BLHeli32 only
    DSHOT_CMD_LED1_ON, // BLHeli32 only
    DSHOT_CMD_LED2_ON, // BLHeli32 only
    DSHOT_CMD_LED3_ON, // BLHeli32 only
    DSHOT_CMD_LED0_OFF, // BLHeli32 only
    DSHOT_CMD_LED1_OFF, // BLHeli32 only
    DSHOT_CMD_LED2_OFF, // BLHeli32 only
    DSHOT_CMD_LED3_OFF, // BLHeli32 only
    DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF = 30, // KISS audio Stream mode on/Off
    DSHOT_CMD_SILENT_MODE_ON_OFF = 31, // KISS silent Mode on/Off
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE = 32,
    DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY = 33,
    DSHOT_CMD_MAX = 47
} dshotCommands_e;

#define DSHOT_PACKET_SIZE     16

#define DSHOT_MIN_THROTTLE    48
#define DSHOT_MAX_THROTTLE    2047

#define MOTOR_DSHOT600_HZ     600000
#define MOTOR_DSHOT300_HZ     300000
#define MOTOR_DSHOT150_HZ     150000

#define DSHOT_MOTOR_BIT_0     7
#define DSHOT_MOTOR_BIT_1     14
#define DSHOT_MOTOR_BITLENGTH 20

#define DSHOT_DMA_BUFFER_SIZE 18 /* resolution + frame reset (2us) */

class Dshot{
    TIM_HandleTypeDef *p_htim;
    uint32_t tim_channel;

    uint32_t dmaBuffer[DSHOT_DMA_BUFFER_SIZE];

    void loadDmaBuffer(uint16_t packet);
    uint16_t assemblePacket(uint16_t value, bool f_telemetry);

public:

    void Initialize(uint32_t dshot_freq_hz);
    void SendThrottle(uint16_t throttle, bool f_telemetry);
    void SendCommand(dshotCommands_e cmd, bool f_telemetry);

    Dshot(TIM_HandleTypeDef *htim, uint32_t channel);
    ~Dshot() = default;

};


#endif /*DSHOT_H_*/