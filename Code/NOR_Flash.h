#ifndef FLASH_H_
#define FLASH_H_

#include "stm32f4xx_hal.h"

typedef struct
{
    uint32_t sector;
    uint32_t start_addr;
    uint32_t size;
} flash_mem_map;

//*****************************************************************************
//
//*****************************************************************************
class Flash {
public:
	Flash();
	virtual ~Flash();

	bool writeData16(uint32_t addr, uint16_t *data, uint16_t dataSize);
    bool writeData32(uint32_t addr, uint32_t *data, uint16_t dataSize);
    bool eraseSectors(uint32_t sector, uint32_t num, uint32_t  *faultyPageAddr);
    void readData8(uint32_t addr, uint8_t *data, uint16_t dataSize);
    void readData32(uint32_t addr, uint32_t *data, uint16_t dataSize);
    void readData16(uint32_t addr, uint16_t *data, uint16_t dataSize);
    bool eraseAndWrite(uint32_t addr, uint32_t *data, uint16_t dataSize);

private:
    #define NUM_SECTORS 6
    const flash_mem_map flash_map[NUM_SECTORS] = { {FLASH_SECTOR_0, 0x08000000, (16 * 1024)},
                                                   {FLASH_SECTOR_1, 0x08004000, (16 * 1024)},
                                                   {FLASH_SECTOR_2, 0x08008000, (16 * 1024)},
                                                   {FLASH_SECTOR_3, 0x0800C000, (16 * 1024)},
                                                   {FLASH_SECTOR_4, 0x08010000, (64 * 1024)},
                                                   {FLASH_SECTOR_5, 0x08020000, (128 * 1024)} };

    uint32_t getSector(uint32_t addr);
};

//*****************************************************************************
//
//*****************************************************************************
class ConfigStore : private Flash
{
public:
    ConfigStore();
    virtual ~ConfigStore();

    config_t getConfig();
    void setConfig(config_t& config);
    uint32_t getID()
    {
        return cfg_id;
    }

private:
    static const uint32_t m_configAddress = 0x08020000;
    static const uint32_t cfg_id = 0x000055AA;
};

#endif /* FLASH_H_ */
