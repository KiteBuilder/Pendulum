#include "app.h"
#include "NOR_Flash.h"

//*****************************************************************************
//Flash class with NORFlash primitives
//*****************************************************************************
/**
  * @brief Constructor
  * @param None
  * @retval None
  */
Flash::Flash() {
	// TODO Auto-generated constructor stub

}

/**
  * @brief Desstructor
  * @param None
  * @retval None
  */
Flash::~Flash() {
	// TODO Auto-generated destructor stub
}

/**
  * @brief Get sector according to address
  * @param addr - 32bit aligned address
  * @retval Sector
  */
uint32_t Flash::getSector(uint32_t addr)
{
    uint32_t sector = 0xFFFFFFFF;

    for (uint32_t i = 0; i < NUM_SECTORS; i++)
    {
        if (flash_map[i].start_addr >= addr)
        {
            sector = flash_map[i].sector;
            break;
        }
    }

    return sector;
}

/**
  * @brief Erase a sector and then write a data there
  * @param addr - 32bit aligned address
  *        data -  pointer to the 32bit data buffer
  *        dataSize - buffer size in bytes
  * @retval None
  */
bool Flash::eraseAndWrite(uint32_t addr, uint32_t *data, uint16_t dataSize)
{
	uint32_t faultySector = 0;
    uint32_t sector = getSector(addr);

    if (sector == 0xFFFFFFFF)
    {
        return false;
    }

    if (!eraseSectors(sector, 1, &faultySector))
    {
        return false;
    }
    
    if (!writeData32(addr, data, dataSize))
    {
        return false;
    }

	return true;
}

/**
  * @brief Write 16bit data array to the NOR Flash
  * @param addr - 16bit aligned address
  *        data -  pointer to the 16bit data buffer
  *        dataSize - buffer size in bytes
  * @retval true if programmed successfully
  */
bool Flash::writeData16(uint32_t addr, uint16_t *data, uint16_t dataSize)
{
    HAL_FLASH_Unlock();

     uint16_t n = dataSize / 2;

    for (uint16_t i = 0; i < n; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + (i << 1), data[i]) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return false;
        }
    }

    HAL_FLASH_Lock();
    return true;
}

/**
  * @brief Write 32bit data array to the NOR Flash
  * @param addr - 32bit aligned address
  *        data -  pointer to the 32bit data buffer
  *        dataSize - buffer size in bytes
  * @retval true if programmed successfully
  */
bool Flash::writeData32(uint32_t addr, uint32_t *data, uint16_t dataSize)
{
    HAL_FLASH_Unlock();

    uint16_t n = dataSize / 4;

    for (uint16_t i = 0; i < n; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + (i << 2), data[i]) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return false;
        }
    }

    HAL_FLASH_Lock();
    return true;
}

/**
  * @brief Erase NOR Flash sector/sectors
  * @param sector - sector from whitch erase starts
  *        num - number of sectors to be erased
  *        faultuSector - if erase failed the last sector should be stored there, set to NULL if not necessary
  * @retval None
  */
bool Flash::eraseSectors(uint32_t sector, uint32_t num, uint32_t *faultySector)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;
    bool status = false;

    HAL_FLASH_Unlock();

    EraseInitStruct.TypeErase   = TYPEERASE_SECTORS;
    EraseInitStruct.Sector = sector;
    EraseInitStruct.NbSectors = num;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;


    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) == HAL_OK)
    {
        status = true;
    }
    else
    {
        if (faultySector != NULL)
        {
            *faultySector = SectorError;
        }
    }

    HAL_FLASH_Lock();
    return status;
}

/**
  * @brief Read array of bytes
  * @param addr - address to start reading
  *        data -  pointer to the 8bit data buffer
  *        dataSize - buffer size in bytes
  * @retval None
  */
void Flash::readData8(uint32_t addr, uint8_t *data, uint16_t dataSize)
{
	uint32_t mem_addr;
	uint32_t mem_data_end;
	uint8_t  *data_ptr;

	mem_addr = addr;
	mem_data_end = addr + dataSize;
	data_ptr = data;

	while (mem_addr < mem_data_end)
	{
		*data_ptr = *(uint8_t*)mem_addr;
		mem_addr++;
		data_ptr++;
	}
}

/**
  * @brief Read array of uint16
  * @param addr - 16bit aligned address
  *        data -  pointer to the 16bit data buffer
  *        dataSize - buffer size in 16bit words
  * @retval None
  */
void Flash::readData16(uint32_t addr, uint16_t *data, uint16_t dataSize)
{
    uint32_t mem_addr;
    uint32_t mem_data_end;
    uint16_t  *data_ptr;

    mem_addr = addr;
    mem_data_end = addr + (dataSize << 1);
    data_ptr = data;

    while (mem_addr < mem_data_end)
    {
        *data_ptr = *(uint16_t*)mem_addr;
        mem_addr += 2;
        data_ptr++;
    }
}

/**
  * @brief Read array of uint32
  * @param addr - 32bit aligned address
  *        data -  pointer to the 32bit data buffer
  *        dataSize - buffer size in 32bit words
  * @retval None
  */
void Flash::readData32(uint32_t addr, uint32_t *data, uint16_t dataSize)
{
    uint32_t mem_addr;
    uint32_t mem_data_end;
    uint32_t  *data_ptr;

    mem_addr = addr;
    mem_data_end = addr + (dataSize << 2);
    data_ptr = data;

    while (mem_addr < mem_data_end)
    {
        *data_ptr = *(uint32_t*)mem_addr;
        mem_addr += 4;
        data_ptr++;
    }
}

//*****************************************************************************
//Config storage class
//*****************************************************************************
/**
  * @brief Constructor
  * @param None
  * @retval None
  */

ConfigStore::ConfigStore() {

}

/**
  * @brief Destructor
  * @param None
  * @retval None
  */
ConfigStore::~ConfigStore() {

}

/**
  * @brief Get config from the NOR Flash memory
  * @param None
  * @retval config_t structure
  */
config_t ConfigStore::getConfig()
{
    config_t config;

    readData8(m_configAddress, (uint8_t*)&config, sizeof(config_t));

    return config;
}

/**
  * @brief Save config to the NOR Flash
  * @param config_t type referenced variable
  * @retval None
  */
void ConfigStore::setConfig(config_t& config)
{
    eraseAndWrite(m_configAddress, (uint32_t*)&config, sizeof(config_t));
}
