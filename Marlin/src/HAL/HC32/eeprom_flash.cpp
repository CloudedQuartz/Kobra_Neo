/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
 * Copyright (c) 2016 Victor Perez victor_pv@hotmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#ifdef ARDUINO_ARCH_HC32

#include "../../inc/MarlinConfig.h"

#if ENABLED(FLASH_EEPROM_EMULATION)

#include "../shared/eeprom_api.h"

// use 256k to improve compatibility
// 32 sectors, 8k bytes per sector
#define FLASH_SECTOR_TOTAL    32
#define FLASH_SECTOR_SIZE     ((uint32_t)(8*1024U))
#define FLASH_ALL_START       0
#define FLASH_ALL_END         ((uint32_t)0x0003FFFFU)

// use last sector to emulate eeprom
#define FLASH_EEPROM_BASE     ((uint32_t)0x0007E000U)


#define FLASH_BASE            ((uint32_t)0x0007E000U)              /*!< FLASH base address in the alias region */

// just use 1k bytes, not a full sector
#define EEPROM_SIZE           1024


// power outage
#define FLASH_OUTAGE_DATA_ADDR  ((uint32_t)0x0007C000U)



/**
 * The STM32 HAL supports chips that deal with "pages" and some with "sectors" and some that
 * even have multiple "banks" of flash.
 *
 * This code is a bit of a mashup of
 *   framework-arduinoststm32/cores/arduino/stm32/stm32_eeprom.c
 *   hal/hal_lpc1768/persistent_store_flash.cpp
 *
 * This has only be written against those that use a single "sector" design.
 *
 * Those that deal with "pages" could be made to work. Looking at the STM32F07 for example, there are
 * 128 "pages", each 2kB in size. If we continued with our EEPROM being 4Kb, we'd always need to operate
 * on 2 of these pages. Each write, we'd use 2 different pages from a pool of pages until we are done.
 */

 static uint8_t eeprom_buffer[EEPROM_SIZE] __attribute__((aligned(8))) = {0};


static bool eeprom_data_written = false;

size_t PersistentStore::capacity()
{
    return MARLIN_EEPROM_SIZE;
}

bool PersistentStore::access_start() {
    memcpy(eeprom_buffer, (uint8_t *)(FLASH_EEPROM_BASE), sizeof(eeprom_buffer));
    return true;
}

bool PersistentStore::access_finish() {
  if (eeprom_data_written) {
      TERN_(HAS_PAUSE_SERVO_OUTPUT, PAUSE_SERVO_OUTPUT());
      DISABLE_ISRS();

      en_result res = Error;
      uint32_t addr = FLASH_EEPROM_BASE;
      uint32_t addr_end = FLASH_EEPROM_BASE + EEPROM_SIZE;
      uint32_t offset = 0;
  
      EFM_Unlock();
      EFM_FlashCmd(Enable);
  
      while(Set != EFM_GetFlagStatus(EFM_FLAG_RDY));
  
      res = EFM_SectorErase(FLASH_EEPROM_BASE);
      if(res != Ok) {
          printf("Error, func: %s, line: %d.\n", __FUNCTION__, __LINE__);
      }
  
      while(addr < addr_end) {
          res = EFM_SingleProgram(addr, *((uint32_t *)((uint8_t *)eeprom_buffer + offset)));
          if(res != Ok) {
              printf("Error, func: %s, line: %d.\n", __FUNCTION__, __LINE__);
              break;
          }
          addr += 4;
          offset += 4;
      }
  
      EFM_Lock();



      ENABLE_ISRS();
      TERN_(HAS_PAUSE_SERVO_OUTPUT, RESUME_SERVO_OUTPUT());
      eeprom_data_written = false;
  }
  return true;
}

bool PersistentStore::write_data(int &pos, const uint8_t *value, size_t size, uint16_t *crc) {
  while (size--) {
    uint8_t v = *value;
      if (v != eeprom_buffer[pos]) {
        eeprom_buffer[pos] = v;
        eeprom_data_written = true;
      }
    crc16(crc, &v, 1);
    pos++;
    value++;
  }
  return false;
}

bool PersistentStore::read_data(int &pos, uint8_t *value, size_t size, uint16_t *crc, const bool writing/*=true*/) {
  do {
    const uint8_t c = TERN(FLASH_EEPROM_LEVELING, ram_eeprom[pos], eeprom_buffer[pos]);
    if (writing) *value = c;
    crc16(crc, &c, 1);
    pos++;
    value++;
  } while (--size);
  return false;
}

// uint32_t PersistentStore::FLASH_If_Erase(uint32_t addr_start, uint32_t addr_end) {
//     Intflash::FlashErasePage(addr_start);
//     return(0);
// }

// uint32_t PersistentStore::FLASH_If_Write(uint32_t destination, const void *p_source, uint32_t length) {
//   return (Intflash::Flash_Updata(destination,p_source,length));
// }





#endif // FLASH_EEPROM_EMULATION
#endif // ARDUINO_ARCH_STM32
