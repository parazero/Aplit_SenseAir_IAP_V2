// Module..: IAP.C
// Chip....: STM32F4xx
//-----------------------------------------------------------------------------

#include "includes.h"
#include "iap.h"
//-----------------------------------------------------------------------------

U32 iap_sector_index;
//-----------------------------------------------------------------------------

void iap_prepare_flash(void)
{
// FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
//                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
 /* Clear pending flags (if any) */
 __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                        FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);


 HAL_FLASH_Unlock();
}
//-----------------------------------------------------------------------------

U32 iap_erase_sector(U32 addr)
{
	  uint32_t UserStartSector;
	  uint32_t SectorError;
	  FLASH_EraseInitTypeDef pEraseInit;
//  U32 sector;
//  sector = iap_get_sector(addr);

  iap_prepare_flash();
  UserStartSector = iap_get_sector(addr);
  pEraseInit.TypeErase = TYPEERASE_SECTORS;
  pEraseInit.Sector = UserStartSector;
  pEraseInit.NbSectors =  1;
  pEraseInit.VoltageRange = VOLTAGE_RANGE_3;

//  if (FLASH_EraseSector(sector, VoltageRange_3) != FLASH_COMPLETE)
//  {
//    // Error occurred while page erase
//    return (1);
//  }
  if (HAL_FLASHEx_Erase(&pEraseInit, &SectorError) != HAL_OK)
  {
     return (1);
  }

  return (0);
}
//-----------------------------------------------------------------------------

U32 iap_write_sector(U32 addr, pU32 data, U32 data_length)
{
  U32 n;

  iap_prepare_flash();

  for (n = 0; (n < data_length); n++)
  {
//     if (FLASH_ProgramWord(addr, *data) == FLASH_COMPLETE)
     if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, *(uint32_t*)data) == HAL_OK)
    {
      if (*(U32*)addr != *data++)
      {
        return (2);
      }

      addr += 4;
    }
    else
    {
      return (1);
    }
  }

  return (0);
}
//-----------------------------------------------------------------------------

U32 iap_verify(pU32 buffer1, pU32 buffer2, U32 data_length)
{
  U32 n;

  for (n = 0; (n < data_length); n++)
  {
    if (*buffer1++ != *buffer2++)
    {
      return (1);
    }
  }

  return (0);
}
//-----------------------------------------------------------------------------

U32 iap_get_sector(U32 addr)
{
/*  U32 sector = 0;

  if ((addr >= IAP_SECTOR_ADDR_0) && (addr < IAP_SECTOR_ADDR_1))
  {
    sector = FLASH_Sector_0;
    iap_sector_index = 0;
  }
  else if ((addr >= IAP_SECTOR_ADDR_1) && (addr < IAP_SECTOR_ADDR_2))
  {
    sector = FLASH_Sector_1;
    iap_sector_index = 1;
  }
  else if ((addr >= IAP_SECTOR_ADDR_2) && (addr < IAP_SECTOR_ADDR_3))
  {
    sector = FLASH_Sector_2;
    iap_sector_index = 2;
  }
  else if ((addr >= IAP_SECTOR_ADDR_3) && (addr < IAP_SECTOR_ADDR_4))
  {
    sector = FLASH_Sector_3;
    iap_sector_index = 3;
  }
  else if ((addr >= IAP_SECTOR_ADDR_4) && (addr < IAP_SECTOR_ADDR_5))
  {
    sector = FLASH_Sector_4;
    iap_sector_index = 4;
  }
  else if ((addr >= IAP_SECTOR_ADDR_5) && (addr < IAP_SECTOR_ADDR_6))
  {
    sector = FLASH_Sector_5;
    iap_sector_index = 5;
  }
  else if ((addr >= IAP_SECTOR_ADDR_6) && (addr < IAP_SECTOR_ADDR_7))
  {
    sector = FLASH_Sector_6;
    iap_sector_index = 6;
  }
  else if ((addr >= IAP_SECTOR_ADDR_7) && (addr < IAP_SECTOR_ADDR_8))
  {
    sector = FLASH_Sector_7;
    iap_sector_index = 7;
  }
  else if ((addr >= IAP_SECTOR_ADDR_8) && (addr < IAP_SECTOR_ADDR_9))
  {
    sector = FLASH_Sector_8;
    iap_sector_index = 8;
  }
  else if ((addr >= IAP_SECTOR_ADDR_9) && (addr < IAP_SECTOR_ADDR_10))
  {
    sector = FLASH_Sector_9;
    iap_sector_index = 9;
  }
  else if ((addr >= IAP_SECTOR_ADDR_10) && (addr < IAP_SECTOR_ADDR_11))
  {
    sector = FLASH_Sector_10;
    iap_sector_index = 10;
  }
  else
  {
    sector = FLASH_Sector_11;
    iap_sector_index = 11;
  }

  return sector;*/

	  uint32_t sector = 0;

	  if((addr < ADDR_FLASH_SECTOR_1) && (addr >= ADDR_FLASH_SECTOR_0))
	  {
	    sector = FLASH_SECTOR_0; iap_sector_index = 0;
	  }
	  else if((addr < ADDR_FLASH_SECTOR_2) && (addr >= ADDR_FLASH_SECTOR_1))
	  {
	    sector = FLASH_SECTOR_1;  iap_sector_index = 1;
	  }
	  else if((addr < ADDR_FLASH_SECTOR_3) && (addr >= ADDR_FLASH_SECTOR_2))
	  {
	    sector = FLASH_SECTOR_2;  iap_sector_index = 2;
	  }
	  else if((addr < ADDR_FLASH_SECTOR_4) && (addr >= ADDR_FLASH_SECTOR_3))
	  {
	    sector = FLASH_SECTOR_3;  iap_sector_index = 3;
	  }
	  else if((addr < ADDR_FLASH_SECTOR_5) && (addr >= ADDR_FLASH_SECTOR_4))
	  {
	    sector = FLASH_SECTOR_4;  iap_sector_index = 4;
	  }
	  else if((addr < ADDR_FLASH_SECTOR_6) && (addr >= ADDR_FLASH_SECTOR_5))
	  {
	    sector = FLASH_SECTOR_5;  iap_sector_index = 5;
	  }
	  else if((addr < ADDR_FLASH_SECTOR_7) && (addr >= ADDR_FLASH_SECTOR_6))
	  {
	    sector = FLASH_SECTOR_6;  iap_sector_index = 6;
	  }
	  else if((addr < ADDR_FLASH_SECTOR_8) && (addr >= ADDR_FLASH_SECTOR_7))
	  {
	    sector = FLASH_SECTOR_7;  iap_sector_index = 7;
	  }
	  else if((addr < ADDR_FLASH_SECTOR_9) && (addr >= ADDR_FLASH_SECTOR_8))
	  {
	    sector = FLASH_SECTOR_8;  iap_sector_index = 8;
	  }
	  else if((addr < ADDR_FLASH_SECTOR_10) && (addr >= ADDR_FLASH_SECTOR_9))
	  {
	    sector = FLASH_SECTOR_9;  iap_sector_index = 9;
	  }
	  else if((addr < ADDR_FLASH_SECTOR_11) && (addr >= ADDR_FLASH_SECTOR_10))
	  {
	    sector = FLASH_SECTOR_10;  iap_sector_index = 10;
	  }
	  else /* (addr < FLASH_END_ADDR) && (addr >= ADDR_FLASH_SECTOR_11) */
	  {
	    sector = FLASH_SECTOR_11;  iap_sector_index = 11;
	  }

	  return sector;

}
