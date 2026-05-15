#include "ATFlash.h"
#ifdef STM32F4XX
#include "usart.h"
#else
uint32_t sector[49] = {sector0_address, sector1_address, sector2_address, sector3_address, sector4_address,
                     sector5_address, sector6_address, sector7_address, sector8_address,
                     sector9_address, sector10_address, sector11_address, sector12_address,
                     sector13_address, sector14_address, sector15_address, sector16_address,
                     sector17_address, sector18_address, sector19_address,
                     sector20_address, sector21_address, sector22_address,
                     sector23_address, sector24_address, sector25_address,
                     sector26_address, sector27_address, sector28_address,
                     sector29_address, sector30_address, sector31_address,
                     sector32_address, sector33_address, sector34_address,
                     sector35_address, sector36_address, sector37_address,
                     sector38_address, sector39_address, sector40_address, sector41_address,
                     sector42_address, sector43_address, sector44_address, sector45_address,
                     sector46_address, sector47_address, sector48_address
                     };


#endif
void flash_init(void) {
    flash_unlock();
    flash_flag_clear(FLASH_ODF_FLAG|FLASH_PRGMERR_FLAG|FLASH_EPPERR_FLAG|FLASH_BANK1_ODF_FLAG|
                    FLASH_BANK1_PRGMERR_FLAG|FLASH_BANK1_EPPERR_FLAG|FLASH_BANK2_ODF_FLAG|
                    FLASH_BANK2_PRGMERR_FLAG|FLASH_BANK2_EPPERR_FLAG);
    flash_lock();
}

uint32_t num = 0;
static uint32_t sector_location(const uint32_t address) {
    uint32_t sector_addr=0;
    if ((address & 0x000F0000) == 0) {
        sector_addr = sector[((address & 0x0000F000) >> 12)]; 
    } else if(((address & 0x000F0000) >> 4) == 1) {
        sector_addr = sector[((address & 0x0000F000) >> 12) + 16];
    } else if(((address & 0x000F0000) >> 4) == 2) {
        sector_addr = sector[((address & 0x0000F000) >> 12) + 32];
    } else if(address >= sector48_address) {
        sector_addr = sector48_address;
    }
    return sector_addr;
}

flash_status_type flash_erase(uint32_t address,uint32_t size) {
    flash_status_type res;
    uint32_t start = 0, end = 0, i = 0;
    int sectorSize = 0;
    flash_unlock();
    start = sector_location(address);
    end = sector_location(address + size);
    sectorSize = (end - start)/(sector2_address - sector1_address);
    if (!(end - start))
    {
        res = FLASH_PROGRAM_ERROR;
        return res;
    }
    for (i = 0; i < sectorSize; i++)
    {
        flash_sector_erase(start + (i * 0x1000));
        res = flash_bank1_operation_status_get();
        if (res != FLASH_OPERATE_DONE)
        {
            return res;
        }
    }
    flash_lock();
    return res;
}

flash_status_type flash_program(uint32_t address, uint32_t const * data,const uint32_t len) {
    flash_status_type res;
    uint16_t i = 0;
    uint32_t addressNew = address;
    flash_unlock();
    for (i = 0; i < len; i++) {
        res = flash_word_program(addressNew, data[i]);
        if (res != FLASH_OPERATE_DONE)
        {
            return res;
        }
        addressNew += 4;
    }
    flash_lock();
    return res;
}

iapfun jump2app; 

void iap_load_app(uint32_t appxaddr) {
	if(((*(uint32_t*)appxaddr)&0x2FFE0000)==0x20000000)	//���ջ����ַ�Ƿ�Ϸ�.
	{ 
		jump2app=(iapfun)*(uint32_t*)(appxaddr+4);		//�û��������ڶ�����Ϊ����ʼ��ַ(��λ��ַ)		
		__set_MSP(*(uint32_t*)appxaddr);					//��ʼ��APP��ջָ��(�û��������ĵ�һ�������ڴ��ջ����ַ)
		jump2app();									//��ת��APP.
	}
}	


