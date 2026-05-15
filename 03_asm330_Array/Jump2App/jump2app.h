/*************************************/
/*           CREATED BY WJY           */
/*            2023/10/26              */
/*************************************/
#ifndef __JUMP2APP_H_
#define __JUMP2APP_H_

#include "at32f435_437.h"

#define		flag_address		(uint32_t)0x08030000 //下载成功后写入的标记地址

void jumpToApp(unsigned char buf);


#endif

