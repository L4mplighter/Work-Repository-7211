/*************************************/
/*           CREATED BY WJY           */
/*            2023/10/26              */
/*************************************/
#include "jump2app.h"
#include "string.h"

unsigned char buff[5] = {0};

void jumpToApp(const unsigned char buf){
    static unsigned char i = 0;
    unsigned char cnt = 0;
    if(i < 5) buff[i++] = buf;
    else if(i == 5){
        while(1){
            if(buff[cnt++] != '#'){
                i = 0;
                memset(buff,0x00,5);
                return;
            }
            if(cnt == 5){
                flash_unlock();
                flash_sector_erase(flag_address);//将写入的下载标志给清除掉，防止复位后又直接执行到用户程序
                flash_lock();
                SCB->VTOR = FLASH_BASE;//设置启动向量表位置为flash首地址
                __disable_irq();//关闭所有中断
                nvic_system_reset();//软复位跳转
            }
        }   
    }
}

