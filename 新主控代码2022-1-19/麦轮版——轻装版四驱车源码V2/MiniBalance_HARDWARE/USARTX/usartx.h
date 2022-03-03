#ifndef __USRAT3_H
#define __USRAT3_H 
#include "sys.h"	 

#define USART5_REC_LEN 11
#define USART5_REC_LENdebug 6
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
void usart3_send(u8 data);
void uart3_init(u32 bound);
void USART3_IRQHandler(void);
void usart5_send(u8 data);
void uart5_init(u32 bound);
void USART5_IRQHandler(void);
extern u8 USART5_TX_BUF[USART5_REC_LEN];
extern u8 USART5_RX_BUF[USART5_REC_LEN];
extern s8 USART5_TX_BUFdebug[USART5_REC_LENdebug];
extern s8 USART5_RX_BUFdebug[USART5_REC_LENdebug];
#endif

