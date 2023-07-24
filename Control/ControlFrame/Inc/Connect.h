/**
 * @file Connect.h
 * @author Zhijie Pan
 * @brief This file contains all the function prototypes for the Connect.c file
 */
#ifndef __CONNECT_H__
#define __CONNECT_H__

#include "main.h"

#define MaxLen 256              /* 缓存数组长度 */
#define Rxbufferlen 1           /* 单次中断缓存长度 */
#define RealDataLen 16          /* 一帧数据中的有效数据长度 */

/* 缓冲队列结构体 */
typedef struct
{
    uint8_t INflag;             /* 缓冲队列进队序号 */
    uint8_t Outflag;            /* 缓冲队列出队序号 */
    uint8_t data[MaxLen];       /* 数据缓冲队列 */
}UART_RX;

/* 数据帧结构体 */
typedef struct
{
    uint8_t beginsign;          /* 帧头接收标志符 */
    uint8_t endsign;            /* 帧尾接收标志符 */
    uint8_t data[RealDataLen];  /* 有效数据 */
}RX_DATA;

extern uint8_t RxBuffer[Rxbufferlen];
extern UART_RX Usart1_RX;          
extern RX_DATA Receive_Data;       

void Usart_Receive_Start(void);
void Usart_Receive_Stop(void);
void Usart_RX_Queue_Init(UART_RX* usart_rx);
void Usart_RX_Data_Init(RX_DATA* rx_data);
int Get_RXbuffer_Used_Len(UART_RX* usart_rx);
int Get_RXbuffer_Available_Len(UART_RX* usart_rx);
int Get_RXbuffer_data(UART_RX* usart_rx,RX_DATA* rx_data);

#endif
