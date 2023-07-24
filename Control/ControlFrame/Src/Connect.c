/**
 * @file Connect.c
 * @author Zhijie Pan
 * @brief This file provides code to parsing data sent by the host computer
 */
#include "Connect.h"
#include "usart.h"
#include "StepMotor.h"

uint8_t RxBuffer[Rxbufferlen];
UART_RX Usart1_RX={0};
RX_DATA Receive_Data={0};

/**
 * @brief 开启串口接收
 * 
 */
void Usart_Receive_Start(void)
{
    MX_USART1_UART_Init();
    /* 开启usart1接收中断 */
    HAL_UART_Receive_IT(&huart1,(uint8_t *)RxBuffer,Rxbufferlen);
    /* 初始化缓冲队列与数据帧结构体 */
    Usart_RX_Queue_Init(&Usart1_RX);
    Usart_RX_Data_Init(&Receive_Data);
}

/**
 * @brief 关闭串口接收
 * 
 */
void Usart_Receive_Stop(void)
{
    HAL_UART_MspDeInit(&huart1);
}

/**
 * @brief 初始化数据缓冲队列
 * 
 * @param usart_rx 缓冲队列结构体
 */
void Usart_RX_Queue_Init(UART_RX* usart_rx)
{
    usart_rx->INflag = 1;
    usart_rx->Outflag = 0;
    for(uint16_t i=0;i<MaxLen;i++)
        usart_rx->data[i]=0;
}

/**
 * @brief 初始化数据帧结构体
 * 
 * @param rx_data 数据帧结构体
 */
void Usart_RX_Data_Init(RX_DATA* rx_data)
{
    rx_data->beginsign = 0;
    rx_data->endsign = 0;
    for(uint8_t i=0;i<RealDataLen;i++)
        rx_data->data[i]=0;
}

/**
 * @brief 获取缓冲队列已用空间
 * 
 * @param usart_rx 缓冲队列结构体
 * @return int, 已用空间长度
 */
int Get_RXbuffer_Used_Len(UART_RX* usart_rx)    
{
    uint16_t len=0;

    if(usart_rx->INflag >= usart_rx->Outflag)
        len = usart_rx->INflag - usart_rx->Outflag;
    else
        len = MaxLen + usart_rx->INflag - usart_rx->Outflag;

    return len;
}

/**
 * @brief  获取缓冲队列可用空间
 * 
 * @param usart_rx 缓冲队列结构体
 * @return int, 可用空间长度
 */
int Get_RXbuffer_Available_Len(UART_RX* usart_rx)   
{
    uint16_t len=0;

    len = MaxLen - Get_RXbuffer_Used_Len(usart_rx);

    return len;
}

/**
 * @brief 串口中断回调函数
 * 
 * @param huart 串口结构体
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* USART1触发接收中断 */
    if(huart->Instance==USART1)
	{
        /* 查询缓冲数组中是否还有空间 */
        if(Get_RXbuffer_Available_Len(&Usart1_RX))         
        {
            Usart1_RX.data[Usart1_RX.INflag] = RxBuffer[0];
            if(Usart1_RX.INflag == (MaxLen-1))   
                Usart1_RX.INflag = 0;
            else 
                Usart1_RX.INflag ++;
        }
	}
}

/**
 * @brief 提取缓冲队列数据
 * 
 * @param usart_rx 
 * @param rx_data 
 * @return int ,0：提取失败 1：提取成功
 */
int Get_RXbuffer_data(UART_RX* usart_rx,RX_DATA* rx_data)
{
    uint16_t i;
    uint8_t buffer[RealDataLen+2]={0};

    /* 缓冲区域中不足一帧数据，退出 */
    if(Get_RXbuffer_Used_Len(usart_rx)< (RealDataLen+3))
        return 0;
    else
    {
        /* 寻找到数据帧头 */
        if(usart_rx->data[usart_rx->Outflag] == 0x50)
        {
            rx_data->beginsign = 1;
            usart_rx->Outflag = (usart_rx->Outflag + 1) % MaxLen;   

            /* 将数据暂存在缓冲数组中 */
            for(i=0;i<RealDataLen+2;i++)
            {
                buffer[i] = usart_rx->data[usart_rx->Outflag];
                usart_rx->Outflag = (usart_rx->Outflag + 1) % MaxLen;
            }
        }
        /* 不是帧头，退出 */
        else
        {
            usart_rx->Outflag = (usart_rx->Outflag + 1) % MaxLen;
            return 0;
        }
    }

    /* 帧尾格式正确，提取成功 */
    if(buffer[RealDataLen+1] == 0x4A && buffer[RealDataLen] == 0x5A)
    {
        rx_data->endsign = 1;
        for(i=0;i<RealDataLen;i++)
            rx_data->data[i] = buffer[i];
        return 1;
    }
    /* 帧尾格式错误，重置状态位 */
    else 
    {
        Usart_RX_Data_Init(rx_data);
        return 0;
    }
}

