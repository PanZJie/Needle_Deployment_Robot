/**
 * @file Joystick.c
 * @author Zhijie Pan
 * @brief This file provides code to get the control signal from joystick
 */
#include "Joystick.h"
#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "tim.h"

#define Kp 0.8
uint8_t Joystick_Key_enable_flag=1;

/* 遥杆状态结构体*/
JOYSTICK_STATE Joystick_State;
/* adc采样值存储数组*/
uint32_t value[20];

/**
 * @brief 开启遥杆adc采样
 * 
 */
void Joystick_Start(void)
{
    Joystick_State_Init(&Joystick_State);
    HAL_ADC_Start_DMA(&hadc1,(uint32_t *)value,20);
}

/**
 * @brief 关闭摇杆adc采样
 * 
 */
void Joystick_Stop(void)
{
    HAL_ADC_Stop_DMA(&hadc1);
}

/**
 * @brief 初始化摇杆状态
 * 
 * @param joystick_state 
 */
void Joystick_State_Init(JOYSTICK_STATE* joystick_state)
{
    joystick_state->X_axis = 0;
    joystick_state->Y_axis = 0;
    joystick_state->lock = 0;
    joystick_state->key = 0;
}

/**
 * @brief 一阶低通数字滤波器
 * 
 * @param k 滤波系数
 * @param In 输入
 * @param Last_Out 上次的输出
 * @return float 输出
 */
float LowPass_Filter(float k,float In,float Last_Out)
{
    float out;
    out = k*In + (1-k)*Last_Out;
    return out;
}

/**
 * @brief 摇杆按键消抖延时函数
 * 
 * @param Delay 延时时间，单位ms
 */
void Joystick_Read_Delay(uint32_t Delay)
{
    uint8_t i;
    uint32_t tickstart = HAL_GetTick();
    uint32_t wait = Delay;

    /* 增加一个频率以保证最小的等待 */
    if (wait < HAL_MAX_DELAY)
    {
        wait += (uint32_t)(uwTickFreq);
    }

    while((HAL_GetTick() - tickstart) < wait)
    {
        /* 按键延时过程中不断查询四个电机的状态 */
        for(i=0;i<4;i++)
            StepMotor_RunState_Set(&Step_Motor[i]);
    }
}

/**
 * @brief 读取摇杆的按键状态
 * 
 * @return uint8_t 0：无按下 1：短按 2：长按
 */
uint8_t Joystick_Key_Read(void)
{
	static uint8_t Continue_press=0;	/* 连续按下按键标志 */
	static uint8_t Lossen=1;			/* 松开按键标志 */
	
    /* 判断按键是否按下 */
	if(HAL_GPIO_ReadPin(ROCKER_KEY_GPIO_Port,ROCKER_KEY_Pin) == 1)
	{
        /* 软件延时消除抖动 */
		Joystick_Read_Delay(10);
		/* 第一次按下 */
		if(Lossen==1)
		{
			Lossen=0;
            /* 确认电平值，确认按键是否按下 */
			if(HAL_GPIO_ReadPin(ROCKER_KEY_GPIO_Port,ROCKER_KEY_Pin)== 1)
			{
                /* 长按标志位计数 */
				Continue_press++;
				return 1;
			}
            /* 认为是噪声抖动 */
			else return 0;
		}
		
        /* 按键长按 */
		else
		{
            /* 长按标志计数 */
			if(Continue_press<200)
			{
				Continue_press++;
                /* 计数不满期间返回按键值3 */
				return 3;
			}
            /* 计数满200，时间约为2s */
			else
			{
                /* 返回长按值2 */
				return 2;
			}
		}
	}
    /* 按键电平为0 */
	else
	{
        /* 恢复状态位 */
		Continue_press=0;
		Lossen=1;
        /* 返回不按按键值 0 */
		return 0;
	}
}

/**
 * @brief 获取摇杆数据
 * 
 * @param joystick_state 
 */
void Joystick_State_Get(JOYSTICK_STATE* joystick_state)
{
    uint8_t i;
    uint8_t key=0;
    float X_init=0,Y_init=0;

    /* 取值过程中先关闭adc的DMA搬运 */
    Joystick_Stop();
    /* 获取adc采样值 */
    for(i=0;i<20;i++)
    {
        if(i%2==0)
            X_init = LowPass_Filter(Kp,value[i],X_init);
        else
            Y_init = LowPass_Filter(Kp,value[i],X_init);
    }
    Joystick_Start();

//    key = Joystick_Key_Read();

    joystick_state->key = key;
    joystick_state->X_axis = X_init;
    joystick_state->Y_axis = Y_init;

//    if(Joystick_Key_enable_flag == 1)
//    {
//        if(key == 2 && joystick_state->lock == 0)
//        {
//            joystick_state->lock = 1;
//            Joystick_Key_enable_flag = 0;
//        }
//        else if(key == 2 && joystick_state->lock == 1)
//        {
//            joystick_state->lock = 0;
//            Joystick_Key_enable_flag = 0;
//        }
//    }
}

/**
 * @brief 设置摇杆锁定状态指示灯，低电平亮
 * 
 * @param joystick_state 
 */
void Joystick_LED_Set(JOYSTICK_STATE* joystick_state)
{
    if(joystick_state->lock == 1)
        HAL_GPIO_WritePin(LOCK_LED_GPIO_Port, LOCK_LED_Pin, GPIO_PIN_RESET);
    else if(joystick_state->lock == 0)
        HAL_GPIO_WritePin(LOCK_LED_GPIO_Port, LOCK_LED_Pin, GPIO_PIN_SET);
}



