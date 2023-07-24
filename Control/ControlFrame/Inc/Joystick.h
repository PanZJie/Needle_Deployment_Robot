/**
 * @file Joystick.c
 * @author Zhijie Pan
 * @brief This file contains all the function prototypes for the Joystick.c file
 */
#ifndef __JOYSTICK_H__
#define __JOYSTICK_H__

#include "main.h"

typedef struct 
{
    uint16_t X_axis;    /* X轴模拟值 */
    uint16_t Y_axis;    /* Y轴模拟值 */
    uint8_t key;        /* 按键状态 */
    uint8_t lock;       /* 锁定状态 */
}JOYSTICK_STATE;

extern JOYSTICK_STATE Joystick_State;
extern uint8_t Joystick_Key_enable_flag;

void Joystick_Start(void);
void Joystick_Stop(void);
void Joystick_State_Init(JOYSTICK_STATE* joystick_state);
float LowPass_Filter(float k,float In,float Last_Out);
void Joystick_Read_Delay(uint32_t Delay);
uint8_t Joystick_Key_Read(void);
void Joystick_State_Get(JOYSTICK_STATE* joystick_state);
void Joystick_LED_Set(JOYSTICK_STATE* joystick_state);

#endif
