/**
 * @file Upload.h
 * @author Zhijie Pan
 * @brief This file contains all the function prototypes for the Upload.c file
 */
#ifndef __UPLOAD_H__
#define __UPLOAD_H__

#include "main.h"
#include "Connect.h"
#include "StepMotor.h"
#include "demo.h"
#include "Joystick.h"
#include "LimitSwitch.h"

extern uint8_t UploadFlag;

void Upload_Puncture_Goal_And_Real(CMD_GOAL* puncture_goal,MotorState* step_motor0,MotorState* step_motor1,MotorState* step_motor2,MotorState* step_motor3);
void Upload_Puncture_Goal(CMD_GOAL* puncture_goal);
void Upload_Motor_State(MotorState* step_motor);
void Upload_LimitSwitch_Trigger(uint8_t id);
void Upload_LimitSwitch_Flag(uint8_t* limitswitch_flag);
void Upload_Rxdata_State(UART_RX* usart_rx);
void Upload_JoyStick_State(JOYSTICK_STATE* joystick_state);
void Upload_Data_Option(uint8_t i);
void Upload_Data(enum CONTROL_MODE Control_Mode);
void Upload_System_Error(void);

#endif
