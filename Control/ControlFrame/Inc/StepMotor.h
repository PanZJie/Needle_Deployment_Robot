/**
 * @file StepMotor.h
 * @author Zhijie Pan (you@domain.com)
 * @brief This file contains all the function prototypes for the StepMotor.c file
 */
#ifndef __STEPMOTOR_H__
#define __STEPMOTOR_H__

#include "main.h"

#define MotorA 0	/* 电机ID */
#define MotorB 1
#define MotorC 2
#define MotorD 3

#define Screw_lead  1       /* 丝杆导程，单位：mm */
#define MaxSpeed    5      	/* 最大转速（转/秒）*/
#define MaxPulse    6225   	/* 最大位置 */
#define MinPulse    -6225  	/* 最小位置 */

/* 电机状态结构体 */
typedef struct motorstate
{
	uint8_t MotorID;		/* 电机ID */
	uint8_t runstate;		/* 电机运行状态 0：静止状态 1：转动状态 */
	uint8_t dir;			/* 电机转动方向 */
	uint8_t division;		/* 电机细分数 */	
	int goalpulse;			/* 电机目标脉冲数 */
	int realpulse;			/* 电机实际转动脉冲数 */
	float goalspeed;		/* 电机目标转速(rpm)*/
	float goalfrequency;	/* 目标转速对应目标频率 */
	uint16_t autoload;		/* 目标频率对应自动重装载值 */
} MotorState;

extern MotorState Step_Motor[4];

void StepMotor_Init(MotorState* step_motor,uint8_t id,uint8_t division);
void StepMotor_goal_Set(MotorState* step_motor,float displacement,float speed);
int StepMotor_Speed_Set(MotorState* step_motor);
void StepMotor_DIR_SET(MotorState* step_motor);
void Start_PWM_and_IT(uint8_t id,uint16_t autoload);
void Stop_PWM_and_IT(uint8_t id);
void StepMotor_RUN(MotorState* step_motor);
void StepMotor_STOP(MotorState* step_motor);
int StepMotor_RunState_Set(MotorState* step_motor);

#endif
