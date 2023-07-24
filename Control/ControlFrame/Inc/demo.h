/**
 * @file demo.h
 * @author Zhijie Pan
 * @brief 
 */
#ifndef __DEMO_H__
#define __DEMO_H__

#include "main.h"
#include "Connect.h"
#include "Joystick.h"

/* 上位机发送的目标结构体 */
typedef struct
{
    /* 穿刺机构整体目标 */
    float pitch;        /* 单位为rad */
    float roll;         /* 单位为rad */
    uint8_t vel_flag;
    float velocity;     /* 单位为rpm */
    
    /* 单电机调试目标 */
    uint8_t debug_flag;
    uint8_t id;
    int pulse;
    uint16_t vel;       /* 单位为rpm */
    float displacement; /* 单位为mm */
}CMD_GOAL;

/* 控制模式 */
enum CONTROL_MODE
{
    None = 0,       /* 无控制状态 */
    Auto = 1,       /* 上位机控制模式 */
    Manual = 2,     /* 摇杆控制模式 */
    Observe = 3,    /* 观察模式 */    
};

/* 系统运行状态 */
enum SYSTEM_STATE
{
    Init = 0,       /* 初始化状态 */
    Normal = 1,     /* 正常运行状态 */
    Debug = 2,      /* 模拟调试状态 */
    Error = 3,      /* 出错状态 */    
};

extern CMD_GOAL Puncture_Goal;
extern enum CONTROL_MODE Control_Mode;
extern enum SYSTEM_STATE Syetem_state;

void Puncture_Goal_Init(CMD_GOAL* puncture_goal);
void Calculate_Slider_displacement(CMD_GOAL* puncture_goal);

void RXData_Analyse(RX_DATA* rx_data,CMD_GOAL* puncture_goal);
void Puncture_Auto_Control(void);
void Joystick_State_Analyse(JOYSTICK_STATE* joystick_state,CMD_GOAL* puncture_goal);
void Puncture_Manual_Control(void);

void System_Init(void);
void System_Running_Indication(void);

enum SYSTEM_STATE System_State_Change(enum SYSTEM_STATE syetem_state,enum SYSTEM_STATE last_syetem_state);
void System_Running(void);

void Control_Mode_Scan(void);
enum CONTROL_MODE Control_Mode_Change(enum CONTROL_MODE control_mode);
void Puncture_Mechanism_Control(void);

#endif
