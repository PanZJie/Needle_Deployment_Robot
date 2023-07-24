/**
 * @file LimitSwitch.h
 * @author Zhijie Pan
 * @brief This file contains all the function prototypes for the LimitSwitch.c file
 */
#ifndef __LIMITSWITCH_H__
#define __LIMITSWITCH_H__

#include "main.h"

extern uint8_t LimitSwitch_Flag[4];

int Limit_Switch_Check(uint8_t id);
int Motor_Position_Init(void);

#endif
