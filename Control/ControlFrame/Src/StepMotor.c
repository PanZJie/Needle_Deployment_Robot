/**
 * @file StepMotor.c
 * @author Zhijie Pan
 * @brief This file provides code to contorl the StepMotor
 */
#include "StepMotor.h"
#include "tim.h"
#include "gpio.h"
#include "demo.h"

MotorState Step_Motor[4];

/**
 * @brief 初始化电机状态
 * 
 * @param stepmotor 电机状态结构体
 * @param id        电机ID
 * @param division  电机细分数,1细分则200脉冲转一圈，2细分则400脉冲转一圈
 */
void StepMotor_Init(MotorState* step_motor,uint8_t id,uint8_t division)
{
    step_motor->MotorID = id;
    step_motor->runstate = 0;    
    step_motor->dir  = 1;
    step_motor->division = division;
    step_motor->goalpulse = 0;
    step_motor->realpulse = 0;
    step_motor->goalspeed = 180.0f;
    step_motor->goalfrequency = 1000.0f;
    step_motor->autoload = 999;
}

/**
 * @brief 设置电机的目标脉冲数与目标速度
 * 
 * @param step_motor 
 * @param displacement 目标位置（目标脉冲数），单位为mm，中心位置为0值
 * @param speed 目标速度，单位为转/分钟（rpm）
 * @return int 
 */
void StepMotor_goal_Set(MotorState* step_motor,float displacement,float speed)
{
    /* 目标脉冲数=（目标位移/丝杆导程）*（驱动器细分数）*200 */
    step_motor->goalpulse = (displacement/Screw_lead)*(step_motor->division)*200;
	/* 目标脉冲数限幅 */
	if(step_motor->goalpulse > MaxPulse)
		step_motor->goalpulse = MaxPulse;
	if(step_motor->goalpulse < MinPulse)
		step_motor->goalpulse = MinPulse;
	
    step_motor->goalspeed = speed;
    /* 设置电机脉冲频率 */
    StepMotor_Speed_Set(step_motor);
}

/**
 * @brief 设置电机脉冲频率
 * 
 * @param step_motor 
 * @return int 
 */
int StepMotor_Speed_Set(MotorState* step_motor)
{
    /* 目标速度(转/秒) */
    float speed_ns;
    speed_ns =  step_motor->goalspeed / 60;
    /* 速度限幅 */
    if(speed_ns>MaxSpeed)
        speed_ns = MaxSpeed;
    /* 目标频率 =（目标秒转速）*（驱动器细分数）*200 */
    if(speed_ns <= 1/((step_motor->division)*200))
        /* 设置最小目标频率 */
        step_motor->goalfrequency = 1;
    else
        step_motor->goalfrequency = speed_ns*(step_motor->division)*200;
    /* 定时器重装载值 = 1000000(分频后的计数频率)/目标频率 - 1 */
    step_motor->autoload = 1000000/(step_motor->goalfrequency)-1;

    return 1;
}

/**
 * @brief 设置控制电机转动方向的引脚，将电机的转动方向值转变为IO口的电平值
 * 
 * @param step_motor 
 */
void StepMotor_DIR_SET(MotorState* step_motor)
{
    switch(step_motor->MotorID)
    {
        case MotorA :   if(step_motor->dir == 1) HAL_GPIO_WritePin(GPIOE,DIR_1_Pin,GPIO_PIN_RESET);
                        else HAL_GPIO_WritePin(GPIOE,DIR_1_Pin,GPIO_PIN_SET);
                break;

        case MotorB :   if(step_motor->dir == 1) HAL_GPIO_WritePin(GPIOE,DIR_2_Pin,GPIO_PIN_RESET);
                        else HAL_GPIO_WritePin(GPIOE,DIR_2_Pin,GPIO_PIN_SET);
                break;

        case MotorC :   if(step_motor->dir == 1) HAL_GPIO_WritePin(GPIOE,DIR_3_Pin,GPIO_PIN_RESET);
                        else HAL_GPIO_WritePin(GPIOE,DIR_3_Pin,GPIO_PIN_SET);
                break;

        case MotorD :   if(step_motor->dir == 1) HAL_GPIO_WritePin(GPIOB,DIR_4_Pin,GPIO_PIN_RESET);
                        else HAL_GPIO_WritePin(GPIOB,DIR_4_Pin,GPIO_PIN_SET);
                break;
        default:
                break;
    }
}

/**
 * @brief 开启PWM信号与定时器中断
 * 
 * @param id 电机ID,MotorA,MotorB,MotorC,MotorD
 * @param autoload 自动重装载值
 */
void Start_PWM_and_IT(uint8_t id,uint16_t autoload)
{
  switch (id)
  {
  case MotorA:  __HAL_TIM_SET_COUNTER(&htim10,1);           /* 清除定时器计数值 */
                __HAL_TIM_SET_AUTORELOAD(&htim10,autoload);
                __HAL_TIM_SET_COMPARE(&htim10,TIM_CHANNEL_1,autoload/2); 
                HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
                HAL_TIM_Base_Start_IT(&htim10);
                break;

  case MotorB:  __HAL_TIM_SET_COUNTER(&htim11,1);
                __HAL_TIM_SET_AUTORELOAD(&htim11,autoload);
                __HAL_TIM_SET_COMPARE(&htim11,TIM_CHANNEL_1,autoload/2);
                HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);
                HAL_TIM_Base_Start_IT(&htim11);
                break;

  case MotorC:  __HAL_TIM_SET_COUNTER(&htim13,1);
                __HAL_TIM_SET_AUTORELOAD(&htim13,autoload);
                __HAL_TIM_SET_COMPARE(&htim13,TIM_CHANNEL_1,autoload/2);
                HAL_TIM_PWM_Start(&htim13,TIM_CHANNEL_1);
                HAL_TIM_Base_Start_IT(&htim13);
                break;

  case MotorD:  __HAL_TIM_SET_COUNTER(&htim14,1);
                __HAL_TIM_SET_AUTORELOAD(&htim14,autoload);
                __HAL_TIM_SET_COMPARE(&htim14,TIM_CHANNEL_1,autoload/2);
                HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
                HAL_TIM_Base_Start_IT(&htim14);
                break;
  default:
    break;
  }
}

/**
 * @brief 关闭PWM信号与定时器中断
 * 
 * @param id 
 */
void Stop_PWM_and_IT(uint8_t id)
{
  switch (id)
  {
  case MotorA:  HAL_TIM_PWM_Stop(&htim10,TIM_CHANNEL_1);
                HAL_TIM_Base_Stop_IT(&htim10);
                break;

  case MotorB:  HAL_TIM_PWM_Stop(&htim11,TIM_CHANNEL_1);
                HAL_TIM_Base_Stop_IT(&htim11);
                break;

  case MotorC:  HAL_TIM_PWM_Stop(&htim13,TIM_CHANNEL_1);
                HAL_TIM_Base_Stop_IT(&htim13);
                break;

  case MotorD:  HAL_TIM_PWM_Stop(&htim14,TIM_CHANNEL_1);
                HAL_TIM_Base_Stop_IT(&htim14);
                break;
  default:
    break;
  }
}

/**
 * @brief 电机运行
 * 
 * @param step_motor 
 */
void StepMotor_RUN(MotorState* step_motor)
{
    Start_PWM_and_IT(step_motor->MotorID,step_motor->autoload);
}

/**
 * @brief 电机停止
 * 
 * @param step_motor 
 */
void StepMotor_STOP(MotorState* step_motor)
{
    Stop_PWM_and_IT(step_motor->MotorID);
    step_motor->runstate = 0;
}

/**
 * @brief 设置电机运行状态，运行电机
 * 
 * @param step_motor 
 * @return int 0：电机静止；1：电机正转；2：电机反转；
 */
int StepMotor_RunState_Set(MotorState* step_motor)
{
    /* 电机处于静止状态 */
    if(step_motor->runstate == 0)
    {
        /* 目标位置大于实际位置 */
        if(step_motor->goalpulse > step_motor->realpulse)
        {
            /* 电机变为转动状态 */
            step_motor->runstate = 1;
            /* 电机正转 */
            step_motor->dir = 1;  
            StepMotor_DIR_SET(step_motor);   
            StepMotor_RUN(step_motor);
            return 1;
        }

        /* 目标位置小于实际位置 */
        else if(step_motor->goalpulse < step_motor->realpulse)
        {
            step_motor->runstate = 1;
            /* 电机反转 */
            step_motor->dir = 0;
            StepMotor_DIR_SET(step_motor);
            StepMotor_RUN(step_motor);;
            return 2;
        }
        /* 电机实际位置等于目标位置 */
        else return 0;
    }

    /* 电机处于转动状态 */
    else
    {
        if(step_motor->goalpulse > step_motor->realpulse)
        {
            /* 转动方向正确，退出 */
            if(step_motor->dir == 1) return 1;
            /* 转动方向错误，改变电机转动方向 */
            else 
            {
                step_motor->dir = 1;
                StepMotor_DIR_SET(step_motor);
                return 1;
            }
        }
        else if (step_motor->goalpulse < step_motor->realpulse)
        {
            if(step_motor->dir == 0) return 2;
            else 
            {
                step_motor->dir = 0;
                StepMotor_DIR_SET(step_motor);
                return 2;
            }
        }

        /* 实际值等于目标值 */
        else
        {
            /* 设置电机为静止状态 */
            step_motor->runstate = 0;
            StepMotor_STOP(step_motor);
            return 0;
        }
    }
}

/**
 * @brief 定时器中断回调函数
 * 
 * @param htim 定时器结构体
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* 计算电机实际转动脉冲数 */
    if(htim == (&htim10))
    {
        if(Step_Motor[MotorA].dir == 1)
			Step_Motor[MotorA].realpulse ++;
        else Step_Motor[MotorA].realpulse --;
    }
    else if(htim == (&htim11))
    {
        if(Step_Motor[MotorB].dir == 1)
            Step_Motor[MotorB].realpulse ++;
        else Step_Motor[MotorB].realpulse --;
    }
    else if(htim == (&htim13))
    {
        if(Step_Motor[MotorC].dir == 1)
            Step_Motor[MotorC].realpulse ++;
        else Step_Motor[MotorC].realpulse --;
    }
    else if(htim == (&htim14))
    {
        if(Step_Motor[MotorD].dir == 1)
            Step_Motor[MotorD].realpulse ++;
        else Step_Motor[MotorD].realpulse --;
    }

    /* 系统运行指示定时器 */
    else if(htim == (&htim3))
    {
        System_Running_Indication();
    }
}
