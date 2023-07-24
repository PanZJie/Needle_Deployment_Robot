/**
 * @file LimitSwitch.c
 * @author Zhijie Pan
 * @brief This file provides code to get the control signal from Limit switch
 */
#include "LimitSwitch.h"
#include "gpio.h"
#include "StepMotor.h"
#include "usart.h"
#include "Upload.h"

#define Limit1_offset	0
#define Limit2_offset	500
#define Limit3_offset	0
#define Limit4_offset	700
/**
 * @brief   限位开关触发标志  
            0:未触发限位开关
            1：触发限位开关
            2：触发限位开关但状态不正常
 */
uint8_t LimitSwitch_Flag[4]={0};

/**
 * @brief 外部中断回调函数
 * 
 * @param GPIO_Pin 
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == LIM_IN_1_Pin)
    {
        /* 停止电机运动 */
        StepMotor_STOP(&Step_Motor[0]);

        /* 常闭端正确断开 */
//        if(HAL_GPIO_ReadPin(LIM_CHECK_1_GPIO_Port,LIM_CHECK_1_Pin) == 0)
//        {
            Step_Motor[0].realpulse = MaxPulse + Limit1_offset;
            LimitSwitch_Flag[0]=1;
//        }
        /* 常闭端未断开，限位开关出错 */
//        else
//            LimitSwitch_Flag[0]=2;
    }

    else if(GPIO_Pin == LIM_IN_2_Pin)
    {
        StepMotor_STOP(&Step_Motor[1]);

//        if(HAL_GPIO_ReadPin(LIM_CHECK_2_GPIO_Port,LIM_CHECK_2_Pin) == 0)
//        {
            Step_Motor[1].realpulse = MaxPulse + Limit2_offset;
            LimitSwitch_Flag[1]=1;
//        }
//        else    
//            LimitSwitch_Flag[1]=2;
    }

    else if(GPIO_Pin == LIM_IN_3_Pin)
    {
        StepMotor_STOP(&Step_Motor[2]);

//        if(HAL_GPIO_ReadPin(LIM_CHECK_3_GPIO_Port,LIM_CHECK_3_Pin) == 0)
//        {
            Step_Motor[2].realpulse = MaxPulse + Limit3_offset;
            LimitSwitch_Flag[2]=1;
//        }
//        else    
//            LimitSwitch_Flag[2]=2;
    }

    else if(GPIO_Pin == LIM_IN_4_Pin)
    {
        StepMotor_STOP(&Step_Motor[3]);

//        if(HAL_GPIO_ReadPin(LIM_CHECK_4_GPIO_Port,LIM_CHECK_4_Pin) == 0)
//        {
            Step_Motor[3].realpulse = MaxPulse + Limit4_offset;
            LimitSwitch_Flag[3]=1;
//        }
//        else    
//            LimitSwitch_Flag[3]=2;
    }
}

/**
 * @brief 检测限位开关是否处于触发状态
 * 
 * @param id 
 * @return int 
 */
int Limit_Switch_Check(uint8_t id)
{
    switch(id)
    {
        case 0: /* 电机的限位开关处于触发状态 */
                if(HAL_GPIO_ReadPin(LIM_CHECK_1_GPIO_Port,LIM_CHECK_1_Pin) == 0)
                {
                    Step_Motor[0].realpulse = MaxPulse + Limit1_offset;
                    LimitSwitch_Flag[0]=1;
                    return 1;
                }
                /* 电机的限位开关不处于触发状态 */
                else return 0;
        
        case 1: if(HAL_GPIO_ReadPin(LIM_CHECK_2_GPIO_Port,LIM_CHECK_2_Pin) == 0)
                {
                    Step_Motor[1].realpulse = MaxPulse + Limit2_offset;
                    LimitSwitch_Flag[1]=1;
                    return 1;
                }
                else return 0;
        
        case 2: if(HAL_GPIO_ReadPin(LIM_CHECK_3_GPIO_Port,LIM_CHECK_3_Pin) == 0)
                {
                    Step_Motor[2].realpulse = MaxPulse + Limit3_offset;
                    LimitSwitch_Flag[2]=1;
                    return 1;
                }
                else return 0;

        case 3: if(HAL_GPIO_ReadPin(LIM_CHECK_4_GPIO_Port,LIM_CHECK_4_Pin) == 0)
                {
                    Step_Motor[3].realpulse = MaxPulse + Limit4_offset;
                    LimitSwitch_Flag[3]=1;
                    return 1;
                }
                else return 0;

        default: return 0;
    }
}

/**
 * @brief 电机位置初始化函数
 * 
 * @return int 
 */
int Motor_Position_Init(void)
{
    uint8_t i,k,l;
    /* 电机位置初始化的顺序 */
    uint8_t array[4]={0,2,1,3};
    /* 初始化限位开关标志 */
    for(i=0;i<4;i++)
    {
        LimitSwitch_Flag[i]=0;
    }

    /* 初始化各个电机的位置 */
    for(i=0;i<4;i++)
    {
		k=array[i];
		if(i>0)
			l=array[i-1];
        
        /* 启动运行电机 */
        Step_Motor[k].dir = 1;
        StepMotor_DIR_SET(&Step_Motor[k]);   
        StepMotor_RUN(&Step_Motor[k]);

        /* 若限位开关最开始不处于触发状态 */
        if(Limit_Switch_Check(k) == 0)
        {
            /* 运行电机直到触发限位开关 */
            while(LimitSwitch_Flag[k] == 0)
            {
                /* 已初始化完毕的电机往中心位置运行 */
                if(i>0)
                {
                    StepMotor_RunState_Set(&Step_Motor[l]);
                }
                Upload_Data(Control_Mode);
            }
        }
        /* 若限位开关一开始处于触发状态 */
        else 
        {
            /* 停止电机运动 */
            StepMotor_STOP(&Step_Motor[k]);
        }
            
		if(i>0)
			StepMotor_STOP(&Step_Motor[l]);
        /* 限位开关正常触发 */
        // if(LimitSwitch_Flag[k] == 1)
        // {
            /* 上传信息表示第k个电机位置初始化完毕 */
            Upload_LimitSwitch_Trigger(k);
            /* 设置初始目标位置和初始目标速度 */
            StepMotor_goal_Set(&Step_Motor[k],0,180);
        // }    
        /* 限位开关异常 */
        // else if(LimitSwitch_Flag[k] == 2)
        //     return 0;
    }
    /* 初始化成功，返回值1 */
    return 1;
}
