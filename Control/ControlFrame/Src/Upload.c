/**
 * @file Upload.c
 * @author Zhijie Pan
 * @brief This file provides code to uplaod the data
 */
#include "Upload.h"
#include "Connect.h"
#include "StepMotor.h"
#include "usart.h"
#include "demo.h"
#include "LimitSwitch.h"

#include "math.h"
#include "stdio.h"
#include "stdlib.h"

#define Upload_Buffer_MaxLen   	100  /* 上传数据数组长度 */
#define PI                      3.14159f 

/* 上传数据标志 */
uint8_t UploadFlag=0;
/* 上传储存数组 */
uint8_t UploadBuffer[100];

/**
 * @brief 初始化上传数据用的缓冲数组
 * 
 */
void Upload_Buffer_Init(void)
{
    uint8_t i;
    for(i=0;i<Upload_Buffer_MaxLen;i++)
        UploadBuffer[i]=0;
}

/**
 * @brief 上传穿刺机构的角度目标值与角度实际值
 * 
 * @param puncture_goal 
 * @param step_motor0 
 * @param step_motor1 
 * @param step_motor2 
 * @param step_motor3 
 */
void Upload_Puncture_Goal_And_Real(CMD_GOAL* puncture_goal,MotorState* step_motor0,MotorState* step_motor1,MotorState* step_motor2,MotorState* step_motor3)
{
    float roll_goal_f,pitch_goal_f,roll_real_f,pitch_real_f;    /* 角度的浮点值 */
    int roll_goal_d,pitch_goal_d,roll_real_d,pitch_real_d;      /* 角度百倍整数值 */
    uint8_t roll_goal_s,pitch_goal_s,roll_real_s,pitch_real_s;  /* 角度的正负符号 */

    roll_goal_f = puncture_goal->roll * 180 / PI;
    pitch_goal_f = puncture_goal->pitch * 180 / PI;

    roll_real_f = -atan((float)(step_motor0->realpulse + step_motor2->realpulse)/30000) * 180 / PI;
    pitch_real_f = atan((float)(step_motor1->realpulse + step_motor3->realpulse)/30000) * 180 / PI;

    if(roll_goal_f >= 0) roll_goal_s = 1;
    else roll_goal_s = 0;

    if(pitch_goal_f >= 0) pitch_goal_s = 1;
    else pitch_goal_s = 0;

    if(roll_real_f >= 0) roll_real_s = 1;
    else roll_real_s = 0;

    if(pitch_real_f >= 0) pitch_real_s = 1;
    else pitch_real_s = 0;

    roll_goal_d = (int)(abs(roll_goal_f*100));
    pitch_goal_d = (int)(abs(pitch_goal_f*100));
    roll_real_d = (int)(abs(roll_real_f*100));
    pitch_real_d = (int)(abs(pitch_real_f*100));

    // sprintf((char*)UploadBuffer,"%f %f %f %f \r\n",roll_goal_f,pitch_goal_f,roll_real_f,pitch_real_f);

    sprintf((char*)UploadBuffer,"P01%1d%1d%04d%04d0000ZJ    P01%1d%1d%04d%04d1111ZJ \r\n",
                                    roll_goal_s,pitch_goal_s,roll_goal_d,pitch_goal_d,
                                    roll_real_s,pitch_real_s,roll_real_d,pitch_real_d);
    
    HAL_UART_Transmit_DMA(&huart1,UploadBuffer,45);

}

/**
 * @brief 发送穿刺机构目标
 * 
 * @param puncture_goal 
 */
void Upload_Puncture_Goal(CMD_GOAL* puncture_goal)
{
    Upload_Buffer_Init();
    if(puncture_goal->debug_flag ==0)
        sprintf((char*)UploadBuffer,"PunctureGoal:pitch:%.2f\troll:%.2f\tVelocity:%.2f \r\n"
                                    ,puncture_goal->pitch,puncture_goal->roll,puncture_goal->velocity);
    else
        sprintf((char*)UploadBuffer,"PunctureGoal:id:%d\tpulse:%d\tvel:%d\tdisplacement:%.2f \r\n"
                                    ,puncture_goal->id,puncture_goal->pulse,puncture_goal->vel,puncture_goal->displacement);
    HAL_UART_Transmit_DMA(&huart1,UploadBuffer,sizeof(UploadBuffer));
}

/**
 * @brief 发送步进电机状态
 * 
 * @param step_motor 
 */
void Upload_Motor_State(MotorState* step_motor)
{
    Upload_Buffer_Init();
    sprintf((char*)UploadBuffer,"MotorState: ID:%d\tDir:%d\tRun:%d\tGP:%d\tRP:%d\tGF:%.1f \r\n"
                                ,step_motor->MotorID,step_motor->dir,step_motor->runstate
                                ,step_motor->goalpulse,step_motor->realpulse,step_motor->goalfrequency);
    HAL_UART_Transmit_DMA(&huart1,UploadBuffer,sizeof(UploadBuffer));
}

/**
 * @brief 电机初始化时提示触发限位开关
 * 
 * @param id 电机ID
 */
void Upload_LimitSwitch_Trigger(uint8_t id)
{
    Upload_Buffer_Init();
    sprintf((char*)UploadBuffer,"Motor%d has been initialized.\r\n",id);
    HAL_UART_Transmit_DMA(&huart1,UploadBuffer,sizeof(UploadBuffer));
}

/**
 * @brief 发送限位开关标志
 * 
 * @param limitswitch_flag 
 */
void Upload_LimitSwitch_Flag(uint8_t* limitswitch_flag)
{
    Upload_Buffer_Init();
    sprintf((char*)UploadBuffer,"limitSwitch_Flag:%d\t%d\t%d\t%d \r\n",limitswitch_flag[0]
                                                        ,limitswitch_flag[1]
                                                        ,limitswitch_flag[2]
                                                        ,limitswitch_flag[3]);
    HAL_UART_Transmit_DMA(&huart1,UploadBuffer,sizeof(UploadBuffer));
}

/**
 * @brief 发送缓冲队列状态
 * 
 * @param usart_rx 
 */
void Upload_Rxdata_State(UART_RX* usart_rx)
{
    Upload_Buffer_Init();
    uint8_t len;
    len = Get_RXbuffer_Used_Len(usart_rx);
    sprintf((char*)UploadBuffer,"RxState:In:%d\tOut:%d\tLen:%d \r\n",usart_rx->INflag,usart_rx->Outflag,len);
    HAL_UART_Transmit_DMA(&huart1,UploadBuffer,sizeof(UploadBuffer));
}

/**
 * @brief 发送摇杆状态
 * 
 * @param joystick_state 
 */
void Upload_JoyStick_State(JOYSTICK_STATE* joystick_state)
{
    Upload_Buffer_Init();
    sprintf((char*)UploadBuffer,"JoyStickState:X:%d\tY:%d\tLock:%d \r\n",joystick_state->X_axis,joystick_state->Y_axis,joystick_state->lock);
    HAL_UART_Transmit_DMA(&huart1,UploadBuffer,sizeof(UploadBuffer));
}

/**
 * @brief 发送控制模式
 * 
 * @param Control_Mode 
 */
void Upload_Control_Mode(enum CONTROL_MODE Control_Mode)
{
    Upload_Buffer_Init();
    switch(Control_Mode)
    {
        case None:      sprintf((char*)UploadBuffer,"Control_Mode: None \r\n");
                break;

        case Auto:      sprintf((char*)UploadBuffer,"Control_Mode: Auto \r\n");
                break;

        case Manual:    sprintf((char*)UploadBuffer,"Control_Mode: Manual \r\n");
                break;
        
        case Observe:   sprintf((char*)UploadBuffer,"Control_Mode: Observe \r\n");
                break;

        default:break;
    }
    HAL_UART_Transmit_DMA(&huart1,UploadBuffer,sizeof(UploadBuffer));
}

/**
 * @brief 选择发送的数据
 * 
 * @param i 
 */
void Upload_Data_Option(uint8_t i)
{
    switch(i)
    {
		case 0: Upload_Control_Mode(Control_Mode);break;
        case 1: Upload_Puncture_Goal(&Puncture_Goal);break;
        case 2: Upload_Motor_State(&Step_Motor[0]);break;
        case 3: Upload_Motor_State(&Step_Motor[1]);break;
        case 4: Upload_Motor_State(&Step_Motor[2]);break;
        case 5: Upload_Motor_State(&Step_Motor[3]);break;
        case 6: Upload_Rxdata_State(&Usart1_RX);break;
        case 7: Upload_JoyStick_State(&Joystick_State);break;
        case 8: Upload_LimitSwitch_Flag(LimitSwitch_Flag);break;
        default: break;
    }
}


/**
 * @brief 发送数据
 * 
 * @param Control_Mode 
 */
void Upload_Data(enum CONTROL_MODE Control_Mode)
{
    if(UploadFlag == 1)
    {
        Upload_Puncture_Goal_And_Real(&Puncture_Goal,&Step_Motor[0],&Step_Motor[1],&Step_Motor[2],&Step_Motor[3]);
    }
    UploadFlag = 0;        
}
// void Upload_Data(enum CONTROL_MODE Control_Mode)
// {
//     static uint8_t k=0;
//     static uint8_t buffer1[8]={0,1,2,3,4,5,6,8};
//     static uint8_t buffer2[8]={0,1,2,3,4,5,7,8};
//     static uint8_t buffer3[8]={0,1,2,3,4,5,6,7};

//     if(UploadFlag == 1)
//     {
//         if(Control_Mode == Auto)
//             Upload_Data_Option(buffer1[k]);
//         else if(Control_Mode == Manual)
//             Upload_Data_Option(buffer2[k]);
//         else if(Control_Mode == None || Control_Mode == Observe)
//             Upload_Data_Option(buffer3[k]);
//         k++;
//     }
//     UploadFlag = 0;
//     if(k==8)
//     {
//         k=0;
//     }
        
// }

/**
 * @brief 系统出错提示
 * 
 */
void Upload_System_Error(void)
{
    Upload_Buffer_Init();
    sprintf((char*)UploadBuffer,"System error!\r\n Please check the system configuration and rerun the program! \r\n");
    HAL_UART_Transmit_DMA(&huart1,UploadBuffer,sizeof(UploadBuffer));
}
