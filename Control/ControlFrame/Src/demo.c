/**
 * @file demo.c
 * @author Zhijie Pan
 * @brief 
 */

#include "demo.h"
#include "math.h"
#include "gpio.h"
#include "tim.h"
#include "StepMotor.h"
#include "Connect.h"
#include "LimitSwitch.h"
#include "Joystick.h"
#include "Upload.h"

#define PI                      3.14159f   
#define Distance_balls_center   30          /* 两滑块中活动球的中心距离，单位:mm */
#define Default_Velocity        120         /* 默认转速:rpm */
#define Joystick_Delta_Goal     0.00001f    /* 摇杆模式下目标的增量值 */
#define MaxTheta                PI/9       	/* 最大转到角度 */

#define LimitSwitch_Equip       1           /* 系统是否有限位开关 1:有; 0:无*/
#define Syetem_Debug_State      0           /* 系统是否处于调试状态 1:是; 0:否*/

CMD_GOAL Puncture_Goal={0};
enum CONTROL_MODE Control_Mode=None;
enum SYSTEM_STATE Syetem_state=Init;

/*===============================================================================
                                ##### 通用部分 #####
 ===============================================================================*/
/**
 * @brief 初始化穿刺机构目标结构体
 * 
 * @param puncture_goal 
 */
void Puncture_Goal_Init(CMD_GOAL* puncture_goal)
{
    puncture_goal->pitch = 0.0f;
    puncture_goal->roll = 0.0f;
    puncture_goal->vel_flag = 0;
    puncture_goal->velocity = 0.0f;
    puncture_goal->debug_flag = 0;
    puncture_goal->id = 0;
    puncture_goal->pulse = 0;
    puncture_goal->vel = 0;
    puncture_goal->displacement = 0.0f;
}

/**
 * @brief 计算滑块离中心位置的距离，赋予电机目标值
 * 
 * @param puncture_goal 
 */
void Calculate_Slider_displacement(CMD_GOAL* puncture_goal)
{
    float Delta_X,Delta_Y;

    /* 三角计算，位移值 = (球心距离/2)*tan(θ) */
    Delta_X = tan(puncture_goal->pitch)*Distance_balls_center/2;
    Delta_Y = tan(puncture_goal->roll)*Distance_balls_center/2;

    /* 根据眼动算法输出调整角度实际值 */
    Delta_Y = -Delta_Y;

    /* 上位机有速度信号，单位：转/分（rpm） */
    if(puncture_goal->vel_flag)
    {
        StepMotor_goal_Set(&Step_Motor[0],Delta_Y,puncture_goal->velocity);
        StepMotor_goal_Set(&Step_Motor[1],Delta_X,puncture_goal->velocity);
        StepMotor_goal_Set(&Step_Motor[2],Delta_Y,puncture_goal->velocity);
        StepMotor_goal_Set(&Step_Motor[3],Delta_X,puncture_goal->velocity);
    }
    /* 上位机无速度信号 */
    else
    {
        StepMotor_goal_Set(&Step_Motor[0],Delta_Y,Default_Velocity);
        StepMotor_goal_Set(&Step_Motor[1],Delta_X,Default_Velocity);
        StepMotor_goal_Set(&Step_Motor[2],Delta_Y,Default_Velocity);
        StepMotor_goal_Set(&Step_Motor[3],Delta_X,Default_Velocity);
    }
}

/*===============================================================================
                            ##### 上位机控制模式 #####
 ===============================================================================*/
/**
 * @brief 解析上位机发送的命令
 * 
 * @param rx_data 
 * @param puncture_goal 
 */
void RXData_Analyse(RX_DATA* rx_data,CMD_GOAL* puncture_goal)
{
    uint8_t i=0;

    /* 穿刺机构整体目标 */
    float roll=0.0f,pitch=0.0f,velocity=0.0f;
    /* 单电机调试目标 */
    uint8_t id=0;
    int pulse=0,vel=0;
    int displacement=0;

    /* 正常模式，无速度信号 */
    if(rx_data->data[1] == 0x31)
    {
        puncture_goal->vel_flag = 0;
        puncture_goal->debug_flag = 0;

        for(i=4;i<8;i++)
            roll = roll*10 + (rx_data->data[i]-0x30);
        
        for(i=8;i<12;i++)
            pitch = pitch*10 + (rx_data->data[i]-0x30);
        /* 查看符号位，1为正数，0为负数 */
        if(rx_data->data[2] == 0x30)
            roll = -roll;
        if(rx_data->data[3] == 0x30)
            pitch = -pitch;
    }
    /* 正常模式，有速度信号 */
    else if(rx_data->data[1] == 0x32)
    {
        puncture_goal->vel_flag = 1;
        puncture_goal->debug_flag = 0;

        for(i=4;i<8;i++)
            roll = roll*10 + (rx_data->data[i]-0x30);

        for(i=8;i<12;i++)
            pitch = pitch*10 + (rx_data->data[i]-0x30);
        
        for(i=12;i<16;i++)
            velocity = velocity*10 + (rx_data->data[i]-0x30);
        /* 查看符号位，1为正数，0为负数 */
        if(rx_data->data[2] == 0x30)
            roll = -roll;
        if(rx_data->data[3] == 0x30)
            pitch = -pitch;
    }
    /* 单电机调试模式，有速度信号，直接给定位移量 */
    else if(rx_data->data[1] == 0x33)
    {
        puncture_goal->debug_flag = 1;
        id = rx_data->data[3] - 0x30;

        displacement = (rx_data->data[4]<<24)|(rx_data->data[5]<<16)|(rx_data->data[6]<<8)|(rx_data->data[7]);
        vel = (rx_data->data[8]<<24)|(rx_data->data[9]<<16)|(rx_data->data[10]<<8)|(rx_data->data[11]);
    }
    /* 单电机调试模式，有速度信号，直接给定脉冲数 */
    else if(rx_data->data[1] == 0x34)
    {
        puncture_goal->debug_flag = 2;
        id = rx_data->data[3] - 0x30;

        pulse = (rx_data->data[4]<<24)|(rx_data->data[5]<<16)|(rx_data->data[6]<<8)|(rx_data->data[7]);
        vel = (rx_data->data[8]<<24)|(rx_data->data[9]<<16)|(rx_data->data[10]<<8)|(rx_data->data[11]);
    }

    /* 解析数据赋给目标结构体 */
    puncture_goal->roll = ((roll/100)/360)*2*PI;
    puncture_goal->pitch =((pitch/100)/360)*2*PI;
    puncture_goal->velocity = velocity/10;

    puncture_goal->id = id;
    puncture_goal->displacement = displacement;
    puncture_goal->pulse = pulse;
    puncture_goal->vel = vel;

    /* 重置状态位 */
    Usart_RX_Data_Init(rx_data);
}

/**
 * @brief 上位机控制函数
 * 
 */
void Puncture_Auto_Control(void)
{
    uint8_t i=0;
    /* 获取解析上位机数据 */
    if(Get_RXbuffer_data(&Usart1_RX,&Receive_Data))
    {
        RXData_Analyse(&Receive_Data,&Puncture_Goal);
        /* 进行目标值赋值 */
        if(Puncture_Goal.debug_flag == 0)
        {
            Calculate_Slider_displacement(&Puncture_Goal);
        }
        else if(Puncture_Goal.debug_flag == 1)
        {
            StepMotor_goal_Set(&Step_Motor[Puncture_Goal.id],(float)Puncture_Goal.displacement,(float)Puncture_Goal.vel);
        }
        else if(Puncture_Goal.debug_flag == 2)
        {
            Step_Motor[Puncture_Goal.id].goalpulse = Puncture_Goal.pulse;
            Step_Motor[Puncture_Goal.id].goalspeed = (float)Puncture_Goal.vel;
            StepMotor_Speed_Set(&Step_Motor[Puncture_Goal.id]);
        }
    }
    /* 不断查询四个电机的状态 */
    for(i=0;i<4;i++)
        StepMotor_RunState_Set(&Step_Motor[i]);
}

/*===============================================================================
                            ##### 摇杆控制模式 #####
 ===============================================================================*/
 /**
  * @brief 解析摇杆模拟值
  * 
  * @param joysticks_tate 
  * @param puncture_goal 
  */
 void Joystick_State_Analyse(JOYSTICK_STATE* joystick_state,CMD_GOAL* puncture_goal)
 {
    if(joystick_state->lock == 0)
    {
        if(joystick_state->X_axis >= 3200)
            puncture_goal->roll = puncture_goal->roll - Joystick_Delta_Goal;
        else if(joystick_state->X_axis <= 1000)
            puncture_goal->roll = puncture_goal->roll + Joystick_Delta_Goal;

        if(joystick_state->Y_axis >= 3500)
            puncture_goal->pitch = puncture_goal->pitch - Joystick_Delta_Goal;
        else if(joystick_state->Y_axis <= 1000)
            puncture_goal->pitch = puncture_goal->pitch + Joystick_Delta_Goal;

        /* 角度限幅 */
        if(puncture_goal->pitch > MaxTheta)
            puncture_goal->pitch = MaxTheta;
        else if(puncture_goal->pitch < -MaxTheta)
            puncture_goal->pitch = -MaxTheta;

        if(puncture_goal->roll > MaxTheta)
            puncture_goal->roll = MaxTheta;
        else if(puncture_goal->roll < -MaxTheta)
            puncture_goal->roll = -MaxTheta;

        /* 无速度信号 */
         puncture_goal->vel_flag = 0;
    }
 }

/**
 * @brief 摇杆控制函数
 * 
 */
void Puncture_Manual_Control(void)
{
    uint8_t i;
    /* 获取摇杆状态 */
    Joystick_State_Get(&Joystick_State);
    Joystick_State_Analyse(&Joystick_State,&Puncture_Goal);
    Joystick_LED_Set(&Joystick_State);
    Calculate_Slider_displacement(&Puncture_Goal);
    /* 不断查询四个电机的状态 */
    for(i=0;i<4;i++)
        StepMotor_RunState_Set(&Step_Motor[i]);
}

/*===============================================================================
                            ##### 系统控制部分 #####
 ===============================================================================*/
/**
 * @brief 系统初始化
 * 
 */
void System_Init(void)
{
    /* 初始化电机状态 */
    StepMotor_Init(&Step_Motor[0],MotorA,5);
    StepMotor_Init(&Step_Motor[1],MotorB,5);
    StepMotor_Init(&Step_Motor[2],MotorC,5);
    StepMotor_Init(&Step_Motor[3],MotorD,5);
    /* 初始化穿刺机构目标 */
    Puncture_Goal_Init(&Puncture_Goal);
    /* 初始化系统运行指示用定时器 */
    HAL_TIM_Base_Start_IT(&htim3);

#if LimitSwitch_Equip
    /* 等待启动 */
	while(HAL_GPIO_ReadPin(BOARD_KEY_GPIO_Port,BOARD_KEY_Pin))
	{
	}
    /* 初始化电机位置 */
    if(Motor_Position_Init())
    {
        /* 限位开关正常 */
        if(Syetem_Debug_State)
        /* 初始化完毕，系统转换为调试状态 */
            Syetem_state = Debug;
        else
        /* 初始化完毕，系统转换为正常状态 */
            Syetem_state = Normal;
    }
    else
        /* 限位开关异常 */
            Syetem_state = Error;
#else
    if(Syetem_Debug_State)
        /* 初始化完毕，系统转换为调试状态 */
        Syetem_state = Debug;
    else
        /* 初始化完毕，系统转换为正常状态 */
        Syetem_state = Normal;
#endif

}

/**
 * @brief 系统运行指示
 * 
 */
void System_Running_Indication(void)
{
    static uint16_t cnt=0;
    cnt++;
    /* 100ms一次，上传一次数据 */
    if(cnt%10 == 0)
        UploadFlag = 1;
    /* 500ms一次，系统板led灯闪烁 */
    if(cnt%50 == 0)
    {
        HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port,BOARD_LED_Pin);
    }
	/* 5s一次，恢复摇杆的按键 */
	if(cnt%1000 == 0)
	{
        Joystick_Key_enable_flag =1;
		cnt = 0;
	}
}

/**
 * @brief 系统状态转换函数
 * 
 * @param syetem_state      当前系统状态
 * @param last_syetem_state 上一时刻系统状态
 * @return enum SYSTEM_STATE    
 */
enum SYSTEM_STATE System_State_Change(enum SYSTEM_STATE syetem_state,enum SYSTEM_STATE last_syetem_state)
{
    /* Init状态转换为Normal状态*/
    if(syetem_state == Normal && last_syetem_state == Init)
    {
        
    }
    /* Init状态转换为Debug状态*/
    else if(syetem_state == Debug && last_syetem_state == Init)
    {
        /* 控制模式转换为观察模式*/
        Control_Mode = Observe;
    }
    /* Init状态转换为Error状态*/
    else if(syetem_state == Error)
    {
        /* 系统故障，电机脱机断电 */
        HAL_GPIO_WritePin(ENA_1_GPIO_Port,ENA_1_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(ENA_2_GPIO_Port,ENA_2_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(ENA_3_GPIO_Port,ENA_3_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(ENA_4_GPIO_Port,ENA_4_Pin,GPIO_PIN_SET);
    }
    return syetem_state;
}

/**
 * @brief 系统运行函数
 * 
 */
void System_Running(void)
{
    static enum SYSTEM_STATE Last_Syetem_state=Init;
    /* 系统转换了状态 */
    if(Last_Syetem_state != Syetem_state)
        Last_Syetem_state = System_State_Change(Syetem_state,Last_Syetem_state);

    switch(Syetem_state)
    {
        case Init:      System_Init();
                break;

        case Normal:    Control_Mode_Scan();
                        Puncture_Mechanism_Control();
                        Upload_Data(Control_Mode);
                break;

        case Debug:     Puncture_Mechanism_Control();
                        Upload_Data(Control_Mode);
                break;

        case Error:     Upload_System_Error();
                        HAL_Delay(500);
                break;
                
        default:break;
    }
}

/**
 * @brief 查询控制模式，控制引脚低电平有效
 * 
 */
void Control_Mode_Scan(void)
{
    uint8_t i,j;
    i = HAL_GPIO_ReadPin(SWITCH_UP_GPIO_Port,SWITCH_UP_Pin);
    j = HAL_GPIO_ReadPin(SWITCH_DOWN_GPIO_Port,SWITCH_DOWN_Pin);
    if(i==1 && j==0)
        Control_Mode = Manual;
    else if(i==0 && j==1)
        Control_Mode = Auto;
    else
        Control_Mode = None;
}

/**
 * @brief 控制模式转换函数
 * 
 * @param control_mode 
 * @return enum CONTROL_MODE 
 */
enum CONTROL_MODE Control_Mode_Change(enum CONTROL_MODE control_mode)
{
    /* 打开串口接收 */
    Usart_Receive_Start();
    /* 打开adc采样 */
    Joystick_Start();
    
    return Control_Mode;
}

/**
 * @brief 穿刺机构控制函数
 * 
 */
void Puncture_Mechanism_Control(void)
{
    static enum CONTROL_MODE Last_Control_Mode=None;
    /* 转换了控制模式 */
    if(Last_Control_Mode != Control_Mode)
        Last_Control_Mode = Control_Mode_Change(Control_Mode);
    
    switch(Control_Mode)
    {
        case None:  	/* 不断查询四个电机的状态 */
						for(uint8_t i=0;i<4;i++)
							StepMotor_RunState_Set(&Step_Motor[i]);
                break;

        case Auto:      Puncture_Auto_Control();
                break;

        case Manual:    Puncture_Manual_Control();
                break;
        
        case Observe:   /* 上位机控制 */
                        Puncture_Auto_Control();
                        /* 获取摇杆状态 */
                        Joystick_State_Get(&Joystick_State);
                break;

        default:break;
    }
}
