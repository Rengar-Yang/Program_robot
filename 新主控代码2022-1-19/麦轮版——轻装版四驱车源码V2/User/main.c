#include "stm32f10x.h"
#include "sys.h"
#include "bmp.h"
#include "DMREG.h"
#include "usart.h"
  /**************************************************************************
工训车新主控代码说明文档：
目前已完成基础功能：PS2手柄遥控，全向运动分解，角度控制，位置控制，上位机显示
目前已搭建功能：舵机云台动作组，上下位机通信，UI界面（可能用Openmv驱动）

电机ID位置：
				C --- A
				 |   |
				 |   |
				D --- B

串口占用情况：串口2用于上位机通信，串口3用于控制机械臂，串口5用于Openmv通信
主要函数均位于control.c文件里

最后更新 2022-2-23
**************************************************************************/
u8 Flag_Left,Flag_Right,Flag_Direction=0,Flag_Way,Flag_Next,Turn_Flag;   //蓝牙遥控相关的变量
u8 Flag_Stop=1; //停止标志位和 显示标志位 默认停止 显示打开
u8 t,i;
int Encoder_A,Encoder_B,Encoder_C,Encoder_D;  //编码器的脉冲计数
long int Position_A,Position_B,Position_C,Position_D,Rate_A,Rate_B,Rate_C,Rate_D; //位置控制相关变量  

//需调试修改变量
//*************************************************************************************
float Position_Rate_X=0.001,Position_Rate_Y=0.001;//位置计算系数，需调整
float	Position_KP=5,Position_KI=0,Position_KD=0;  //位置控制PID参数，需调整
float Velocity_KP=10,Velocity_KI=10,Angle_KP=0.9,Angle_KI=0,Angle_KD=20;	          //速度控制PID参数，需调整，前面是速度环，后面是角度环
//**************************************************************************************

s8 Position_MX,Position_MY,Position_X,Position_Y,Position_Z;//当前位置坐标，由算法得出，不用管
long int Target_X,Target_Y;//目标位置坐标
long int Motor_A,Motor_B,Motor_C,Motor_D; //电机PWM变量
long int Target_A,Target_B,Target_C,Target_D; //电机目标值
int Voltage;//电池电压采样相关的变量                       
u8 delay_50,delay_flag; //延时相关变量
u8 Run_Flag=0;  //速度或者位置模式指示变量
u8 rxbuf[8],Urxbuf[8],CAN_ON_Flag=0,Usart_ON_Flag=0,Usart_Flag,PID_Send;  //CAN和串口控制相关变量
u8 txbuf[8],txbuf2[8];  //CAN发送相关变量
float Pitch,Roll,Yaw,Gryo_Z,Move_X,Move_Y,Move_Z;   //三轴角度 Z轴陀螺仪和XYZ轴目标速度
float r,error_p,error_i,error_d,last_angle,angle_speed;//速度PID中间变量
float P_error_pX,P_error_pY,P_error_i,P_error_d,P_speed,last_Position,p_rx,p_ry;//位置PID中间变量
float target_yaw;	//目标角度
int RC_Velocity=30,RC_Position=3000;         //设置遥控的速度和位置值
int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY; //PS2相关变量 
u16 CCD_Zhongzhi,CCD_Yuzhi,ADV[128]={0};//CCD相关变量
int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;//电磁巡线相关
int main(void)
  { 
		delay_init();	    	            //=====延时函数初始化	
	//JTAG_Set(JTAG_SWD_DISABLE);     //=====关闭JTAG接口
		JTAG_Set(SWD_ENABLE);           //=====打开SWD接口 可以利用主板的SWD接口调试
		LED_Init();                     //=====初始化与 LED 连接的硬件接口
	  KEY_Init();                     //=====按键初始化
		MY_NVIC_PriorityGroupConfig(2);	//=====设置中断分组
    MiniBalance_PWM_Init(7199,0);   //=====初始化PWM 10KHZ，用于驱动电机 如需初始化电调接口 
		uart2_init(9600);               //=====串口2初始化
		uart3_init(9600);             //=====串口3初始化 
		uart5_init(9600);             //=====串口5初始化 
		OLED_Init();                    //=====OLED初始化	    
    Encoder_Init_TIM2();            //=====编码器接口
		Encoder_Init_TIM3();            //=====编码器接口
		Encoder_Init_TIM4();            //=====编码器接口
    Encoder_Init_TIM5();            //=====初始化编码器
		OLED_DrawBMP(0,0,128,64,gImage_logo);
		delay_ms(1000);                  //=====延时
    delay_ms(1000);                  //=====延时
		delay_ms(1000);                  //=====延时
		if(Run_Flag==0){ while(select())	{	}	} //=====选择运行模式 
		else Flag_Stop=0;//===位置模式直接使能电机
		Position_D=0;
		delay_ms(500);                  //=====延时
    IIC_Init();                     //=====IIC初始化
    MPU6050_initialize();           //=====MPU6050初始化	
  	DMP_Init();                     //=====初始化DMP   
		CAN1_Mode_Init(1,2,3,6,0);			//=====CAN初始化,波特率1Mbps
		Adc_Init();                     //=====adc初始化		
	 if(Flag_Way==1)
	  {
		PS2_Init();											//=====PS2手柄初始化
		PS2_SetInit();									//=====ps2配置初始化,配置“红绿灯模式”，并选择是否可以修改
	  }
	  else if(Flag_Way==2)ccd_Init();  //=====CCD初始化
	  else if(Flag_Way==3)ele_Init();  //=====电磁传感器初始化	
	  MiniBalance_EXTI_Init();        //=====MPU6050 5ms定时中断初始化
		USART5_TX_BUF[0]=0xa5;
		USART5_TX_BUF[USART5_REC_LEN-1]=0x5a;
    while(1)
	   {	
			   if(Flag_Way==1)
			   {
					  PS2_LX=PS2_AnologData(PSS_LX);    //PS2数据采集    
						PS2_LY=PS2_AnologData(PSS_LY);
						PS2_RX=PS2_AnologData(PSS_RX);
						PS2_RY=PS2_AnologData(PSS_RY);
						PS2_KEY=PS2_DataKey();	
					 if(PS2_KEY==16)
						 UART_DM_ReportData(DM0_Speed10_Position_0,10); //直接改变位置是10，执行动作组是5	
					 if(PS2_KEY==14)
						 UART_DM_ReportData(DM0_Speed10_Position_90,10); 
					 if(PS2_KEY==13)
							UART_DM_ReportData(DM_Action0,5); 				
			   }
				  if(Flag_Way==0)
			   {
				  CAN1_SEND();             //CAN发送		
          //USART_TX();               //串口发送
				 }
				 
				  USART5_TX_BUF[1]=Position_X;//串口5通信，用于与Openmv通信。帧头0xa5，帧尾0x5a。目前暂时用于调试位置环
					USART5_TX_BUF[2]=Position_Y;
					USART5_TX_BUF[USART5_REC_LEN-2]=0;
				 for(i=1;i<=USART5_REC_LEN-3;i++)
					USART5_TX_BUF[USART5_REC_LEN-2]+=USART5_TX_BUF[i];				 
					 for(t=0;t<USART5_REC_LEN;t++)
					{
						USART_SendData(UART5, USART5_TX_BUF[t]);//向串口5发送数据
						while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//等待发送结束
					}
					
					APP_Show();	       //APP相关
					oled_show();          //===显示屏打开
	  } 
}

