#include "stm32f10x.h"
#include "sys.h"
#include "bmp.h"
#include "DMREG.h"
#include "usart.h"
  /**************************************************************************
��ѵ�������ش���˵���ĵ���
Ŀǰ����ɻ������ܣ�PS2�ֱ�ң�أ�ȫ���˶��ֽ⣬�Ƕȿ��ƣ�λ�ÿ��ƣ���λ����ʾ
Ŀǰ�Ѵ���ܣ������̨�����飬����λ��ͨ�ţ�UI���棨������Openmv������

���IDλ�ã�
				C --- A
				 |   |
				 |   |
				D --- B

����ռ�����������2������λ��ͨ�ţ�����3���ڿ��ƻ�е�ۣ�����5����Openmvͨ��
��Ҫ������λ��control.c�ļ���

������ 2022-2-23
**************************************************************************/
u8 Flag_Left,Flag_Right,Flag_Direction=0,Flag_Way,Flag_Next,Turn_Flag;   //����ң����صı���
u8 Flag_Stop=1; //ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
u8 t,i;
int Encoder_A,Encoder_B,Encoder_C,Encoder_D;  //���������������
long int Position_A,Position_B,Position_C,Position_D,Rate_A,Rate_B,Rate_C,Rate_D; //λ�ÿ�����ر���  

//������޸ı���
//*************************************************************************************
float Position_Rate_X=0.001,Position_Rate_Y=0.001;//λ�ü���ϵ���������
float	Position_KP=5,Position_KI=0,Position_KD=0;  //λ�ÿ���PID�����������
float Velocity_KP=10,Velocity_KI=10,Angle_KP=0.9,Angle_KI=0,Angle_KD=20;	          //�ٶȿ���PID�������������ǰ�����ٶȻ��������ǽǶȻ�
//**************************************************************************************

s8 Position_MX,Position_MY,Position_X,Position_Y,Position_Z;//��ǰλ�����꣬���㷨�ó������ù�
long int Target_X,Target_Y;//Ŀ��λ������
long int Motor_A,Motor_B,Motor_C,Motor_D; //���PWM����
long int Target_A,Target_B,Target_C,Target_D; //���Ŀ��ֵ
int Voltage;//��ص�ѹ������صı���                       
u8 delay_50,delay_flag; //��ʱ��ر���
u8 Run_Flag=0;  //�ٶȻ���λ��ģʽָʾ����
u8 rxbuf[8],Urxbuf[8],CAN_ON_Flag=0,Usart_ON_Flag=0,Usart_Flag,PID_Send;  //CAN�ʹ��ڿ�����ر���
u8 txbuf[8],txbuf2[8];  //CAN������ر���
float Pitch,Roll,Yaw,Gryo_Z,Move_X,Move_Y,Move_Z;   //����Ƕ� Z�������Ǻ�XYZ��Ŀ���ٶ�
float r,error_p,error_i,error_d,last_angle,angle_speed;//�ٶ�PID�м����
float P_error_pX,P_error_pY,P_error_i,P_error_d,P_speed,last_Position,p_rx,p_ry;//λ��PID�м����
float target_yaw;	//Ŀ��Ƕ�
int RC_Velocity=30,RC_Position=3000;         //����ң�ص��ٶȺ�λ��ֵ
int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY; //PS2��ر��� 
u16 CCD_Zhongzhi,CCD_Yuzhi,ADV[128]={0};//CCD��ر���
int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;//���Ѳ�����
int main(void)
  { 
		delay_init();	    	            //=====��ʱ������ʼ��	
	//JTAG_Set(JTAG_SWD_DISABLE);     //=====�ر�JTAG�ӿ�
		JTAG_Set(SWD_ENABLE);           //=====��SWD�ӿ� �������������SWD�ӿڵ���
		LED_Init();                     //=====��ʼ���� LED ���ӵ�Ӳ���ӿ�
	  KEY_Init();                     //=====������ʼ��
		MY_NVIC_PriorityGroupConfig(2);	//=====�����жϷ���
    MiniBalance_PWM_Init(7199,0);   //=====��ʼ��PWM 10KHZ������������� �����ʼ������ӿ� 
		uart2_init(9600);               //=====����2��ʼ��
		uart3_init(9600);             //=====����3��ʼ�� 
		uart5_init(9600);             //=====����5��ʼ�� 
		OLED_Init();                    //=====OLED��ʼ��	    
    Encoder_Init_TIM2();            //=====�������ӿ�
		Encoder_Init_TIM3();            //=====�������ӿ�
		Encoder_Init_TIM4();            //=====�������ӿ�
    Encoder_Init_TIM5();            //=====��ʼ��������
		OLED_DrawBMP(0,0,128,64,gImage_logo);
		delay_ms(1000);                  //=====��ʱ
    delay_ms(1000);                  //=====��ʱ
		delay_ms(1000);                  //=====��ʱ
		if(Run_Flag==0){ while(select())	{	}	} //=====ѡ������ģʽ 
		else Flag_Stop=0;//===λ��ģʽֱ��ʹ�ܵ��
		Position_D=0;
		delay_ms(500);                  //=====��ʱ
    IIC_Init();                     //=====IIC��ʼ��
    MPU6050_initialize();           //=====MPU6050��ʼ��	
  	DMP_Init();                     //=====��ʼ��DMP   
		CAN1_Mode_Init(1,2,3,6,0);			//=====CAN��ʼ��,������1Mbps
		Adc_Init();                     //=====adc��ʼ��		
	 if(Flag_Way==1)
	  {
		PS2_Init();											//=====PS2�ֱ���ʼ��
		PS2_SetInit();									//=====ps2���ó�ʼ��,���á����̵�ģʽ������ѡ���Ƿ�����޸�
	  }
	  else if(Flag_Way==2)ccd_Init();  //=====CCD��ʼ��
	  else if(Flag_Way==3)ele_Init();  //=====��Ŵ�������ʼ��	
	  MiniBalance_EXTI_Init();        //=====MPU6050 5ms��ʱ�жϳ�ʼ��
		USART5_TX_BUF[0]=0xa5;
		USART5_TX_BUF[USART5_REC_LEN-1]=0x5a;
    while(1)
	   {	
			   if(Flag_Way==1)
			   {
					  PS2_LX=PS2_AnologData(PSS_LX);    //PS2���ݲɼ�    
						PS2_LY=PS2_AnologData(PSS_LY);
						PS2_RX=PS2_AnologData(PSS_RX);
						PS2_RY=PS2_AnologData(PSS_RY);
						PS2_KEY=PS2_DataKey();	
					 if(PS2_KEY==16)
						 UART_DM_ReportData(DM0_Speed10_Position_0,10); //ֱ�Ӹı�λ����10��ִ�ж�������5	
					 if(PS2_KEY==14)
						 UART_DM_ReportData(DM0_Speed10_Position_90,10); 
					 if(PS2_KEY==13)
							UART_DM_ReportData(DM_Action0,5); 				
			   }
				  if(Flag_Way==0)
			   {
				  CAN1_SEND();             //CAN����		
          //USART_TX();               //���ڷ���
				 }
				 
				  USART5_TX_BUF[1]=Position_X;//����5ͨ�ţ�������Openmvͨ�š�֡ͷ0xa5��֡β0x5a��Ŀǰ��ʱ���ڵ���λ�û�
					USART5_TX_BUF[2]=Position_Y;
					USART5_TX_BUF[USART5_REC_LEN-2]=0;
				 for(i=1;i<=USART5_REC_LEN-3;i++)
					USART5_TX_BUF[USART5_REC_LEN-2]+=USART5_TX_BUF[i];				 
					 for(t=0;t<USART5_REC_LEN;t++)
					{
						USART_SendData(UART5, USART5_TX_BUF[t]);//�򴮿�5��������
						while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
					}
					
					APP_Show();	       //APP���
					oled_show();          //===��ʾ����
	  } 
}

