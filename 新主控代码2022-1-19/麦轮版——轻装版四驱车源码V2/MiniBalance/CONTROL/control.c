#include "control.h"	
#include "filter.h"	

u8 Flag_Target,Flag_Change;  //相关标志位
float Voltage_Count,Voltage_All;  //电压采样相关变量
float Gyro_K=0.004;       //陀螺仪比例系数
int j,sum;
#define a_PARAMETER          (0.095f)    
#define b_PARAMETER          (0.086f)  
/**************************************************************************
函数功能：小车运动数学模型
入口参数：X Y Z 三轴速度或者位置
返回  值：无
**************************************************************************/
void Kinematic_Analysis(float Vx,float Vy,float Vz)
{
	if(PS2_KEY==15)//陀螺模式
	{
				Target_A   = +Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER)*5-40;
        Target_B   = -Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER)*5-40;
	      Target_C   = -Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER)*5+40;
				Target_D   = +Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER)*5+40; 
	}
	else//正常模式
	{
        Target_A   = +Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER)*5;
        Target_B   = -Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER)*5;
	      Target_C   = -Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER)*5;
				Target_D   = +Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER)*5;  
	}
}
/**************************************************************************
函数功能：获取位置控制过程速度值
入口参数：X Y Z 三轴位置变化量
返回  值：无
**************************************************************************/
void Kinematic_Analysis2(float Vy,float Vz)
{
	      Rate_A   = Vy-Vz*(a_PARAMETER)*10;
        Rate_B   = Vy-Vz*(a_PARAMETER)*10;
	      Rate_C   = Vy+Vz*(a_PARAMETER)*10;
				Rate_D   = Vy+Vz*(a_PARAMETER)*10;
}
/**************************************************************************
函数功能：所有的控制代码都在这里面
         5ms定时中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步				 
**************************************************************************/
int EXTI9_5_IRQHandler(void) 
{    
	 if(INT==0)		
	{     
		   EXTI->PR=1<<5;   //清除LINE5上的中断标志位  		
		   Flag_Target=!Flag_Target;
		  if(Flag_Target==1)   //5ms读取一次陀螺仪和加速度计的值
			{
					if(Usart_Flag==0&&Usart_ON_Flag==1)  memcpy(rxbuf,Urxbuf,8*sizeof(u8));	//如果解锁了串口控制标志位，进入串口控制模式
					Read_DMP();   //===更新姿态		
					 if(Flag_Way==2)
					 {	 
								RD_TSL();  //===读取线性CCD数据 
						 	  Find_CCD_Zhongzhi(); //===提取中线 
					 }
					 if(Flag_Way==3)		
					 {
					 			Sensor_Left=Get_Adc(11);                //采集左边电感的数据
								Sensor_Right=Get_Adc(13);               //采集右边电感的数据
								Sensor_Middle=Get_Adc(12);              //采集中间电感的数据
					  	  sum=Sensor_Left*1+Sensor_Middle*100+Sensor_Right*199;  //归一化处理
								Sensor=sum/(Sensor_Left+Sensor_Middle+Sensor_Right);   //求偏差
					 }						 
			  	Key();//扫描按键变化	
			return 0;	                                               
			}     //===10ms控制一次，为了保证M法测速的时间基准，首先读取编码器数据
			Encoder_A=-Read_Encoder(2);  //===读取编码器的值
			Position_A+=Encoder_A;      //===积分得到速度   
			Encoder_B=-Read_Encoder(3);  //===读取编码器的值
			Position_B+=Encoder_B;      //===积分得到速度   
			Encoder_C=Read_Encoder(4);  //===读取编码器的值
			Position_C+=Encoder_C;      //===积分得到速度   
			Encoder_D=Read_Encoder(5);  //===读取编码器的值
			Position_D+=Encoder_D;      //===积分得到速度   
			
			//坐标计算
			Position_MX=(Position_A-Position_B-Position_C+Position_D)*Position_Rate_X;//根据轮子运动结合编码器得出当前坐标，准确的未知，有待测试
			Position_MY=(-Position_A-Position_B-Position_C-Position_D)*Position_Rate_Y;
			Position_X=Position_MX*cos(Yaw*3.141593f/180)-Position_MY*sin(Yaw*3.141593f/180);//yaw角度顺时针为负数
			Position_Y=Position_MY*cos(Yaw*3.141593f/180)+Position_MX*sin(Yaw*3.141593f/180);
			
			if(Position_X>=127)Position_X=127;//受字节限制，只能发到127
			if(Position_X<=-127)Position_X=-127;
			if(Position_Y>=127)Position_Y=127;
			if(Position_Y<=-127)Position_Y=-127;
			
	  	Read_DMP();  //===更新姿态	
  		Led_Flash(100);  //===LED闪烁;常规模式 1s改变一次指示灯的状态	
			Voltage_All+=Get_battery_volt(); //多次采样累积
			if(++Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//求平均值 获取电池电压	   
		 if(Turn_Off(Voltage)==0)   //===如果电池电压不存在异常
		 { 			 
		  if(Run_Flag==0)//速度模式
			{		
			  Get_RC();   //运动数据解析函数
				Motor_A=Incremental_PI_A(Encoder_A,Target_A);  //===速度闭环控制计算电机A最终PWM
				Motor_B=Incremental_PI_B(Encoder_B,Target_B);  //===速度闭环控制计算电机B最终PWM
				Motor_C=Incremental_PI_C(Encoder_C,Target_C);  //===速度闭环控制计算电机C最终PWM
		    Motor_D=Incremental_PI_D(Encoder_D,Target_D);  //===速度闭环控制计算电机C最终PWM
			}
		 Xianfu_Pwm(6900);                     //===PWM限幅
		 Set_Pwm(Motor_A,Motor_B,Motor_C,Motor_D);     //===赋值给PWM寄存器  
		 }
 }
	 return 0;	 
} 
/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d)
{
	  	if(motor_a>0)			PWMA1=7200,PWMA2=7200-motor_a;
			else 	            PWMA2=7200,PWMA1=7200+motor_a;
		
		  if(motor_b>0)			PWMB1=7200,PWMB2=7200-motor_b;
			else 	            PWMB2=7200,PWMB1=7200+motor_b;
	
	    if(motor_c>0)			PWMC1=7200,PWMC2=7200-motor_c;
			else 	            PWMC2=7200,PWMC1=7200+motor_c;
		
	    if(motor_d>0)			PWMD1=7200,PWMD2=7200-motor_d;
			else 	            PWMD2=7200,PWMD1=7200+motor_d;
}
/**************************************************************************
函数功能：限制PWM赋值 
入口参数：幅值
返回  值：无
**************************************************************************/
void Xianfu_Pwm(int amplitude)
{	
    if(Motor_A<-amplitude) Motor_A=-amplitude;	
		if(Motor_A>amplitude)  Motor_A=amplitude;	
	  if(Motor_B<-amplitude) Motor_B=-amplitude;	
		if(Motor_B>amplitude)  Motor_B=amplitude;		
	  if(Motor_C<-amplitude) Motor_C=-amplitude;	
		if(Motor_C>amplitude)  Motor_C=amplitude;		
	  if(Motor_D<-amplitude) Motor_D=-amplitude;	
	  if(Motor_D>amplitude)  Motor_D=amplitude;		
}
/**************************************************************************
函数功能：位置PID控制过程中速度的设置
入口参数：无、幅值
返回  值：无
**************************************************************************/
void Xianfu_Velocity(int amplitude_A,int amplitude_B,int amplitude_C,int amplitude_D)
{	
    if(Motor_A<-amplitude_A) Motor_A=-amplitude_A;	//位置控制模式中，A电机的运行速度
		if(Motor_A>amplitude_A)  Motor_A=amplitude_A;	  //位置控制模式中，A电机的运行速度
	  if(Motor_B<-amplitude_B) Motor_B=-amplitude_B;	//位置控制模式中，B电机的运行速度
		if(Motor_B>amplitude_B)  Motor_B=amplitude_B;		//位置控制模式中，B电机的运行速度
	  if(Motor_C<-amplitude_C) Motor_C=-amplitude_C;	//位置控制模式中，C电机的运行速度
		if(Motor_C>amplitude_C)  Motor_C=amplitude_C;		//位置控制模式中，C电机的运行速度
		if(Motor_D<-amplitude_D) Motor_D=-amplitude_D;	//位置控制模式中，C电机的运行速度
		if(Motor_D>amplitude_D)  Motor_D=amplitude_D;		//位置控制模式中，C电机的运行速度
}
/**************************************************************************
函数功能：按键修改小车运行状态 
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{	
	u8 tmp;
	tmp=click_N_Double(50); 
	if(tmp==1)Flag_Stop=!Flag_Stop;//单击控制电机启停                  
}
/**************************************************************************
函数功能：异常关闭电机
入口参数：电压
返回  值：1：异常  0：正常
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<400||Flag_Stop==1)//电池电压低于4.0V关闭电机
			{	                                                
      temp=1;      
     	PWMA1=0; //电机控制位清零                                           
			PWMB1=0; //电机控制位清零
			PWMC1=0; //电机控制位清零
			PWMD1=0; //电机控制位清零
			PWMA2=0; //电机控制位清零
			PWMB2=0; //电机控制位清零
			PWMC2=0; //电机控制位清零	
      PWMD2=0; //电机控制位清零						
      }
			else
      temp=0;
      return temp;			
}

/**************************************************************************
函数功能：绝对值函数
入口参数：long int
返回  值：unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return Pwm;                         //增量输出
}
int Incremental_PI_B (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return Pwm;                         //增量输出
}
int Incremental_PI_C (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                                  //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return Pwm;                         //增量输出
}
int Incremental_PI_D (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                                  //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return Pwm;                         //增量输出
}

//角度控制PID
float angle_control(float now_angle,float target_angle)
{
	if(target_angle>179)
	{
		target_angle=-175;
	}
	if(target_angle<-179)
	{
		target_angle=175;
	}
	
	
	if(now_angle/target_angle<=0&&fabs(target_angle)>100)//过0点检测，有待改进
		error_p=target_angle+now_angle;
	else
		error_p=-Angle_KP*(now_angle-target_angle);
	
	
		if(now_angle-target_angle>=8||now_angle-target_angle<=-8)
				error_i+=Angle_KI*error_p;
		else
			  error_i=0;
		error_d=-Angle_KD*angle_speed;
	angle_speed=now_angle-last_angle;
	r=error_p+error_i+error_d;
	last_angle=now_angle;
	return r;
}

//位置控制PID
float positionX_control(s8 Position,s8 target)
{
		P_error_pX=-(Position-target)*Position_KP;
		
		if(Position-target>=3||Position-target<=-3)
				P_error_i+=Position_KI*P_error_pX;
		else
			  P_error_i=0;
		
		P_error_d=-Position_KD*P_speed;
		
	P_speed=Position-last_Position;
	p_rx=P_error_pX+P_error_i+P_error_d;
	last_Position=Position;
	return p_rx;
}

float positionY_control(s8 Position,s8 target)
{
		P_error_pY=(Position-target)*Position_KP;
		
		if(Position-target>=3||Position-target<=-3)
				P_error_i+=Position_KI*P_error_pY;
		else
			  P_error_i=0;
		
		P_error_d=-Position_KD*P_speed;
		
	P_speed=Position-last_Position;
	p_ry=P_error_pY+P_error_i+P_error_d;
	last_Position=Position;
	return p_ry;
}
/**************************************************************************
函数功能：通过指令对小车进行遥控
入口参数：串口指令
返回  值：无
**************************************************************************/
void Get_RC(void)
{
	float step=0.5;   //设置速度控制步进值?
	u8 Flag_Move=1;
	int Yuzhi=2;  		//PS2控制防抖阈值
	float LX,LY,RX,RY;  //PS2手柄控制变量
	static float Bias,Last_Bias;  //偏差和历史值
//	int flag_Y,flag_Z;
	if(Flag_Way==0)//串口等控制方式
	{
		if(CAN_ON_Flag==0&&Usart_ON_Flag==0) 
			{
         switch(Flag_Direction)   //方向控制
				 {
				  case 1:  Move_X=0;           Move_Y+=step;  Flag_Move=1;               break;
				  case 2:  Move_X+=step;       Move_Y+=step;  Flag_Move=1;               break;
				  case 3:  Move_X+=step;       Move_Y=0;      Flag_Move=1;               break;
				  case 4:  Move_X+=step;       Move_Y-=step;  Flag_Move=1;               break;
				  case 5:  Move_X=0;           Move_Y-=step;  Flag_Move=1;               break;
				  case 6:  Move_X-=step;       Move_Y-=step;  Flag_Move=1;               break;
				  case 7:  Move_X-=step;       Move_Y=0;      Flag_Move=1;               break;
				  case 8:  Move_X-=step;       Move_Y+=step;  Flag_Move=1;               break; 
				  default: Flag_Move=0;   		  Move_X=Move_X/1.05;Move_Y=Move_Y/1.05;      break;	 
			   }
				 
			 if(Flag_Move==0)		//如果无方向控制指令	 ，检查转向控制状态
			   {	
			  	if(Flag_Left==1)        Move_Z-=step,Gyro_K=0;    //左自旋   
				  else if(Flag_Right==1)  Move_Z+=step,Gyro_K=0;    //右自旋		
				  else 		                Move_Z=0,Gyro_K=0.004;    //停止
			   }	
			 if(Flag_Move==1)	Flag_Left=0,Flag_Right=0,Move_Z=0;
			 if(Move_X<-RC_Velocity) Move_X=-RC_Velocity;	   //速度控制限幅
			 if(Move_X>RC_Velocity)  Move_X=RC_Velocity;	     
			 if(Move_Y<-RC_Velocity) Move_Y=-RC_Velocity;	
			 if(Move_Y>RC_Velocity)  Move_Y=RC_Velocity;	 
			 if(Move_Z<-RC_Velocity) Move_Z=-RC_Velocity;	
			 if(Move_Z>RC_Velocity)  Move_Z=RC_Velocity;	 
			}		
			else
				{
					if(rxbuf[1]==0)Move_Y=rxbuf[0]; //识别运动方向
					else           Move_Y=-rxbuf[0]; //速度
					if(rxbuf[3]==0)Move_Z=rxbuf[2]; //识别运动方向
					else           Move_Z=-rxbuf[2]; //速度			 
				}
	 }
		if(Flag_Way==1)//PS2控制
		{
			LX=PS2_LX-128; //获取偏差
			RY=PS2_RY-128; //获取偏差
			LY=PS2_LY-128; //获取偏差
			RX=PS2_RX-128; //获取偏差
			if(RY>-Yuzhi&&RY<Yuzhi)RY=0; //设置小角度的死区
			if(LY>-Yuzhi&&LY<Yuzhi)LY=0; //设置小角度的死区
			if(RX>-Yuzhi&&RX<Yuzhi)RX=0; //设置小角度的死区
			Move_X=LX*RC_Velocity/200; //三轴运动目标值
			Move_Y=LY*RC_Velocity/200;	
			
			if(PS2_KEY==15)//陀螺模式
			{
				Move_X=(Move_X*cos(Yaw*3.141593f/180)-Move_Y*sin(Yaw*3.141593f/180))*1.5;
				Move_Y=(Move_X*sin(Yaw*3.141593f/180)+Move_Y*cos(Yaw*3.141593f/180))*1.5;
				Move_Z=0;
			}
			else//正常模式
			{				 
				 target_yaw-=RX/1000;
				if(target_yaw>179)
					target_yaw=-179;
				if(target_yaw<-179)
					target_yaw=179;
				 //Move_Z=-RX*RC_Velocity/200;
				 Move_Z=angle_control(Yaw,target_yaw);	
			}
		}
		if(Flag_Way==2)//CCD巡线
		{
			 RC_Velocity=45;//巡线速度
	   	 Move_Y=RC_Velocity;
			 Bias=CCD_Zhongzhi-64;   //提取偏差
			 Move_Z=-Bias*0.8-(Bias-Last_Bias)*5; //PD控制
			 Last_Bias=Bias;   //保存上一次的偏差
		}
		if(Flag_Way==3)//电磁巡线
		{
			 RC_Velocity=45;//巡线速度
			 Move_Y=RC_Velocity;
			 Bias=100-Sensor;  //提取偏差
			 Move_Z=abs(Bias)*Bias*0.01+Bias*0.1+(Bias-Last_Bias)*9; //
			 Last_Bias=Bias;   //上一次的偏差
		}	
		if(Flag_Way==4)//位置控制
		{		
			Target_X=USART5_RX_BUFdebug[1];//上位机发来目标位置坐标
			Target_Y=USART5_RX_BUFdebug[2];
			target_yaw=USART5_RX_BUFdebug[3];
			Move_X=positionX_control(Position_X,Target_X); //三轴运动目标值
			Move_Y=positionY_control(Position_Y,Target_Y); //三轴运动目标值		
			Move_Z=angle_control(Yaw,target_yaw);				 
		}
		Kinematic_Analysis(Move_X,Move_Y,Move_Z);//得到控制目标值，进行运动学分析
}

/**************************************************************************
函数功能：线性CCD取中值
入口参数：无
返回  值：无
**************************************************************************/
void  Find_CCD_Zhongzhi(void)
{ 
	 static u16 i,j,Left,Right,Last_CCD_Zhongzhi;
	 static u16 value1_max,value1_min;
	
	   value1_max=ADV[0];  //动态阈值算法，读取最大和最小值
     for(i=5;i<123;i++)   //两边各去掉5个点
     {
        if(value1_max<=ADV[i])
        value1_max=ADV[i];
     }
	   value1_min=ADV[0];  //最小值
     for(i=5;i<123;i++) 
     {
        if(value1_min>=ADV[i])
        value1_min=ADV[i];
     }
   CCD_Yuzhi=(value1_max+value1_min)/2;	  //计算出本次中线提取的阈值
	 for(i = 5;i<118; i++)   //寻找左边跳变沿
	{
		if(ADV[i]>CCD_Yuzhi&&ADV[i+1]>CCD_Yuzhi&&ADV[i+2]>CCD_Yuzhi&&ADV[i+3]<CCD_Yuzhi&&ADV[i+4]<CCD_Yuzhi&&ADV[i+5]<CCD_Yuzhi)
		{	
			Left=i;
			break;	
		}
	}
	 for(j = 118;j>5; j--)//寻找右边跳变沿
  {
		if(ADV[j]<CCD_Yuzhi&&ADV[j+1]<CCD_Yuzhi&&ADV[j+2]<CCD_Yuzhi&&ADV[j+3]>CCD_Yuzhi&&ADV[j+4]>CCD_Yuzhi&&ADV[j+5]>CCD_Yuzhi)
		{	
		  Right=j;
		  break;	
		}
  }
	CCD_Zhongzhi=(Right+Left)/2;//计算中线位置
	if(myabs(CCD_Zhongzhi-Last_CCD_Zhongzhi)>90)   //计算中线的偏差，如果太大
	CCD_Zhongzhi=Last_CCD_Zhongzhi;    //则取上一次的值
	Last_CCD_Zhongzhi=CCD_Zhongzhi;  //保存上一次的偏差
}
