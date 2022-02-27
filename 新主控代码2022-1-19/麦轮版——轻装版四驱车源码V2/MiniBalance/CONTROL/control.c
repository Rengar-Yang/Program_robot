#include "control.h"	
#include "filter.h"	

u8 Flag_Target,Flag_Change;  //Ïà¹Ø±êÖ¾Î»
float Voltage_Count,Voltage_All;  //µçÑ¹²ÉÑùÏà¹Ø±äÁ¿
float Gyro_K=0.004;       //ÍÓÂİÒÇ±ÈÀıÏµÊı
int j,sum;
#define a_PARAMETER          (0.095f)    
#define b_PARAMETER          (0.086f)  
/**************************************************************************
º¯Êı¹¦ÄÜ£ºĞ¡³µÔË¶¯ÊıÑ§Ä£ĞÍ
Èë¿Ú²ÎÊı£ºX Y Z ÈıÖáËÙ¶È»òÕßÎ»ÖÃ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void Kinematic_Analysis(float Vx,float Vy,float Vz)
{
	if(PS2_KEY==15)//ÍÓÂİÄ£Ê½
	{
				Target_A   = +Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER)*5-40;
        Target_B   = -Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER)*5-40;
	      Target_C   = -Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER)*5+40;
				Target_D   = +Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER)*5+40; 
	}
	else//Õı³£Ä£Ê½
	{
        Target_A   = +Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER)*5;
        Target_B   = -Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER)*5;
	      Target_C   = -Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER)*5;
				Target_D   = +Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER)*5;  
	}
}
/**************************************************************************
º¯Êı¹¦ÄÜ£º»ñÈ¡Î»ÖÃ¿ØÖÆ¹ı³ÌËÙ¶ÈÖµ
Èë¿Ú²ÎÊı£ºX Y Z ÈıÖáÎ»ÖÃ±ä»¯Á¿
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void Kinematic_Analysis2(float Vy,float Vz)
{
	      Rate_A   = Vy-Vz*(a_PARAMETER)*10;
        Rate_B   = Vy-Vz*(a_PARAMETER)*10;
	      Rate_C   = Vy+Vz*(a_PARAMETER)*10;
				Rate_D   = Vy+Vz*(a_PARAMETER)*10;
}
/**************************************************************************
º¯Êı¹¦ÄÜ£ºËùÓĞµÄ¿ØÖÆ´úÂë¶¼ÔÚÕâÀïÃæ
         5ms¶¨Ê±ÖĞ¶ÏÓÉMPU6050µÄINTÒı½Å´¥·¢
         ÑÏ¸ñ±£Ö¤²ÉÑùºÍÊı¾İ´¦ÀíµÄÊ±¼äÍ¬²½				 
**************************************************************************/
int EXTI9_5_IRQHandler(void) 
{    
	 if(INT==0)		
	{     
		   EXTI->PR=1<<5;   //Çå³ıLINE5ÉÏµÄÖĞ¶Ï±êÖ¾Î»  		
		   Flag_Target=!Flag_Target;
		  if(Flag_Target==1)   //5ms¶ÁÈ¡Ò»´ÎÍÓÂİÒÇºÍ¼ÓËÙ¶È¼ÆµÄÖµ
			{
					if(Usart_Flag==0&&Usart_ON_Flag==1)  memcpy(rxbuf,Urxbuf,8*sizeof(u8));	//Èç¹û½âËøÁË´®¿Ú¿ØÖÆ±êÖ¾Î»£¬½øÈë´®¿Ú¿ØÖÆÄ£Ê½
					Read_DMP();   //===¸üĞÂ×ËÌ¬		
					 if(Flag_Way==2)
					 {	 
								RD_TSL();  //===¶ÁÈ¡ÏßĞÔCCDÊı¾İ 
						 	  Find_CCD_Zhongzhi(); //===ÌáÈ¡ÖĞÏß 
					 }
					 if(Flag_Way==3)		
					 {
					 			Sensor_Left=Get_Adc(11);                //²É¼¯×ó±ßµç¸ĞµÄÊı¾İ
								Sensor_Right=Get_Adc(13);               //²É¼¯ÓÒ±ßµç¸ĞµÄÊı¾İ
								Sensor_Middle=Get_Adc(12);              //²É¼¯ÖĞ¼äµç¸ĞµÄÊı¾İ
					  	  sum=Sensor_Left*1+Sensor_Middle*100+Sensor_Right*199;  //¹éÒ»»¯´¦Àí
								Sensor=sum/(Sensor_Left+Sensor_Middle+Sensor_Right);   //ÇóÆ«²î
					 }						 
			  	Key();//É¨Ãè°´¼ü±ä»¯	
			return 0;	                                               
			}     //===10ms¿ØÖÆÒ»´Î£¬ÎªÁË±£Ö¤M·¨²âËÙµÄÊ±¼ä»ù×¼£¬Ê×ÏÈ¶ÁÈ¡±àÂëÆ÷Êı¾İ
			Encoder_A=-Read_Encoder(2);  //===¶ÁÈ¡±àÂëÆ÷µÄÖµ
			Position_A+=Encoder_A;      //===»ı·ÖµÃµ½ËÙ¶È   
			Encoder_B=-Read_Encoder(3);  //===¶ÁÈ¡±àÂëÆ÷µÄÖµ
			Position_B+=Encoder_B;      //===»ı·ÖµÃµ½ËÙ¶È   
			Encoder_C=Read_Encoder(4);  //===¶ÁÈ¡±àÂëÆ÷µÄÖµ
			Position_C+=Encoder_C;      //===»ı·ÖµÃµ½ËÙ¶È   
			Encoder_D=Read_Encoder(5);  //===¶ÁÈ¡±àÂëÆ÷µÄÖµ
			Position_D+=Encoder_D;      //===»ı·ÖµÃµ½ËÙ¶È   
			
			//×ø±ê¼ÆËã
			Position_MX=(Position_A-Position_B-Position_C+Position_D)*Position_Rate_X;//¸ù¾İÂÖ×ÓÔË¶¯½áºÏ±àÂëÆ÷µÃ³öµ±Ç°×ø±ê£¬×¼È·µÄÎ´Öª£¬ÓĞ´ı²âÊÔ
			Position_MY=(-Position_A-Position_B-Position_C-Position_D)*Position_Rate_Y;
			Position_X=Position_MX*cos(Yaw*3.141593f/180)-Position_MY*sin(Yaw*3.141593f/180);//yaw½Ç¶ÈË³Ê±ÕëÎª¸ºÊı
			Position_Y=Position_MY*cos(Yaw*3.141593f/180)+Position_MX*sin(Yaw*3.141593f/180);
			
			if(Position_X>=127)Position_X=127;//ÊÜ×Ö½ÚÏŞÖÆ£¬Ö»ÄÜ·¢µ½127
			if(Position_X<=-127)Position_X=-127;
			if(Position_Y>=127)Position_Y=127;
			if(Position_Y<=-127)Position_Y=-127;
			
	  	Read_DMP();  //===¸üĞÂ×ËÌ¬	
  		Led_Flash(100);  //===LEDÉÁË¸;³£¹æÄ£Ê½ 1s¸Ä±äÒ»´ÎÖ¸Ê¾µÆµÄ×´Ì¬	
			Voltage_All+=Get_battery_volt(); //¶à´Î²ÉÑùÀÛ»ı
			if(++Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//ÇóÆ½¾ùÖµ »ñÈ¡µç³ØµçÑ¹	   
		 if(Turn_Off(Voltage)==0)   //===Èç¹ûµç³ØµçÑ¹²»´æÔÚÒì³£
		 { 			 
		  if(Run_Flag==0)//ËÙ¶ÈÄ£Ê½
			{		
			  Get_RC();   //ÔË¶¯Êı¾İ½âÎöº¯Êı
				Motor_A=Incremental_PI_A(Encoder_A,Target_A);  //===ËÙ¶È±Õ»·¿ØÖÆ¼ÆËãµç»úA×îÖÕPWM
				Motor_B=Incremental_PI_B(Encoder_B,Target_B);  //===ËÙ¶È±Õ»·¿ØÖÆ¼ÆËãµç»úB×îÖÕPWM
				Motor_C=Incremental_PI_C(Encoder_C,Target_C);  //===ËÙ¶È±Õ»·¿ØÖÆ¼ÆËãµç»úC×îÖÕPWM
		    Motor_D=Incremental_PI_D(Encoder_D,Target_D);  //===ËÙ¶È±Õ»·¿ØÖÆ¼ÆËãµç»úC×îÖÕPWM
			}
		 Xianfu_Pwm(6900);                     //===PWMÏŞ·ù
		 Set_Pwm(Motor_A,Motor_B,Motor_C,Motor_D);     //===¸³Öµ¸øPWM¼Ä´æÆ÷  
		 }
 }
	 return 0;	 
} 
/**************************************************************************
º¯Êı¹¦ÄÜ£º¸³Öµ¸øPWM¼Ä´æÆ÷
Èë¿Ú²ÎÊı£ºPWM
·µ»Ø  Öµ£ºÎŞ
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
º¯Êı¹¦ÄÜ£ºÏŞÖÆPWM¸³Öµ 
Èë¿Ú²ÎÊı£º·ùÖµ
·µ»Ø  Öµ£ºÎŞ
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
º¯Êı¹¦ÄÜ£ºÎ»ÖÃPID¿ØÖÆ¹ı³ÌÖĞËÙ¶ÈµÄÉèÖÃ
Èë¿Ú²ÎÊı£ºÎŞ¡¢·ùÖµ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void Xianfu_Velocity(int amplitude_A,int amplitude_B,int amplitude_C,int amplitude_D)
{	
    if(Motor_A<-amplitude_A) Motor_A=-amplitude_A;	//Î»ÖÃ¿ØÖÆÄ£Ê½ÖĞ£¬Aµç»úµÄÔËĞĞËÙ¶È
		if(Motor_A>amplitude_A)  Motor_A=amplitude_A;	  //Î»ÖÃ¿ØÖÆÄ£Ê½ÖĞ£¬Aµç»úµÄÔËĞĞËÙ¶È
	  if(Motor_B<-amplitude_B) Motor_B=-amplitude_B;	//Î»ÖÃ¿ØÖÆÄ£Ê½ÖĞ£¬Bµç»úµÄÔËĞĞËÙ¶È
		if(Motor_B>amplitude_B)  Motor_B=amplitude_B;		//Î»ÖÃ¿ØÖÆÄ£Ê½ÖĞ£¬Bµç»úµÄÔËĞĞËÙ¶È
	  if(Motor_C<-amplitude_C) Motor_C=-amplitude_C;	//Î»ÖÃ¿ØÖÆÄ£Ê½ÖĞ£¬Cµç»úµÄÔËĞĞËÙ¶È
		if(Motor_C>amplitude_C)  Motor_C=amplitude_C;		//Î»ÖÃ¿ØÖÆÄ£Ê½ÖĞ£¬Cµç»úµÄÔËĞĞËÙ¶È
		if(Motor_D<-amplitude_D) Motor_D=-amplitude_D;	//Î»ÖÃ¿ØÖÆÄ£Ê½ÖĞ£¬Cµç»úµÄÔËĞĞËÙ¶È
		if(Motor_D>amplitude_D)  Motor_D=amplitude_D;		//Î»ÖÃ¿ØÖÆÄ£Ê½ÖĞ£¬Cµç»úµÄÔËĞĞËÙ¶È
}
/**************************************************************************
º¯Êı¹¦ÄÜ£º°´¼üĞŞ¸ÄĞ¡³µÔËĞĞ×´Ì¬ 
Èë¿Ú²ÎÊı£ºÎŞ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void Key(void)
{	
	u8 tmp;
	tmp=click_N_Double(50); 
	if(tmp==1)Flag_Stop=!Flag_Stop;//µ¥»÷¿ØÖÆµç»úÆôÍ£                  
}
/**************************************************************************
º¯Êı¹¦ÄÜ£ºÒì³£¹Ø±Õµç»ú
Èë¿Ú²ÎÊı£ºµçÑ¹
·µ»Ø  Öµ£º1£ºÒì³£  0£ºÕı³£
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<400||Flag_Stop==1)//µç³ØµçÑ¹µÍÓÚ4.0V¹Ø±Õµç»ú
			{	                                                
      temp=1;      
     	PWMA1=0; //µç»ú¿ØÖÆÎ»ÇåÁã                                           
			PWMB1=0; //µç»ú¿ØÖÆÎ»ÇåÁã
			PWMC1=0; //µç»ú¿ØÖÆÎ»ÇåÁã
			PWMD1=0; //µç»ú¿ØÖÆÎ»ÇåÁã
			PWMA2=0; //µç»ú¿ØÖÆÎ»ÇåÁã
			PWMB2=0; //µç»ú¿ØÖÆÎ»ÇåÁã
			PWMC2=0; //µç»ú¿ØÖÆÎ»ÇåÁã	
      PWMD2=0; //µç»ú¿ØÖÆÎ»ÇåÁã						
      }
			else
      temp=0;
      return temp;			
}

/**************************************************************************
º¯Êı¹¦ÄÜ£º¾ø¶ÔÖµº¯Êı
Èë¿Ú²ÎÊı£ºlong int
·µ»Ø  Öµ£ºunsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
º¯Êı¹¦ÄÜ£ºÔöÁ¿PI¿ØÖÆÆ÷
Èë¿Ú²ÎÊı£º±àÂëÆ÷²âÁ¿Öµ£¬Ä¿±êËÙ¶È
·µ»Ø  Öµ£ºµç»úPWM
¸ù¾İÔöÁ¿Ê½ÀëÉ¢PID¹«Ê½ 
pwm+=Kp[e£¨k£©-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)´ú±í±¾´ÎÆ«²î 
e(k-1)´ú±íÉÏÒ»´ÎµÄÆ«²î  ÒÔ´ËÀàÍÆ 
pwm´ú±íÔöÁ¿Êä³ö
ÔÚÎÒÃÇµÄËÙ¶È¿ØÖÆ±Õ»·ÏµÍ³ÀïÃæ£¬Ö»Ê¹ÓÃPI¿ØÖÆ
pwm+=Kp[e£¨k£©-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //¼ÆËãÆ«²î
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //ÔöÁ¿Ê½PI¿ØÖÆÆ÷
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //±£´æÉÏÒ»´ÎÆ«²î 
	 return Pwm;                         //ÔöÁ¿Êä³ö
}
int Incremental_PI_B (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //¼ÆËãÆ«²î
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //ÔöÁ¿Ê½PI¿ØÖÆÆ÷
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //±£´æÉÏÒ»´ÎÆ«²î 
	 return Pwm;                         //ÔöÁ¿Êä³ö
}
int Incremental_PI_C (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                                  //¼ÆËãÆ«²î
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //ÔöÁ¿Ê½PI¿ØÖÆÆ÷
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //±£´æÉÏÒ»´ÎÆ«²î 
	 return Pwm;                         //ÔöÁ¿Êä³ö
}
int Incremental_PI_D (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                                  //¼ÆËãÆ«²î
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //ÔöÁ¿Ê½PI¿ØÖÆÆ÷
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //±£´æÉÏÒ»´ÎÆ«²î 
	 return Pwm;                         //ÔöÁ¿Êä³ö
}

//½Ç¶È¿ØÖÆPID
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
	
	
	if(now_angle/target_angle<=0&&fabs(target_angle)>100)//¹ı0µã¼ì²â£¬ÓĞ´ı¸Ä½ø
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

//Î»ÖÃ¿ØÖÆPID
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
º¯Êı¹¦ÄÜ£ºÍ¨¹ıÖ¸Áî¶ÔĞ¡³µ½øĞĞÒ£¿Ø
Èë¿Ú²ÎÊı£º´®¿ÚÖ¸Áî
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void Get_RC(void)
{
	float step=0.5;   //ÉèÖÃËÙ¶È¿ØÖÆ²½½øÖµ¡
	u8 Flag_Move=1;
	int Yuzhi=2;  		//PS2¿ØÖÆ·À¶¶ãĞÖµ
	float LX,LY,RX,RY;  //PS2ÊÖ±ú¿ØÖÆ±äÁ¿
	static float Bias,Last_Bias;  //Æ«²îºÍÀúÊ·Öµ
//	int flag_Y,flag_Z;
	if(Flag_Way==0)//´®¿ÚµÈ¿ØÖÆ·½Ê½
	{
		if(CAN_ON_Flag==0&&Usart_ON_Flag==0) 
			{
         switch(Flag_Direction)   //·½Ïò¿ØÖÆ
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
				 
			 if(Flag_Move==0)		//Èç¹ûÎŞ·½Ïò¿ØÖÆÖ¸Áî	 £¬¼ì²é×ªÏò¿ØÖÆ×´Ì¬
			   {	
			  	if(Flag_Left==1)        Move_Z-=step,Gyro_K=0;    //×ó×ÔĞı   
				  else if(Flag_Right==1)  Move_Z+=step,Gyro_K=0;    //ÓÒ×ÔĞı		
				  else 		                Move_Z=0,Gyro_K=0.004;    //Í£Ö¹
			   }	
			 if(Flag_Move==1)	Flag_Left=0,Flag_Right=0,Move_Z=0;
			 if(Move_X<-RC_Velocity) Move_X=-RC_Velocity;	   //ËÙ¶È¿ØÖÆÏŞ·ù
			 if(Move_X>RC_Velocity)  Move_X=RC_Velocity;	     
			 if(Move_Y<-RC_Velocity) Move_Y=-RC_Velocity;	
			 if(Move_Y>RC_Velocity)  Move_Y=RC_Velocity;	 
			 if(Move_Z<-RC_Velocity) Move_Z=-RC_Velocity;	
			 if(Move_Z>RC_Velocity)  Move_Z=RC_Velocity;	 
			}		
			else
				{
					if(rxbuf[1]==0)Move_Y=rxbuf[0]; //Ê¶±ğÔË¶¯·½Ïò
					else           Move_Y=-rxbuf[0]; //ËÙ¶È
					if(rxbuf[3]==0)Move_Z=rxbuf[2]; //Ê¶±ğÔË¶¯·½Ïò
					else           Move_Z=-rxbuf[2]; //ËÙ¶È			 
				}
	 }
		if(Flag_Way==1)//PS2¿ØÖÆ
		{
			LX=PS2_LX-128; //»ñÈ¡Æ«²î
			RY=PS2_RY-128; //»ñÈ¡Æ«²î
			LY=PS2_LY-128; //»ñÈ¡Æ«²î
			RX=PS2_RX-128; //»ñÈ¡Æ«²î
			if(RY>-Yuzhi&&RY<Yuzhi)RY=0; //ÉèÖÃĞ¡½Ç¶ÈµÄËÀÇø
			if(LY>-Yuzhi&&LY<Yuzhi)LY=0; //ÉèÖÃĞ¡½Ç¶ÈµÄËÀÇø
			if(RX>-Yuzhi&&RX<Yuzhi)RX=0; //ÉèÖÃĞ¡½Ç¶ÈµÄËÀÇø
			Move_X=LX*RC_Velocity/200; //ÈıÖáÔË¶¯Ä¿±êÖµ
			Move_Y=LY*RC_Velocity/200;	
			
			if(PS2_KEY==15)//ÍÓÂİÄ£Ê½
			{
				Move_X=(Move_X*cos(Yaw*3.141593f/180)-Move_Y*sin(Yaw*3.141593f/180))*1.5;
				Move_Y=(Move_X*sin(Yaw*3.141593f/180)+Move_Y*cos(Yaw*3.141593f/180))*1.5;
				Move_Z=0;
			}
			else//Õı³£Ä£Ê½
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
		if(Flag_Way==2)//CCDÑ²Ïß
		{
			 RC_Velocity=45;//Ñ²ÏßËÙ¶È
	   	 Move_Y=RC_Velocity;
			 Bias=CCD_Zhongzhi-64;   //ÌáÈ¡Æ«²î
			 Move_Z=-Bias*0.8-(Bias-Last_Bias)*5; //PD¿ØÖÆ
			 Last_Bias=Bias;   //±£´æÉÏÒ»´ÎµÄÆ«²î
		}
		if(Flag_Way==3)//µç´ÅÑ²Ïß
		{
			 RC_Velocity=45;//Ñ²ÏßËÙ¶È
			 Move_Y=RC_Velocity;
			 Bias=100-Sensor;  //ÌáÈ¡Æ«²î
			 Move_Z=abs(Bias)*Bias*0.01+Bias*0.1+(Bias-Last_Bias)*9; //
			 Last_Bias=Bias;   //ÉÏÒ»´ÎµÄÆ«²î
		}	
		if(Flag_Way==4)//Î»ÖÃ¿ØÖÆ
		{		
			Target_X=USART5_RX_BUF[1];//ÉÏÎ»»ú·¢À´Ä¿±êÎ»ÖÃ×ø±ê
			Target_Y=USART5_RX_BUF[2];
			target_yaw=USART5_RX_BUF[3];
			Move_X=positionX_control(Position_X,Target_X); //ÈıÖáÔË¶¯Ä¿±êÖµ
			Move_Y=positionY_control(Position_Y,Target_Y); //ÈıÖáÔË¶¯Ä¿±êÖµ		
			Move_Z=angle_control(Yaw,target_yaw);				 
		}
		Kinematic_Analysis(Move_X,Move_Y,Move_Z);//µÃµ½¿ØÖÆÄ¿±êÖµ£¬½øĞĞÔË¶¯Ñ§·ÖÎö
}

/**************************************************************************
º¯Êı¹¦ÄÜ£ºÏßĞÔCCDÈ¡ÖĞÖµ
Èë¿Ú²ÎÊı£ºÎŞ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void  Find_CCD_Zhongzhi(void)
{ 
	 static u16 i,j,Left,Right,Last_CCD_Zhongzhi;
	 static u16 value1_max,value1_min;
	
	   value1_max=ADV[0];  //¶¯Ì¬ãĞÖµËã·¨£¬¶ÁÈ¡×î´óºÍ×îĞ¡Öµ
     for(i=5;i<123;i++)   //Á½±ß¸÷È¥µô5¸öµã
     {
        if(value1_max<=ADV[i])
        value1_max=ADV[i];
     }
	   value1_min=ADV[0];  //×îĞ¡Öµ
     for(i=5;i<123;i++) 
     {
        if(value1_min>=ADV[i])
        value1_min=ADV[i];
     }
   CCD_Yuzhi=(value1_max+value1_min)/2;	  //¼ÆËã³ö±¾´ÎÖĞÏßÌáÈ¡µÄãĞÖµ
	 for(i = 5;i<118; i++)   //Ñ°ÕÒ×ó±ßÌø±äÑØ
	{
		if(ADV[i]>CCD_Yuzhi&&ADV[i+1]>CCD_Yuzhi&&ADV[i+2]>CCD_Yuzhi&&ADV[i+3]<CCD_Yuzhi&&ADV[i+4]<CCD_Yuzhi&&ADV[i+5]<CCD_Yuzhi)
		{	
			Left=i;
			break;	
		}
	}
	 for(j = 118;j>5; j--)//Ñ°ÕÒÓÒ±ßÌø±äÑØ
  {
		if(ADV[j]<CCD_Yuzhi&&ADV[j+1]<CCD_Yuzhi&&ADV[j+2]<CCD_Yuzhi&&ADV[j+3]>CCD_Yuzhi&&ADV[j+4]>CCD_Yuzhi&&ADV[j+5]>CCD_Yuzhi)
		{	
		  Right=j;
		  break;	
		}
  }
	CCD_Zhongzhi=(Right+Left)/2;//¼ÆËãÖĞÏßÎ»ÖÃ
	if(myabs(CCD_Zhongzhi-Last_CCD_Zhongzhi)>90)   //¼ÆËãÖĞÏßµÄÆ«²î£¬Èç¹ûÌ«´ó
	CCD_Zhongzhi=Last_CCD_Zhongzhi;    //ÔòÈ¡ÉÏÒ»´ÎµÄÖµ
	Last_CCD_Zhongzhi=CCD_Zhongzhi;  //±£´æÉÏÒ»´ÎµÄÆ«²î
}
