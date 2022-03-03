#include "Auto.h"

void GetToQRcode()//前往二维码地点
{
	Target_X=0;
	Target_Y=1;
}

void GetToMateralStorage()//前往原料区
{
	Target_X=0;
	Target_Y=2;
}

void GetToRoughProduct()//前往粗加工区
{
	Target_X=1;
	Target_Y=4;
}

void GetToSemifinishedProduct()//前往半成品区
{
	Target_X=4;
	Target_Y=2;
}

void GetToDestination()//前往终点
{
	Target_X=4;
	Target_Y=0;
}

void PostionInit()//红外传感器初始化
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能端口时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1; //端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //上拉输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);					      //根据设定参数初始化GPIO
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能端口时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5; //端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //上拉输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);					      //根据设定参数初始化GPIO
}

void PostionDetect()//计算大坐标
{
	s8 key_x,key_y;
	
	if(front==1&&back==1){key_x=0;}
	if(right==1&&left==1){key_y=0;}
	
	if(front==0&&back==0&&Move_X>0&&key_x==0){X+=1;key_x=1;}
	else if(front==0&&back==0&&Move_X<0&&key_x==0){X-=1;key_x=1;}
	if(right==0&&left==0&&Move_Y<0&&key_y==0){Y-=1;key_y=1;}
	else if(right==0&&left==0&&Move_Y>0&&key_y==0){Y+=1;key_y=1;}
	
	
}