#include "Auto.h"

void GetToQRcode()//ǰ����ά��ص�
{
	Target_X=0;
	Target_Y=1;
}

void GetToMateralStorage()//ǰ��ԭ����
{
	Target_X=0;
	Target_Y=2;
}

void GetToRoughProduct()//ǰ���ּӹ���
{
	Target_X=1;
	Target_Y=4;
}

void GetToSemifinishedProduct()//ǰ�����Ʒ��
{
	Target_X=4;
	Target_Y=2;
}

void GetToDestination()//ǰ���յ�
{
	Target_X=4;
	Target_Y=0;
}

void PostionInit()//���⴫������ʼ��
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //ʹ�ܶ˿�ʱ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1; //�˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //��������
  GPIO_Init(GPIOB, &GPIO_InitStructure);					      //�����趨������ʼ��GPIO
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //ʹ�ܶ˿�ʱ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5; //�˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);					      //�����趨������ʼ��GPIO
}

void PostionDetect()//���������
{
	s8 key_x,key_y;
	
	if(front==1&&back==1){key_x=0;}
	if(right==1&&left==1){key_y=0;}
	
	if(front==0&&back==0&&Move_X>0&&key_x==0){X+=1;key_x=1;}
	else if(front==0&&back==0&&Move_X<0&&key_x==0){X-=1;key_x=1;}
	if(right==0&&left==0&&Move_Y<0&&key_y==0){Y-=1;key_y=1;}
	else if(right==0&&left==0&&Move_Y>0&&key_y==0){Y+=1;key_y=1;}
	
	
}