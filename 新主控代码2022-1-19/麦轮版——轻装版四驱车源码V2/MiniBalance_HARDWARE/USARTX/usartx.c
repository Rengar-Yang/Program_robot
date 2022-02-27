#include "usartx.h"
#include "usart.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
u8 Usart3_Receive;
u16 USART5_RX_STA=0;       //����״̬���	  
s8 USART5_TX_BUF[USART5_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
s8 USART5_RX_BUF[USART5_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
/**************************ʵ�ֺ���**********************************************
*��    ��:		usart����һ���ֽ�
*********************************************************************************/
void usart3_send(u8 data)
{
	USART3->DR = data;
	while((USART3->SR&0x40)==0);	
}

void usart5_send(u8 Data)
{
	USART_SendData(UART5,Data);
	while( USART_GetFlagStatus(UART5, USART_FLAG_TC) == RESET );
}

/**************************************************************************
�������ܣ�����3��ʼ��
��ڲ����� bound:������
����  ֵ����
**************************************************************************/
void uart3_init(u32 bound)
{  	 
	  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);// ��Ҫʹ��AFIOʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	//ʹ��GPIOʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//ʹ��USARTʱ��
		GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
	//USART_TX  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //Pc10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOC, &GPIO_InitStructure);
   
  //USART_RX	  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//Pc11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  //UsartNVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
   //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART3, &USART_InitStructure);     //��ʼ������3
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ���3 
}

/**************************************************************************
�������ܣ�����3�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
//void USART3_IRQHandler(void)
//{	
//	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //���յ�����
//	{	  
//				u8 temp;
//				static u8 count,last_data,last_last_data,Usart_ON_Count;
//		    if(Usart_ON_Flag==0)
//				{	
//		    if(++Usart_ON_Count>10)Usart_ON_Flag=1;
//				}
//				temp=USART3->DR;
//				 if(Usart_Flag==0)
//					{	
//					if(last_data==0xfe&&last_last_data==0xff) 
//					Usart_Flag=1,count=0;	
//					}
//				 if(Usart_Flag==1)
//					{	
//						Urxbuf[count]=temp;     
//						count++;                
//						if(count==8)Usart_Flag=0;
//					}
//					last_last_data=last_data;
//					last_data=temp;
//	}  											 
//} 


/**************************************************************************
�������ܣ�����5��ʼ��
��ڲ����� bound:������
����  ֵ����
**************************************************************************/
void uart5_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;        
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD, ENABLE );
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE );
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;         //UART5 TX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    //??????;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);             //??C;
	    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;              //UART5 RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //????;
	GPIO_Init(GPIOD, &GPIO_InitStructure);                 //??D;
	
	USART_InitStructure.USART_BaudRate = bound;                  //???;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //???8?;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;      //???1?;
	USART_InitStructure.USART_Parity = USART_Parity_No ;        //????;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //?????;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                  //????;
	USART_Init(UART5, &USART_InitStructure);                                         //??????;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //?????,4??????,4??????;
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn; //???;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //?????;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //?????;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	USART_Cmd(UART5, ENABLE); //????;
}
void UART5_IRQHandler(void) //����5�жϴ���
{
	s8 Res,start=1;
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(UART5);//��ȡ���յ�������
		USART5_RX_BUF[USART5_RX_STA]=Res ;
		if(USART5_RX_BUF[0]!=-91)
		{
			start=0;
			USART5_RX_STA=0;
		}
		else
			start=1;
		if(start==1)
		{		
		USART5_RX_STA++;
		if(USART5_RX_STA>(USART5_REC_LEN-1))USART5_RX_STA=0;//�������ݴ���,���¿�ʼ����	
		}  			
	}
} 