#include "usartx.h"
#include "usart.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
u8 Usart3_Receive;
u16 USART5_RX_STA=0;       //接收状态标记	  
s8 USART5_TX_BUF[USART5_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
s8 USART5_RX_BUF[USART5_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
/**************************实现函数**********************************************
*功    能:		usart发送一个字节
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
函数功能：串口3初始化
入口参数： bound:波特率
返回  值：无
**************************************************************************/
void uart3_init(u32 bound)
{  	 
	  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);// 需要使能AFIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	//使能GPIO时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//使能USART时钟
		GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
	//USART_TX  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //Pc10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOC, &GPIO_InitStructure);
   
  //USART_RX	  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//Pc11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  //UsartNVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
   //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure);     //初始化串口3
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART3, ENABLE);                    //使能串口3 
}

/**************************************************************************
函数功能：串口3发送中断
入口参数：无
返回  值：无
**************************************************************************/
//void USART3_IRQHandler(void)
//{	
//	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //接收到数据
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
函数功能：串口5初始化
入口参数： bound:波特率
返回  值：无
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
void UART5_IRQHandler(void) //串口5中断处理
{
	s8 Res,start=1;
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(UART5);//读取接收到的数据
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
		if(USART5_RX_STA>(USART5_REC_LEN-1))USART5_RX_STA=0;//接收数据错误,重新开始接收	
		}  			
	}
} 