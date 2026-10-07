#include "sys.h"
#include "usart.h"	
#include "stm32f10x.h"
#include "LED.h"
#include "delay.h"
#include "stdarg.h"	 	 
#include "stdio.h"	 	 
#include "string.h"

//**********************************************************************
/*USART表示可同步异步通信
 而UART只可进行异步通信
 串口1，被CH340占用，不可接收，但金装版（MINI口为金色）stm32可正常接收发
 */




//////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif

//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 


   
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA = 0;       //接收状态标记	  
u16 USART2_RX_STA = 0; 
u16 USART3_RX_STA = 0; 
u16 UART4_RX_STA = 0; 
u16 UART5_RX_STA = 0; 

u8 USART2_RX_BUF[USART2_REC_LEN]; 
u8 USART3_RX_BUF[USART3_REC_LEN]; 
u8 UART4_RX_BUF[UART4_REC_LEN]; 
u8 UART5_RX_BUF[UART5_REC_LEN]; 

float p1 = 0.0f;  // P2参数值
float i1 = 0.0f;  // I2参数值
float d1 = 0.0f;  
float p2 = 0.0f;  // P2参数值
float i2 = 0.0f;  // I2参数值
float d2 = 0.0f;  
// 	u16 t;  
	u16 len;	
int data[5];
int x;


void usart_init(u32 bound) {
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);    //使能USART1，GPIOA时钟

    //USART1_TX   GPIOA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    //复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9

    //USART1_RX	  GPIOA.10初始化
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10

    //Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);    //根据指定的参数初始化VIC寄存器

    //USART 初始化设置

    USART_InitStructure.USART_BaudRate = bound;//串口波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    //收发模式

    USART_Init(USART1, &USART_InitStructure); //初始化串口1
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
    USART_Cmd(USART1, ENABLE);                    //使能串口1

}

void usart2_init(u32 bound)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);	//使能GPIOA时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); //使能USART2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	
	USART_DeInit(USART2);
	
	
	//USART2_TX   GPIOA.2
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //PA.2
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOD,&GPIO_InitStructure);//初始化GPIOA.2
   
  //USART2_RX	  GPIOA.3
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;//PA.3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOD,&GPIO_InitStructure);//初始化GPIOA.3  

  //Usart2 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART2,&USART_InitStructure); //初始化串口2
  USART_ITConfig(USART2,USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART2,ENABLE);                    //使能串口2
	GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
}
//串口3 PB10 PB11 ,APB1时钟
void usart3_init(u32 bound) {
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);    //使能USART3，GPIOB时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);    //使能USART3，GPIOB时钟

    //USART3_TX   GPIOB.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    //复用推挽输出
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB.10

    //USART3_RX	  GPIOB.11初始化
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PB11
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB.11

    //Usart3 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);    //根据指定的参数初始化VIC寄存器

    //USART 初始化设置
    USART_InitStructure.USART_BaudRate = bound;//串口波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    //收发模式

    USART_Init(USART3, &USART_InitStructure); //初始化串口3
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
    USART_Cmd(USART3, ENABLE);                    //使能串口3
}





//串口4 PC10 PC11 ,APB1时钟
void uart4_init(u32 bound) {
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);    //使能UART4，GPIOC时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);    //使能UART4，GPIOC时钟

    //UART4_TX   GPIOC.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PC.10
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    //复用推挽输出
    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIOC.10

    //UART4_RX	  GPIOC.11初始化
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PC11
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIOC.11

    //Usart4 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);    //根据指定的参数初始化VIC寄存器

    //USART 初始化设置
    USART_InitStructure.USART_BaudRate = bound;//串口波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    //收发模式

    USART_Init(UART4, &USART_InitStructure); //初始化串口4
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启串口接受中断
    USART_Cmd(UART4, ENABLE);                    //使能串口4
}
//串口5 PC12 PD2 ,APB1时钟
void uart5_init(u32 bound) {
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);    //使能UART4，GPIOC时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOC, ENABLE);    //使能UART4，GPIOC时钟

    //UART5_TX   GPIOC.12
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; //PD.12
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    //复用推挽输出
    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIOC.10

    //UART5_RX	  GPIOD.2初始化
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;//PD2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化GPIOC.11

    //Usart4 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);    //根据指定的参数初始化VIC寄存器

    //USART 初始化设置
    USART_InitStructure.USART_BaudRate = bound;//串口波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    //收发模式

    USART_Init(UART5, &USART_InitStructure); //初始化串口4
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//开启串口接受中断
    USART_Cmd(UART5, ENABLE);                    //使能串口4
}

void USART1_IRQHandler(void)                	//串口1中断服务程序
	{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
		Res =USART_ReceiveData(USART1);	//读取接收到的数据
		
		if((USART_RX_STA&0x8000)==0)//接收未完成
			{
			if(USART_RX_STA&0x4000)//接收到了0x0d
				{
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
				}
			else //还没收到0X0D
				{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
					{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
					}		 
				}
			}   		 
     } 
#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntExit();  											 
#endif
} 
void USART2_IRQHandler(void)                	//串口2中断服务程序
	{
	u8 Res;

#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
		Res =USART_ReceiveData(USART2);	//读取接收到的数据
	
				//	LED0=!LED0;
		if((USART2_RX_STA&0x8000)==0)//接收未完成
			{
			if(USART2_RX_STA&0x4000)//接收到了0x0d
				{
				if(Res!=0x0a)USART2_RX_STA=0;//接收错误,重新开始
				else USART2_RX_STA|=0x8000;	//接收完成了 
				}
			else //还没收到0X0D
				{	
				if(Res==0x0d)USART2_RX_STA|=0x4000;
				else
					{
					USART2_RX_BUF[USART2_RX_STA&0X3FFF]=Res ;
					USART2_RX_STA++;
					if(USART2_RX_STA>(USART2_REC_LEN-1))USART2_RX_STA=0;//接收数据错误,重新开始接收	  
					}		 
				}
			}   

//if(USART2_RX_STA&0x8000) {
//    u2_printf("检测到接收完成标志\r\n");
//    len = USART2_RX_STA & 0X7FFF;
//    USART2_RX_BUF[len] = 0;
//  
//        
//       if (strncmp((char*)USART2_RX_BUF, "P1=", 3) == 0) {
//    sscanf((char*)USART2_RX_BUF, "P1=%f", &p1);
//    u2_printf("设置P1 = %.2f\r\n", p1);
//}
//        else if (strncmp((char*)USART2_RX_BUF, "I1=", 3) == 0) {
//    sscanf((char*)USART2_RX_BUF, "I1=%f", &i1);
//      u2_printf("设置I1 = %.2f\r\n", i1);
//}    else if (strncmp((char*)USART2_RX_BUF, "D1=", 3) == 0) {
//    sscanf((char*)USART2_RX_BUF, "D1=%f", &d1);
//      u2_printf("设置D1 = %.2f\r\n", d1);
//}       
//else if (strncmp((char*)USART2_RX_BUF, "P2=", 3) == 0) {
//    sscanf((char*)USART2_RX_BUF, "P2=%f", &p2);
//    u2_printf("设置P2 = %.2f\r\n", p2);
//}
//        else if (strncmp((char*)USART2_RX_BUF, "I2=", 3) == 0) {
//    sscanf((char*)USART2_RX_BUF, "I2=%f", &i2);
//      u2_printf("设置I2 = %.2f\r\n", i2);
//}    else if (strncmp((char*)USART2_RX_BUF, "D2=", 3) == 0) {
//    sscanf((char*)USART2_RX_BUF, "D2=%f", &d2);
//      u2_printf("设置D2 = %.2f\r\n", d2);
//}
    
    
//    memset(USART2_RX_BUF, 0, USART2_REC_LEN);  // 清空缓冲区
			USART2_RX_STA = 0;                          // 重置接收状态

			
     } 
		
	 
#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntExit();  											 
#endif
} 
//void USART3_IRQHandler(void)                	//串口2中断服务程序
//	{
//	u8 Res;
//#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
//	OSIntEnter();    
//#endif
//	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
//		{
//		Res =USART_ReceiveData(USART3);	//读取接收到的数据
//		if((USART3_RX_STA&0x8000)==0)//接收未完成
//			{
//			if(USART3_RX_STA&0x4000)//接收到了0x0d
//				{
//				if(Res!=0x0a)USART3_RX_STA=0;//接收错误,重新开始
//				else USART3_RX_STA|=0x8000;	//接收完成了 
//				}
//			else //还没收到0X0D
//				{	
//				if(Res==0x0d)USART3_RX_STA|=0x4000;
//				else
//					{
//					USART3_RX_BUF[USART3_RX_STA&0X3FFF]=Res ;
//					USART3_RX_STA++;
//					if(USART3_RX_STA>(USART3_REC_LEN-1))USART3_RX_STA=0;//接收数据错误,重新开始接收	  
//					}		 
//				}
//			}   		 
//     } 
//#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
//	OSIntExit();  											 
//#endif
//} 
void UART4_IRQHandler(void)                	//串口2中断服务程序
	{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
		Res =USART_ReceiveData(UART4);	//读取接收到的数据
		if((UART4_RX_STA&0x8000)==0)//接收未完成
			{
			if(UART4_RX_STA&0x4000)//接收到了0x0d
				{
				if(Res!=0x0a)UART4_RX_STA=0;//接收错误,重新开始
				else UART4_RX_STA|=0x8000;	//接收完成了 
				}
			else //还没收到0X0D
				{	
				if(Res==0x0d)UART4_RX_STA|=0x4000;
				else
					{
					UART4_RX_BUF[UART4_RX_STA&0X3FFF]=Res ;
					UART4_RX_STA++;
					if(UART4_RX_STA>(UART4_REC_LEN-1))UART4_RX_STA=0;//接收数据错误,重新开始接收	  
					}		 
				}
			} 
			
if(UART4_RX_STA&0x8000)   //接收完成
			{
						len=UART4_RX_STA&0X7FFF;	//得到数据长度
						USART_RX_BUF[len]=0;	 	//加入结束符
						sscanf((char*)UART4_RX_BUF,"%d",&x);
      if(x == 1) {
          data[0] = x;
			}
      else if(x == 2 || x == 3) {
          data[1] = x;
       }				
						UART4_RX_STA=0;
						//printf("(%d,%d,%d,%d,%d,%d,%d,%d,%d)\r\n",x1,x2,x3,x4,x5,x6,x7,x8,x9);//使用printf()需要将下方代码注释解除
						while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET)
						{
								clear_USART_RX_BUF();
						}//等待发送完毕
						u4_printf("%d\r\n",data[0]);
            u4_printf("%d\r\n",data[1]);							
			}				
     } 
 }
	void UART5_IRQHandler(void)                	//串口2中断服务程序
	{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
		Res =USART_ReceiveData(UART5);	//读取接收到的数据
		if((UART5_RX_STA&0x8000)==0)//接收未完成
			{
			if(UART5_RX_STA&0x4000)//接收到了0x0d
				{
				if(Res!=0x0a)UART5_RX_STA=0;//接收错误,重新开始
				else UART5_RX_STA|=0x8000;	//接收完成了 
				}
			else //还没收到0X0D
				{	
				if(Res==0x0d)UART5_RX_STA|=0x4000;
				else
					{
					UART5_RX_BUF[UART5_RX_STA&0X3FFF]=Res ;
					UART5_RX_STA++;
					if(UART5_RX_STA>(UART5_REC_LEN-1))UART5_RX_STA=0;//接收数据错误,重新开始接收	  
					}		 
				}
			}   		 
     } 
#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntExit();  											 
#endif
} 
	void u2_printf(char* fmt, ...)
{
    int i;
    int len;
    char buffer[256]; // 增大缓冲区，避免溢出
    va_list args;
    
    va_start(args, fmt);
    vsprintf(buffer, fmt, args);
    va_end(args);

    len = strlen(buffer);

    // 发送数据，不干扰接收缓冲区
    for(i = 0; i < len; i++)
    {
        // 等待发送缓冲区为空
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        
        USART_SendData(USART2, buffer[i]);
    }
}
	void u3_printf(char *fmt, ...) {
    int i;
    int len;
    char buffer[50]; //足够容纳才可以，可以搞大点
    va_list args;
    va_start(args, fmt);
    vsprintf(buffer, fmt, args);
    va_end(args);

    len = strlen(buffer);

    for (i = 0; i < len; i++) {
        while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
				{
				memset(USART3_RX_BUF, 0, i);//将USART2_RX_BUF的前i个值都置为0
				}					//等待发送完毕
        USART_SendData(USART3, buffer[i]); //发送一个字符
    }
}
	void u4_printf(char *fmt, ...) {
    int i;
    int len;
    char buffer[50]; //足够容纳才可以，可以搞大点
    va_list args;
    va_start(args, fmt);
    vsprintf(buffer, fmt, args);
    va_end(args);

    len = strlen(buffer);

    for (i = 0; i < len; i++) {
        while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET)
				{
				memset(UART4_RX_BUF, 0, i);//将USART2_RX_BUF的前i个值都置为0
				}					//等待发送完毕
        USART_SendData(UART4, buffer[i]); //发送一个字符
    }
}
void u5_printf(char *fmt, ...) {
    int i;
    int len;
    char buffer[50]; //足够容纳才可以，可以搞大点
    va_list args;
    va_start(args, fmt);
    vsprintf(buffer, fmt, args);
    va_end(args);

    len = strlen(buffer);

    for (i = 0; i < len; i++) {
        while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET)
				{
				memset(UART5_RX_BUF, 0, i);//将USART2_RX_BUF的前i个值都置为0
				}					//等待发送完毕
        USART_SendData(UART5, buffer[i]); //发送一个字符
    }
}

// vofa数据发送函数
void vofa_send(float speed1, float speed2, float output1, float output2) {
    char buffer[100];
    snprintf(buffer, sizeof(buffer), "%.2f,%.2f,%.2f,%.2f\n", 
             speed1, speed2, output1, output2);
    
    int len = strlen(buffer);
    for (int i = 0; i < len; i++) {
        while (!(USART2->SR & USART_FLAG_TXE));
        USART2->DR = buffer[i];
    }
}

void clear_USART_RX_BUF() {
    memset(USART_RX_BUF, 0, sizeof(USART_RX_BUF));
}
#endif	

