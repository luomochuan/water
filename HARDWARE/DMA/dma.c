#include "dma.h"																	   	  
#include "delay.h"																	   	  
#include "lcd.h"																	   	  
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F103������ 
//DMA���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2015/1/15
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved	
//********************************************************************************
//V1.1 20150416
//��ӣ�MYDMA_SRAMLCD_Init��MYDMA_SRAMLCD_Enable������
//////////////////////////////////////////////////////////////////////////////////

 
//SRAM --> LCD_RAM dma����
//caddr������Դ��ַ
//16λ,��SRAM���䵽LCD_RAM. 
void MYDMA_SRAMLCD_Init(u32 caddr)
{  
	DMA_InitTypeDef DMA_InitStructure;
	
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);	//ʹ��DMA����
	
  DMA_DeInit(DMA2_Channel5);   //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ

	DMA_InitStructure.DMA_PeripheralBaseAddr = caddr;  //DMA����ADC����ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&LCD->LCD_RAM;  //DMA�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //���ݴ��䷽�򣬴������
	DMA_InitStructure.DMA_BufferSize = 0;  //DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;  //�����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;  //�ڴ��ַ�Ĵ���������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //���ݿ��Ϊ16λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //���ݿ��Ϊ16λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //��������������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMAͨ�� xӵ�������ȼ� 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;  //DMAͨ��x����Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMA2_Channel5, &DMA_InitStructure);  //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��USART1_Tx_DMA_Channel����ʶ�ļĴ���

	
}  
//����һ��SRAM��LCD��DMA�Ĵ���
void MYDMA_SRAMLCD_Enable(void)
{	   
 
	DMA_Cmd(DMA2_Channel5, DISABLE ); //�ر�DMA����        
 	DMA_SetCurrDataCounter(DMA2_Channel5,lcddev.width);//DMAͨ����DMA����Ĵ�С
 	DMA_Cmd(DMA2_Channel5, ENABLE);  //ʹ��USART1 TX DMA1 ��ָʾ��ͨ�� 	
} 


 

























