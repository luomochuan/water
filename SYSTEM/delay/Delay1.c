/****************************************Copyright (c)****************************************************
**                                        
**                                 
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			
** Last modified Date:          
** Last Version:		   
** Descriptions:							
**--------------------------------------------------------------------------------------------------------
** Created by:			FiYu
** Created date:		2015-3-1
** Version:			    1.0
** Descriptions:		GPIO����LEDʵ��					
**--------------------------------------------------------------------------------------------------------
** Modified by:			FiYu
** Modified date:		
** Version:				
** Descriptions:		
** Rechecked by:			
**********************************************************************************************************/
#include "delay.h"


//#define USE_TIMEMODE    1   //ʹ�ö�ʱ����ʽ������ʱ(ʹ�ö�ʱ����ʱʱ�����˺궨��)

#ifdef USE_TIMEMODE


#else


/**************************************************************************************
 * ��  �� : ������ʱ
 * ��  �� : dly:��ʱ������ֵԽ����ʱʱ��Խ��
 * ����ֵ : ��
 **************************************************************************************/
void delay1_us(u32 us)
{
  u16 i;
	
	do
  {
    i = 6;
		while(i--)__nop();
  } while (--us);
}
/**************************************************************************************
 * ��  �� : ������ʱ
 * ��  �� : dly:��ʱ������ֵԽ����ʱʱ��Խ��
 * ����ֵ : ��
 **************************************************************************************/
void delay1_ms(u16 ms)
{
  do
  {
    delay_us(250);
    delay_us(250);
    delay_us(250);
    delay_us(250);
  } while (--ms);
}
#endif
/*********************************END FILE********************************************/