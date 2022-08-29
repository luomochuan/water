//传感器部分头文件
#include "stm32f10x.h"
#include "bsp_usart1.h"
#include <string.h>
#include "Delay1.h"
#include "math.h"
#include "OLED.h"
#include "ds18b20.h"
#include "bsp_adc.h"

//传感器模块代码

#define RES2 820.0   //运放电阻，与硬件电路有关
#define ECREF 200.0 //极片输入电压，与硬件电路相关


GPIO_InitTypeDef  GPIO_InitStructure; 
unsigned char AD_CHANNEL=0;
unsigned long PH_num=0,PU_V=0;
float PH_Value=0;
u8 ph_temp=0,tu_temp=0;
u16 ph_result=0,tu_result=0;
u16 adc_1,adc_2;
u16 adc_v_1,adc_v_2;

//TDS
float TDS=0.0,TDS_voltage;
float TDS_value=0.0,voltage_value;
float temp_data=0.0;
float compensationCoefficient=1.0;//温度校准系数
float compensationVolatge;
float kValue=1.0;

u8 ISendByte(unsigned char sla,unsigned char c);
u8 IRcvByte(unsigned char sla);	
u8 SPIx_ReadWriteByte(u8 TxData);

unsigned char  Tx[20];   //无线发送缓存

//电导率参数
float EC_voltage;
float EC_value=0.0,voltage_value;
float compensationVolatge;
float kValue_Low=0.5;  //校准时进行修改
float kValue_High=1.0; //校准时进行修改
float rawEC=0.0;
float EC_valueTemp=0.0;


// ADC1转换的电压值通过MDA方式传到SRAM
extern uint16_t AD_Value[3];

// 局部变量，用于保存转换计算后的电压值 	 
float ADC_ConvertedValueLocal;  

/***************************************************************************
 * 描  述 : MAIN函数
 * 入  参 : 无
 * 返回值 : 无
 **************************************************************************/
 
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* Enable the GPIO  Clock */					 		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC| RCC_APB2Periph_AFIO|RCC_APB2Periph_SPI1,ENABLE);
	
	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);		//屏蔽所有作为JTAG口的GPIO口
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);		//屏蔽PB口上IO口JTAG功能
}

void PH_Value_Conversion()
{
	
	  ADC_ConvertedValueLocal =(float) AD_Value[0]/4096*3.3; // 读取转换的AD值
	
		PH_Value=-5.7541*ADC_ConvertedValueLocal+16.654;

	if(PH_Value<=0.0){PH_Value=0.0;}
	if(PH_Value>=14.0){PH_Value=14.0;}
	
		  /*显示电压*/
	Tx[0]=(int)(PH_Value*100)/1000+'0';	
	Tx[1]=(int)(PH_Value*100)%1000/100+'0';
	Tx[2]='.';
	Tx[3]=(int)(PH_Value*100)%100/10+'0';
	Tx[4]=(int)(PH_Value*100)%10+'0';
}

void Display_Data()
{
		//显示PH值	
		OLED_ShowChar(1,4,Tx[0]);
		OLED_ShowChar(1,5,Tx[1]);
		OLED_ShowChar(1,6,Tx[2]);
		OLED_ShowChar(1,7,Tx[3]);
		OLED_ShowChar(1,8,Tx[4]);
}

/**************TDS值采集函数***************/

void TDS_Value_Conversion()
{
	TDS_voltage =(float) AD_Value[1]/4096*3.3; // 读取转换的AD值
	
	compensationCoefficient=1.0+0.02*((temp_data/10)-25.0); 
	compensationVolatge=TDS_voltage/compensationCoefficient;
	TDS_value=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 
	255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5*kValue;
	
	  if((TDS_value<=0)){TDS_value=0;}
		if((TDS_value>1400)){TDS_value=1400;}
		
		/*显示TDS*/

		Tx[0]=(int)(TDS_value)/1000+'0';
		Tx[1]=(int)(TDS_value)%1000/100+'0';	
		Tx[2]=(int)(TDS_value)%100/10+'0';
		Tx[3]=(int)(TDS_value)%10+'0';
		
		OLED_ShowChar(2,5,Tx[0]);
		OLED_ShowChar(2,6,Tx[1]);
		OLED_ShowChar(2,7,Tx[2]);
		OLED_ShowChar(2,8,Tx[3]);
		
}

//电导率函数
void EC_Value_Conversion()
{
	EC_voltage =(float) AD_Value[2]/4096*3300; // 读取转换的AD值
	
	rawEC = 1000*EC_voltage/RES2/ECREF;
	EC_valueTemp=rawEC*kValue;
	
	/*First Range:(0,2); Second Range:(2,20)*/
	if(EC_valueTemp>2.0)
	{
	  kValue=kValue_High;
	}
	else if(EC_valueTemp<=2.0)
	{
	  kValue=kValue_Low;
	}
	EC_value=rawEC*kValue;
	
	compensationCoefficient=1.0+0.0185*((temp_data/10)-25.0); 
	
	EC_value=EC_value/compensationCoefficient;
	
		if((EC_value<=0)){EC_value=0;}
		if((EC_value>20)){EC_value=20;}//20mS/cm
		
		/*显示EC*/
		Tx[0]=(int)(EC_value*100)/1000+'0';
		Tx[1]=(int)(EC_value*100)%1000/100+'0';
		Tx[2]='.';
		Tx[3]=(int)(EC_value)%100/10+'0';	
		Tx[4]=(int)(EC_value)%10+'0';
		
		OLED_ShowChar(3,4,Tx[0]);
		OLED_ShowChar(3,5,Tx[1]);
		OLED_ShowChar(3,6,Tx[2]);
		OLED_ShowChar(3,7,Tx[3]);
		OLED_ShowChar(3,8,Tx[4]);
		OLED_ShowString(3,9,"mS/cm");
}

void TEMP_Value_Conversion()
{
	  temp_data=DS18B20_Get_Temp();
	
	  Tx[4]=(int)(temp_data)%1000/100+'0';	
	  Tx[5]=(int)(temp_data)%100/10+'0';
	  Tx[6]='.';
	  Tx[7]=(int)(temp_data)%10+'0';
		OLED_ShowChar(4,5,Tx[4]);
		OLED_ShowChar(4,6,Tx[5]);
		OLED_ShowChar(4,7,Tx[6]);
		OLED_ShowChar(4,8,Tx[7]);
		OLED_ShowString(4,9,"C");
}