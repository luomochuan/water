#ifndef __warn_H
#define __warn_H	 
#include "sys.h"

#define LED0 PBout(5)// PB5
#define LED1 PEout(5)// PE5	

void LED_Init(void);//��ʼ��
void warn(float TDS_value);

#endif
