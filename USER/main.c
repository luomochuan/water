#include "sys.h"
#include "delay.h"
#include "usart.h" 
#include "led.h" 		 	 
#include "lcd.h"  
#include "key.h"     
#include "usmart.h" 
#include "malloc.h"
#include "sdio_sdcard.h"  
#include "w25qxx.h"    
#include "ff.h"  
#include "exfuns.h"   
#include "text.h"
#include "piclib.h"	
#include "string.h"		
#include "beep.h" 
#include "timer.h" 
#include "ov2640.h"
#include "dma.h"
#include "data_processing.h"		//该头文件为数据处理头文件，用于处理各个传感器采集的数据

//ALIENTEK STM32F103开发板 扩展实验6
//OV2640照相机 实验 
//技术支持：www.openedv.com
//广州市星翼电子科技有限公司

#define OV2640_JPEG_WIDTH	240	//JPEG拍照的宽度	
#define OV2640_JPEG_HEIGHT	320		//JPEG拍照的高度

u8* ov2640_framebuf;				//帧缓存
extern u8 ov_frame;					//在timer.c里面定义	   
 
//文件名自增（避免覆盖）
//mode:0,创建.bmp文件;1,创建.jpg文件.
//bmp组合成:形如"0:PHOTO/PIC13141.bmp"的文件名
//jpg组合成:形如"0:PHOTO/PIC13141.jpg"的文件名
void camera_new_pathname(u8 *pname,u8 mode)
{	 
	u8 res;					 
	u16 index=0;
	while(index<0XFFFF)
	{
		if(mode==0)sprintf((char*)pname,"0:PHOTO/PIC%05d.bmp",index);
		else sprintf((char*)pname,"0:PHOTO/PIC%05d.jpg",index);
		res=f_open(ftemp,(const TCHAR*)pname,FA_READ);//尝试打开这个文件
		if(res==FR_NO_FILE)break;		//该文件名不存在=正是我们需要的.
		index++;
	}
}
//OV2640速度控制
//根据LCD分辨率的不同，设置不同的参数
void ov2640_speed_ctrl(void)
{
	u8 clkdiv,pclkdiv;			//时钟分频系数和PCLK分频系数
	if(lcddev.width==240)		//2.8寸LCD
	{
		clkdiv=1;
		pclkdiv=28;
	}else if(lcddev.width==320)	//3.5寸LCD
	{
		clkdiv=3;
		pclkdiv=15;
	}else						//4.3/7寸LCD
	{
		clkdiv=15;
		pclkdiv=4;
	} 
	SCCB_WR_Reg(0XFF,0X00);		
	SCCB_WR_Reg(0XD3,pclkdiv);	//设置PCLK分频
	SCCB_WR_Reg(0XFF,0X01);
	SCCB_WR_Reg(0X11,clkdiv);	//设置CLK分频	
}

#define jpeg_buf_size 20*1024  			//定义JPEG数据缓存jpeg_buf的大小
u8 jpeg_buf[jpeg_buf_size];	//JPEG数据缓存buf

//JPEG尺寸支持列表
const u16 jpeg_img_size_tbl[][2]=
{
	176,144,	//QCIF： 0		
	160,120,	//QQVGA：1
	352,288,	//CIF：  2
	320,240,	//QVGA：	3
	640,480,	//VGA：	4
//	800,600,	//SVGA
//	1024,768,	//XGA
//	1280,1024,	//SXGA
//	1600,1200,	//UXGA
};

const u8*EFFECTS_TBL[7]={"Normal","Negative","B&W","Redish","Greenish","Bluish","Antique"};	//7种特效 
const u8*JPEG_SIZE_TBL[9]={"QCIF","QQVGA","CIF","QVGA","VGA","SVGA","XGA","SXGA","UXGA"};	//JPEG图片 9种尺寸 

//OV2640拍照jpg图片
//pname:要保存的jpg照片路径+名字
//返回值:0,成功
//    其他,错误代码
u8 ov2640_jpg_photo(u8 *pname)
{
	FIL* f_jpg;
	u8 res=0;
	u32 bwr;
	u32 i=0;
	u32 jpeglen=0;
	u8* pbuf;
	f_jpg=(FIL *)mymalloc(SRAMIN,sizeof(FIL));	//开辟FIL字节的内存区域 
	if(f_jpg==NULL)return 0XFF;					//内存申请失败.
	
	OV2640_JPEG_Mode();							//切换为JPEG模式 
  	OV2640_OutSize_Set(OV2640_JPEG_WIDTH,OV2640_JPEG_HEIGHT); 
	SCCB_WR_Reg(0XFF,0X00);
	SCCB_WR_Reg(0XD3,30);
	SCCB_WR_Reg(0XFF,0X01);
	SCCB_WR_Reg(0X11,0X1); 
	for(i=0;i<10;i++)		//丢弃10帧，等待OV2640自动调节好（曝光白平衡之类的）
	{
		while(OV2640_VSYNC==1);	 
		while(OV2640_VSYNC==0);	  
	}  
	while(OV2640_VSYNC==1)	//开始采集jpeg数据
	{
		while(OV2640_HREF)
		{  
			while(OV2640_PCLK==0); 
			ov2640_framebuf[jpeglen]=OV2640_DATA;
			while(OV2640_PCLK==1);  
			jpeglen++;
		} 
	}
	
	res=f_open(f_jpg,(const TCHAR*)pname,FA_WRITE|FA_CREATE_NEW);//模式0,或者尝试打开失败,则创建新文件	 
	if(res==0)
	{
		printf("jpeg data size:%d\r\n",jpeglen);	//串口打印JPEG文件大小
		pbuf=(u8*)ov2640_framebuf;
		for(i=0;i<jpeglen;i++)//查找0XFF,0XD8
		{
			if((pbuf[i]==0XFF)&&(pbuf[i+1]==0XD8))break;
		}
		if(i==jpeglen)res=0XFD;//没找到0XFF,0XD8
		else		//找到了
		{
			pbuf+=i;//偏移到0XFF,0XD8处
			res=f_write(f_jpg,pbuf,jpeglen-i,&bwr);
			if(bwr!=(jpeglen-i))res=0XFE; 
		}
	} 
	f_close(f_jpg);  
	OV2640_RGB565_Mode();	//RGB565模式 
	myfree(SRAMIN,f_jpg); 
	return res;
}  

//将ov2640摄像头采集到的数据通过串口1发送给上位机。
//函数返回值设置成一个数字用于调节上传照片尺寸,size自增到5后变为0，对应上面定义的二维数组
int ov2640_sendphoto(u8 size){		

	u8 *pbuf;
	u8 key=0; 
	u8 effect=0,saturation=2,contrast=2;
	
	u8 msgbuf[15];	//消息缓存区
	uint32_t jpeglen=0,i=0,headok=0,jpgstart=0,jpglen=0;	
	
	OV2640_JPEG_Mode();							//切换为JPEG模式 
  	OV2640_OutSize_Set(jpeg_img_size_tbl[size][0],jpeg_img_size_tbl[size][1]); 
	SCCB_WR_Reg(0XFF,0X00);
	SCCB_WR_Reg(0XD3,30);
	SCCB_WR_Reg(0XFF,0X01);
	SCCB_WR_Reg(0X11,0X1); 
	for(i=0;i<10;i++)		//丢弃10帧，等待OV2640自动调节好（曝光白平衡之类的）
	{
		while(OV2640_VSYNC==1);	 
		while(OV2640_VSYNC==0);	  
	}  
	while(OV2640_VSYNC==1)	//开始采集jpeg数据
	{
		while(OV2640_HREF)
		{  
			while(OV2640_PCLK==0); 
			ov2640_framebuf[jpeglen]=OV2640_DATA;
			while(OV2640_PCLK==1);  
			jpeglen++;
		} 
	}
	
		pbuf=(u8*)ov2640_framebuf;
		for(i=0;i<jpeglen;i++)//查找0XFF,0XD8
		{
			if((pbuf[i]==0XFF)&&(pbuf[i+1]==0XD8))//找到FF D8
			{
				jpgstart=i;
				headok=1;	//标记找到jpg头(FF D8)
			}
			if((pbuf[i]==0XFF)&&(pbuf[i+1]==0XD9)&&headok)//找到头以后,再找FF D9
			{
				jpglen=i-jpgstart+2;
				LED1=!LED1;
				break;
			}
		}
		LCD_ShowString(30,210,210,16,16,"Send data will be come!!");	//调试显示
		if(jpglen)	//正常的jpeg数据 
		{
			
			pbuf+=jpgstart;			//偏移到0XFF,0XD8处 
			for(i=0;i<jpglen;i++)	//发送整个jpg文件
			{
				while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);	//循环发送,直到发送完毕  		
				USART_SendData(USART1,pbuf[i]);
				key=KEY_Scan(0); 
				if(key)break;
			}
			if(key==KEY0_PRES)	//有按键按下,需要处理
			{   
				size++;  
				if(size>=5)size=0;   
				OV2640_OutSize_Set(jpeg_img_size_tbl[size][0],jpeg_img_size_tbl[size][1]);//设置输出尺寸  
				sprintf((char*)msgbuf,"JPEG Size:%s",JPEG_SIZE_TBL[size]);
						
				LCD_ShowString(30,180,210,16,16,msgbuf);//显示提示内容
				delay_ms(800); 				  
			}else LCD_ShowString(30,210,210,16,16,"Send data complete!!");//提示传输结束设置			
			
		}else
			LED1=!LED1;	
		return size;
}

 int main(void)
 {	 
	u8 res;							 
	u8 *pname;					//带路径的文件名 
	u8 key;						//键值		   
	u8 sd_ok=1;					//0,sd卡不正常;1,SD卡正常.
 	u16 pixcnt=0;				//像素统计
	u16 linecnt=0;				//行数统计
	u8 size=4;					//用于设置上传照片尺寸
	 
	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	uart_init(921600);	 	//串口初始化为961200, 			PA9,PA10
	usmart_dev.init(72);		//初始化USMART		
  	LED_Init();		  			//初始化与LED连接的硬件接口 PB5
	KEY_Init();					//初始化按键					PA0 PE2,3,4
	LCD_Init();			   		//初始化LCD  
	BEEP_Init();        		//蜂鸣器初始化	 			PB8
	W25QXX_Init();				//初始化W25Q128				PB12
 	my_mem_init(SRAMIN);		//初始化内部内存池
	exfuns_init();				//为fatfs相关变量申请内存  
 	f_mount(fs[0],"0:",1); 		//挂载SD卡 
 	f_mount(fs[1],"1:",1); 		//挂载FLASH. 
	 
	//传感器部分代码
	//GPIO_Configuration();
    /* 配置USART1 */
    //USART1_Config();
	// ADC 初始化
	  ADCx_Init();  											//PA1, PA2, PA4
    /* 初始化显示屏 */
	OLED_Init();  												//PB6, PB7
	OLED_ShowString(1,1,"PH :");  
	OLED_ShowString(2, 1, "TDS:");
	OLED_ShowString(3, 1, "EC :");
	OLED_ShowString(4, 1, "T  : ");
	POINT_COLOR=RED;      
 	while(font_init()) 			//检查字库
	{	    
		LCD_ShowString(30,50,200,16,16,"Font Error!");
		delay_ms(200);				  
		LCD_Fill(30,50,240,66,WHITE);//清除显示	     
	}  	 
 	Show_Str(30,50,200,16,"STM32F103 开发板",16,0);				    	 
	Show_Str(30,70,200,16,"OV2640照相机实验",16,0);			    	 
	Show_Str(30,90,200,16,"KEY0:发送图片尺寸",16,0);			    	 
	Show_Str(30,110,200,16,"KEY1:拍照(jpg格式)",16,0);	 	
	Show_Str(30,130,200,16,"2015年4月16日",16,0);
	res=f_mkdir("0:/PHOTO");		//创建PHOTO文件夹
	if(res!=FR_EXIST&&res!=FR_OK) 	//发生了错误
	{		    
		Show_Str(30,150,240,16,"SD卡错误,无法拍照!",16,0); 	 		  	     
		sd_ok=0;  	
	} 
	ov2640_framebuf=mymalloc(SRAMIN,52*1024);//申请帧缓存
 	pname=mymalloc(SRAMIN,30);		//为带路径的文件名分配30个字节的内存		    
 	while(!pname||!ov2640_framebuf)	//内存分配出错
 	{	    
		Show_Str(30,150,240,16,"内存分配失败!",16,0);
		delay_ms(200);				  
		LCD_Fill(30,150,240,146,WHITE);//清除显示	     
		delay_ms(200);				  
	}   											  
	while(OV2640_Init())			//初始化OV2640
	{
		Show_Str(30,150,240,16,"OV2640 错误!",16,0);
		delay_ms(200);
	    LCD_Fill(30,150,239,206,WHITE);
		delay_ms(200);
	}
 	Show_Str(30,170,200,16,"OV2640 正常",16,0);
	delay_ms(1500);	 		 
	//TIM6_Int_Init(10000,7199);	//10Khz计数频率,1秒钟中断,屏蔽则不打印帧率									  
	OV2640_RGB565_Mode();			//RGB565模式
  	OV2640_OutSize_Set(lcddev.width,lcddev.height); 
	ov2640_speed_ctrl();
	MYDMA_SRAMLCD_Init((u32)ov2640_framebuf);	 
 	while(1)
	{
		while(OV2640_VSYNC)			//等待帧信号
		{
			key=KEY_Scan(0);		//不支持连按
			LCD_ShowString(30,210,210,16,16,"OK!!");//提示传输结束设置
			delay_ms(2000);
			if(key==KEY1_PRES)
			{ 	
				if(sd_ok)			//SD卡正常才可以拍照 
				{ 
						camera_new_pathname(pname,1);			//得到文件名	
 						res=ov2640_jpg_photo(pname); 
						OV2640_OutSize_Set(lcddev.width,lcddev.height); 	
						ov2640_speed_ctrl(); 
 					if(res)//拍照有误
					{
						Show_Str(30,130,240,16,"写入文件错误!",16,0);		 
					}else 
					{
						Show_Str(30,130,240,16,"拍照成功!",16,0);
						Show_Str(30,150,240,16,"保存为:",16,0);
						Show_Str(30+42,150,240,16,pname,16,0);		    
						BEEP=1;	//蜂鸣器短叫，提示拍照完成
						delay_ms(100);
					} 
				}else 					//提示SD卡错误
				{					    
					Show_Str(40,130,240,12,"SD卡错误!",12,0);
					Show_Str(40,150,240,12,"拍照功能不可用!",12,0);			    
				}
				BEEP=0;				//关闭蜂鸣器 
				delay_ms(1800);
			} 
			
		size=ov2640_sendphoto(size);	//串口发送图片
		delay_ms(1000); 
		//pH函数		
		PH_Value_Conversion();	
		Display_Data();
		//TDS
		TDS_Value_Conversion();	
		//温度
		TEMP_Value_Conversion();
		//电导率
		EC_Value_Conversion();
		
		printf_Data();		//串口一发送实时数据
		delay1_ms(1000);
		}
		
		//下面注释的在正点原子给的代码中有用，貌似是将ov2640采集到的数据
		//显示在tftlcd屏幕上，咱们项目没用屏幕，所以删了之后代码整体运行没有问题。
		
//		LCD_SetCursor(0,0);			//设置坐标
//		LCD_WriteRAM_Prepare();		//开始写入GRAM	 
// 		linecnt=0;					//行统计清零
//		pixcnt=0;					//像素计数器清零
//		while(linecnt>lcddev.height)	
//		{
//			while(OV2640_HREF)
//			{  
//				while(OV2640_PCLK==0);
//				ov2640_framebuf[pixcnt++]=OV2640_DATA;   
//				while(OV2640_PCLK==1); 
//				while(OV2640_PCLK==0); 
//				ov2640_framebuf[pixcnt++]=OV2640_DATA; 
//				while(OV2640_PCLK==1);		
//			}  
//			if(pixcnt)
//			{
//				MYDMA_SRAMLCD_Enable();	//启动DMA数据传输
//				pixcnt=0;
//				linecnt++;
//			}
//		} 
		ov_frame++;
		LED0=!LED0;

	} 	
}

















