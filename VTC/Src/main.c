/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//**************************** 全局变量定义  **************************
	int k=0,i=0,j=0,counter=0;//循环计数器，每1000次循环重新拨号
	int Reset=0,Locked=0,Stolen=0,exlocked=0;//状态变量
	const double pi=3.1415926535;//Π值
	double speed_standard = 1.45;//速度阈值,单位m/s(可调)
	char douhao=',';
	char GNGGA[6]={"GNGGA,"};//GNGGA格式GPS语句起始标识符
	uint8_t startsign_uint='$',input;//GPS语句起始标识符
	char Ending = 0x1A;//结束符，无回车换行符号
	//************************ uint8型GNGGA结构体  *****************************
struct GNGGA_uint8{	//参见NMEA-0183协议
	uint8_t UTC[11];						//UTC时间
	uint8_t Latitude[11];				//纬度
	uint8_t La_hemi[2];					//纬度半球
	uint8_t	Longitude[12];			//经度
	uint8_t	Lo_hemi[2];					//经度半球
	uint8_t	GPS_state[2];				//定位状态
	uint8_t satellites[3];			//当前卫星数量
	uint8_t index[3];						//精度因子
	uint8_t Elevation[6];				//海拔高度
	uint8_t Elevation_base[4];	//海拔水准
}UINT_GNGGA_DATA;
//************************ char型GNGGA结构体  ******************************
struct GNGGA_char{//参见NMEA-0183协议
	char UTC[11];						//UTC时间 hhmmss.ss
	char Latitude[11];			//纬度	ddff.fffff
	char La_hemi[2];				//纬度半球 N/S
	char Longitude[12];			//经度	dddff.fffff
	char Lo_hemi[2];				//经度半球	E/W
	char GPS_state[2];			//定位状态	0/1/2
	char satellites[3];			//当前卫星数量	nn
	char index[3];					//精度因子	x.xx
	char Elevation[6];			//海拔高度	xxxx.x
	char Elevation_base[4];	//海拔水准	xx.x
}CHAR_GNGGA_DATA;
//************************ 物理世界数据类型结构体定义 ***************************************
struct REAL_GNNGA{							
	double	UTC;						//UTC时间输入
	double 	s;							//UTC时间-秒
	double 	Latitude;				//维度-度
	int 		La_hemi;				//维度半球,rule:1N,0S
	double	Longitude;			//经度-度
	int 		Lo_hemi;				//经度半球,rule:1E,0W
	int 		GPS_state;			//gps状态：0-未定位，1-非差分定位，2-差分定位
	int 		satellites;			//当前卫星数
	double	index;					//精度指数
	double	Elevation;			//海拔高度-m
	double 	Elevation_base;	//海拔水准-m
}REAL_GNGGA_DATA;
	//************************ 速度相关数据结构体定义 ***************************************
struct SPEED_DATA_{
//存放当前循环输入的结果，用于和2计算得速度。计算完成后移入2
	double time1;			//时间-秒
	double Latitude1;	//纬度-度
	double Longitude1;//经度-度
//存放上次循环输入的结果，用于和1计算得速度。
	double time2;			//时间-秒
	double Latitude2;	//纬度-度
	double Longitude2;//经度-度
//存放当此循环计算的结果。
	double curren_speed;//当前速度-m/s
}SPEED_DATA={0,0,0,0,0,0,0};//初始化

	//************************经度解析 结构体定义***************************************
struct Longitude{
	unsigned long init; 		//整型经度
	unsigned	int d;				//度
	unsigned	int f1;				//分-整数部分
	unsigned	int f2;				//分-小数部分
	double f;								//分-浮点型
	double out;							//经度最终结果,单位度
}longitude;
				//************************经度解析 结构体定义***************************************
//纬度解析 结构体定义
struct Latitude{
	int init;			//整型纬度
	int d;				//度-整数
	int f1;				//分-整数部分
	int f2;				//分-小数部分
	double f;			//分-浮点结果
	double out;		//纬度最终结果,单位度
}latitude;				
	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//--------------------------------------------------------------函数封装-------------------------------------------------------------------------
//**********************************L610初始化函数**********************************
void l610init(void)
{
	printf("AT\r\n");//检测模块
	HAL_Delay(100);
	printf("ATI\r\n");//查询模块版本
	HAL_Delay(100);
	printf("AT+CPIN?\r\n");//查询SIM卡
	HAL_Delay(100);
	printf("AT+CSQ\r\n");//查询信号
	HAL_Delay(100);
	printf("AT+CGREG?\r\n");//查询PS注册情况
	HAL_Delay(100);
	printf("AT+GTSET=\"IPRFMT\",5\r\n");//查询模块信息
	HAL_Delay(100);
	printf("AT+CGMR?\r\n");//查询版本号
	HAL_Delay(100);
	printf("AT+MIPCALL?\r\n");//查询是否获得IP
	HAL_Delay(100);
	printf("AT+MIPCALL=1\r\n");//请求IP
	HAL_Delay(100);
	printf("AT+MIPCALL?\r\n");//查询是否获得IP
	HAL_Delay(100);
	printf("AT+TCDEVINFOSET=1,\"WTOL134TCU\",\"vehicle\",\"4gE7q66Gv65hk3NzmrvHVQ==\"\r\n");//设置平台设备信息
	HAL_Delay(100);
	printf("AT+TCMQTTCONN=1,10000,240,1,1\r\n");//设置链接参数并连接
	HAL_Delay(100);
	printf("AT+TCMQTTSUB=\"$thing/down/property/WTOL134TCU/vehicle\",1\r\n");//订阅上报下行属性标签
	HAL_Delay(100);
}

//***********************************短信初始化函数**********************************
void NOTE(void) 
{
	printf("AT\r\n");//检测模块
	HAL_Delay(100);
	printf("ATI\r\n");//查询模块版本
	HAL_Delay(100);
	printf("AT+CPIN?\r\n");//查询SIM卡
	HAL_Delay(100);
	printf("AT+CSQ\r\n");//查询信号
	HAL_Delay(100);
	printf("AT+CGREG?\r\n");//查询PS注册情况
	HAL_Delay(100);
	printf("AT+COPS?\r\n");//查询运营商
	HAL_Delay(100);
		//printf("AT+CAVIMS=1\r\n");//电信卡使能（联通、移动不用）
		//HAL_Delay(100);
	printf("AT+CSCA?\r\n");//查询短信中心号码
	HAL_Delay(100);
		//printf("AT+CSCA=\".....\"\r\n");//若无短信中心号码，自主设置
		//HAL_Delay(100);
	printf("AT+CPMS=\"SM\"\r\n");//短信有优先存于SM卡中
	HAL_Delay(100);
	printf("AT+CNMI=2,1,0,0,0\r\n");//上报收到的短信编号
	HAL_Delay(100);
	printf("AT+CMGF=1\r\n");//文本短信模式
	HAL_Delay(100);
	printf("AT+CSMP=17,167,0,0\r\n");//有效期24小时
	HAL_Delay(100);
}

//***********************************下行接收函数**********************************
void down_detect()
{
				for(i=0;i<200;i++)
					{
						HAL_UART_Receive(&huart1, &input,1 ,10);
						if(input=='L')
						{
							HAL_UART_Receive(&huart1, &input,1 ,10);
							if(input=='o')
							{
								HAL_UART_Receive(&huart1, &input,1 ,10);
								if(input=='c')
								{
									HAL_UART_Receive(&huart1, &input,1 ,10);
									if(input=='k')
									{
										HAL_UART_Receive(&huart1, &input,1 ,10);
										if(input=='e')
										{
											HAL_UART_Receive(&huart1, &input,1 ,10);
											if(input=='d')
											{
												HAL_UART_Receive(&huart1, &input,1 ,10);
												if(input=='"')
												{
													HAL_UART_Receive(&huart1, &input,1 ,10);
													if(input==':')
													{
														HAL_UART_Receive(&huart1, &input,1 ,10);
														if(input=='1')//若检测到锁车命令
														{
															Locked=1;
															printf("AT+TCMQTTPUB=\"$thing/up/property/WTOL134TCU/vehicle\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"Locked\\\":1}}\"\r\n");
															
															if(exlocked==0)
															{
																//短信
																printf("AT+CMGS=\"15859624566\"\r\n");//接收号码
																HAL_Delay(100);
																printf("Locked successfully.\r\n");//短信内容
																HAL_Delay(300);
																printf("%c",Ending);//结束
																HAL_Delay(300);
																printf("AT+CMGD=1\r\n");//删除第一条短信内容，避免溢出
																HAL_Delay(200);
																
																exlocked=Locked;
															}
															
															
															/*调试用代码*/  //printf("Lock turns 1 detected");
															break;
														}
														if(input=='0')//若检测到开锁命令
														{
															Locked=0;
															printf("AT+TCMQTTPUB=\"$thing/up/property/WTOL134TCU/vehicle\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"Locked\\\":0}}\"\r\n");
															
														if(exlocked==1)
															{
																//短信
																printf("AT+CMGS=\"15859624566\"\r\n");//接收号码
																HAL_Delay(100);
																printf("Unlocked successfully.\r\n");//短信内容
																HAL_Delay(300);
																printf("%c",Ending);//结束
																HAL_Delay(300);
																printf("AT+CMGD=1\r\n");//删除第一条短信内容，避免溢出
																HAL_Delay(200);
																
																exlocked=Locked;
															}	
															
															/*调试用代码*/  //printf("Lock turns 0 detected");
															break;
														}
													}
												}
											}
										}
									}
								}
							}
						}
					}
}

//***********************************数据存储函数**********************************
void GNGGA_save()
{
			i=0;
			j=0;
//****************** 数据预存储 ********************
		uint8_t data[70];
		for(i=0;i<70;i++)//字符串长度，可改
			{
				HAL_UART_Receive(&huart6, &input, 1 ,10);//获取
					data[i]=input;
				if(input==startsign_uint)//检测到$自动跳出
					break;
			}
//******************  UTC时间存储 ********************
			for(i=0;;)
			{
				if(data[j]!=douhao)
				{		
					UINT_GNGGA_DATA.UTC[i]=data[j];
					i++;
					j++;
				}
				else
				{	
					j++;
					break;//如果是','跳出
				}
			}
//****************** 维度存储 ********************/
			for(i=0;;)
			{
				if(data[j]!=douhao)
				{		
					UINT_GNGGA_DATA.Latitude[i]=data[j];
					i++;
					j++;
				}
				else
				{	
					j++;
					break;	//如果检测到','跳出
				}
			}
//****************** 纬度半球 ********************
			for(i=0;;)
			{
				if(data[j]!=douhao)
				{		
					UINT_GNGGA_DATA.La_hemi[i]=data[j];
					i++;
					j++;
				}
				else
				{	
					j++;
					break;	//如果检测到','跳出
				}
			}
//****************** 经度 存储 ********************
			for(i=0;;)
			{
				if(data[j]!=douhao)
				{		
					UINT_GNGGA_DATA.Longitude[i]=data[j];
					i++;
					j++;
				}
				else
				{	
					j++;
					break;		//如果检测到','跳出
				}
			}
//****************** 经度半球 ********************
			for(i=0;;)
			{
				if(data[j]!=douhao)
				{		
					UINT_GNGGA_DATA.Lo_hemi[i]=data[j];
					i++;
					j++;
				}
				else
				{	
					j++;
					break;		//如果检测到','跳出
				}
			}
//****************** GPS定位状态 ********************
			for(i=0;;)
			{
				if(data[j]!=douhao)
				{		
					UINT_GNGGA_DATA.GPS_state[i]=data[j];
					i++;
					j++;
				}
				else
				{	
					j++;
					break;		//如果检测到','跳出
				}
			}
//****************** 卫星数量 存储********************/
			for(i=0;;)
			{
				if(data[j]!=douhao)
				{		
					UINT_GNGGA_DATA.satellites[i]=data[j];
					i++;
					j++;
				}
				else
				{	
					j++;
					break;		//如果检测到','跳出
				}
			}
//****************** 精度因子 存储 *******************
			for(i=0;;)
			{
				if(data[j]!=douhao)
				{		
					UINT_GNGGA_DATA.index[i]=data[j];
					i++;
					j++;
				}
				else
				{	
					j++;
					break;		//如果检测到','跳出
				}
			}
//****************** 海拔高度 ********************/
		for(i=0;;)
			{
				if(data[j]!=douhao)
				{		
					UINT_GNGGA_DATA.Elevation[i]=data[j];
					i++;
					j++;
				}
				else
				{	
					j++;
					break;		//如果检测到','跳出
				}
			}
//****************** 海拔水准 ********************
			for(i=0;;)
			{
				if(data[j]!=douhao)
				{		
					UINT_GNGGA_DATA.Elevation_base[i]=data[j];
					i++;
					j++;
				}
				else
				{	
					j++;
					break;		//如果检测到','跳出
				}
			}
		}
//-----------------定位信息解析函数------------------------
void GNGGA_analysis()
{
	//****************** UCT转换 1 ********************
				char* p;//空数组定义	
				//结构体定义
				struct uct{						
					int time;
					int h;
					int m;
					int s1;
					int s2;
					double s;
				}uct;
			
				//uint8转char
				for(i=0;i<10;i++)								
				{
					CHAR_GNGGA_DATA.UTC[i]=(char)UINT_GNGGA_DATA.UTC[i];
				}
			
				//字符转整型
				REAL_GNGGA_DATA.UTC=strtof(CHAR_GNGGA_DATA.UTC,&p);
				uct.time=REAL_GNGGA_DATA.UTC*100.0;
				
				//数据解析 存入uct.s，单位秒，float
				uct.h =uct.time/1000000;
				uct.m =uct.time/10000;
				uct.m =uct.m%100;
				uct.s1 =uct.time/100;
				uct.s1 =uct.s1%100;
				uct.s2 =uct.time%100;
				uct.s = uct.h*3600.0+uct.m*60.0+uct.s1*1.0+uct.s2/1000;
				
				//时间解析结果存储
				REAL_GNGGA_DATA.UTC=uct.s;
				SPEED_DATA.time1=uct.s;//时分秒转换为秒

//******************  纬度解析  ********************
				
				//char* p  ;空数组已经定义	
				//纬度 uint8转char
				for(i=0;i<10;i++)								
				{CHAR_GNGGA_DATA.Latitude[i]=(char)UINT_GNGGA_DATA.Latitude[i];}
				
				//字符转整型
				REAL_GNGGA_DATA.Latitude=strtof(CHAR_GNGGA_DATA.Latitude,&p);
				latitude.init =REAL_GNGGA_DATA.Latitude*100000.0;
				
				//纬度解析
				latitude.d=latitude.init/10000000.0;
				latitude.f1=latitude.init/100000.0;
				latitude.f1=latitude.f1%100;
				latitude.f2=latitude.init%100000;
				latitude.f=(double)latitude.f1*1.0+(double)latitude.f2/100000;
				latitude.f=latitude.f/60.0;
				latitude.out=(double)latitude.d+latitude.f;
				
				//纬度解析结果存储
				REAL_GNGGA_DATA.Latitude=latitude.out;
				SPEED_DATA.Latitude1=latitude.out;
				
//******************  经度解析 ********************
				//char* p  ;空数组已经定义	
				
				//uint8转char
				for(i=0;i<11;i++)								
				{CHAR_GNGGA_DATA.Longitude[i]=(char)UINT_GNGGA_DATA.Longitude[i];}
				
				//字符转整型
				REAL_GNGGA_DATA.Longitude=strtof(CHAR_GNGGA_DATA.Longitude,&p);
				longitude.init =REAL_GNGGA_DATA.Longitude*100000;
				
				//纬度解析
				longitude.d=longitude.init/10000000.0;
				longitude.f1=longitude.init/100000.0;
				longitude.f1=longitude.f1%100;
				longitude.f2=longitude.init%100000;
				longitude.f=(double)longitude.f1*1.0+(double)longitude.f2/100000;
				longitude.f=(double)longitude.f/60.0;
				longitude.out=(double)longitude.d+(double)longitude.f;
				
				//经度解析结果存储
				REAL_GNGGA_DATA.Longitude=longitude.out;
				SPEED_DATA.Longitude1=longitude.out;
				
				//海拔转化
				for(i=0;i<5;i++)								
				{CHAR_GNGGA_DATA.Elevation[i]=(char)UINT_GNGGA_DATA.Elevation[i];}
				REAL_GNGGA_DATA.Elevation=strtof(CHAR_GNGGA_DATA.Elevation,&p);
				
			//printf("REAL_GNGGA_DATA.UTC= %lf\n\n",REAL_GNGGA_DATA.UTC);
			//printf("REAL_GNGGA_DATA.Latitude= %lf\n\n",REAL_GNGGA_DATA.Latitude);
			//printf("REAL_GNGGA_DATA.Longitude %lf\n\n\n",REAL_GNGGA_DATA.Longitude);
//******************  速度计算  ********************
					
		/*时间SPEED_DATA.time1=uct.s;
		//纬度SPEED_DATA.Latitude1=latitude.out;
		//经度	SPEED_DATA.Longitude1=longitude.out;*/
				
				//定义距离、地球半径
					//两点距离，单位米
						double distance;
					//地球半径，单位米
					double R_earth=6378137;
				
				//角度转弧度
						//定义
							double rad_la1;//纬度1的弧度值
							double rad_lo1;//经度1的弧度制
							double rad_la2;//纬度2的弧度值
							double rad_lo2;//经度2的弧度制
							//Π的宏定义为pi
						//纬度
							rad_la1=(pi/180.0)*SPEED_DATA.Latitude1;
							rad_la2=(pi/180.0)*SPEED_DATA.Latitude2;
						//经度
							rad_lo1=(pi/180.0)*SPEED_DATA.Longitude1;
							rad_lo2=(pi/180.0)*SPEED_DATA.Longitude2;
							
					
				//计算 位移 
					double sin1,sin2,cos1,cos2,cos3,arccos0,mul1,mul2,sum0;
					sin1=sin(rad_la1);
					sin2=sin(rad_la2);
					cos1=cos(rad_la1);
					cos2=cos(rad_la2);
					cos3=cos(rad_lo1-rad_lo2);
					mul1=sin1*sin2;
					mul2=cos1*cos2*cos3;
					sum0=mul1+mul2;
					arccos0=acos(sum0);
					
					distance=arccos0*R_earth;
					
				//计算 时间差
					double time_dif=fabs(SPEED_DATA.time1-SPEED_DATA.time2);
					
				//计算 速度
					double speed_cal;
					speed_cal=distance/time_dif;
					
				//返回 到SPEED_DATA
				if(SPEED_DATA.Latitude2!=0)
					if(speed_cal<100)
					SPEED_DATA.curren_speed=speed_cal;
					
				//	printf("SPEED_DATA.curren_speed %lf\n\n\n",SPEED_DATA.curren_speed);
					
//******************  把 SPEED_DATA中的1存入2 ********************

				SPEED_DATA.Latitude2=SPEED_DATA.Latitude1;
				SPEED_DATA.Longitude2=SPEED_DATA.Longitude1;
				SPEED_DATA.time2=SPEED_DATA.time1;
}	

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */



int main(void)
{
	
  /* USER CODE BEGIN 1 */
	
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		
	//************************ L610功能初始化 ***************************************
		if(counter==0)//初次启动
		{
			l610init();	//L610设置初始化
			NOTE();			//短信设置
			counter++;
		}
		if(counter>200)//每200次初始化，防掉线
		{
			l610init();//L610初始化
			NOTE();//短信设置
			counter=0;//计数器置零
		}
		else
		{counter+=1;}
//*************************  $GNGGA字头识别函数  **************************************
		down_detect();
		i=0;
		while(1)
		{
			for(;;)
				{
					HAL_UART_Receive(&huart6, &input,1,1);
					if(input==startsign_uint)
					break;
				}

			uint8_t uint_input_6[6];
			char 		char_input_6[6]={"000000"};
				
			HAL_UART_Receive(&huart6, uint_input_6,6,10);
			
			for(i=0;i<6;i++)
				{char_input_6[i]=(char)uint_input_6[i];}
					
			if(strcmp(char_input_6,GNGGA)==0)
				{
					//printf("GNGGA DETECTED");//调试用
					break;
				}
		}
		
			//数据存储
			GNGGA_save();
//************************存储结果测试*(调试用)**************************
			
		/*	printf("UINT_GNGGA_DATA.UTC =  %s\n\n",UINT_GNGGA_DATA.UTC);
				printf("UINT_GNGGA_DATA.Latitude =  %s\n\n",UINT_GNGGA_DATA.Latitude);
				printf("UINT_GNGGA_DATA.La_hemi =  %s\n\n",UINT_GNGGA_DATA.La_hemi);
				printf("UINT_GNGGA_DATA.Longitude =  %s\n\n",UINT_GNGGA_DATA.Longitude);
				printf("UINT_GNGGA_DATA.Lo_hemi =  %s\n\n",UINT_GNGGA_DATA.Lo_hemi);
				printf("UINT_GNGGA_DATA.GPS_state =  %s\n\n",UINT_GNGGA_DATA.GPS_state);
				printf("UINT_GNGGA_DATA.satellites =  %s\n\n",UINT_GNGGA_DATA.satellites);
				printf("UINT_GNGGA_DATA.index =  %s\n\n",UINT_GNGGA_DATA.satellites);
				printf("UINT_GNGGA_DATA.Elevation =  %s\n\n",UINT_GNGGA_DATA.Elevation);
				printf("UINT_GNGGA_DATA.Elevation_base =  %s\n\n",UINT_GNGGA_DATA.Elevation_base);
		*/
		
			//下行接收
			down_detect();
			//定位信息解析
			GNGGA_analysis();
			//下行接收
			down_detect();
			
			//判断状态并上行定位信息和速度
					if(Locked==1)
						{printf("AT+TCMQTTPUB=\"$thing/up/property/WTOL134TCU/vehicle\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"Reset\\\":%d,\\\"Locked\\\":%d,\\\"Speed\\\":%.3lf,\\\"Elevation\\\":\\\"%lf\\\",\\\"Longitude\\\":\\\"%lf\\\",\\\"Latitude\\\":\\\"%lf\\\"}}\"\r\n",Reset,Locked,SPEED_DATA.curren_speed,REAL_GNGGA_DATA.Elevation,REAL_GNGGA_DATA.Longitude,REAL_GNGGA_DATA.Latitude);}
					else
						printf("AT+TCMQTTPUB=\"$thing/up/property/WTOL134TCU/vehicle\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"Reset\\\":%d,\\\"Speed\\\":%.3lf,\\\"Elevation\\\":\\\"%lf\\\",\\\"Longitude\\\":\\\"%lf\\\",\\\"Latitude\\\":\\\"%lf\\\"}}\"\r\n",Reset,SPEED_DATA.curren_speed,REAL_GNGGA_DATA.Elevation,REAL_GNGGA_DATA.Longitude,REAL_GNGGA_DATA.Latitude);
					
					char Ending = 0x1A;//结束符，无回车换行符号
					if((SPEED_DATA.curren_speed>=speed_standard)&&(k<10)&&(Locked==1))//k用于短信发送的次数限制
					{
						printf("AT+CMGS=\"15859624566\"\r\n");//接收号码
						HAL_Delay(100);
						printf("Your vehicle is moving, speed: %lf, itmay have been STOLEN!! Please check your WeChat official account TENCENT_lianlian.\r\n",SPEED_DATA.curren_speed);//短信内容
						HAL_Delay(300);
						printf("%c",Ending);//结束
						HAL_Delay(300);
						printf("AT+CMGD=1\r\n");//删除第一条短信内容，避免溢出
						HAL_Delay(200);
						k++;
					}
		
    /* USER CODE BEGIN 3 */
  }
	
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

