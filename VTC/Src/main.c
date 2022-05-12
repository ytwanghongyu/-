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

//**************************** ȫ�ֱ�������  **************************
	int k=0,i=0,j=0,counter=0;//ѭ����������ÿ1000��ѭ�����²���
	int Reset=0,Locked=0,Stolen=0,exlocked=0;//״̬����
	const double pi=3.1415926535;//��ֵ
	double speed_standard = 1.45;//�ٶ���ֵ,��λm/s(�ɵ�)
	char douhao=',';
	char GNGGA[6]={"GNGGA,"};//GNGGA��ʽGPS�����ʼ��ʶ��
	uint8_t startsign_uint='$',input;//GPS�����ʼ��ʶ��
	char Ending = 0x1A;//���������޻س����з���
	//************************ uint8��GNGGA�ṹ��  *****************************
struct GNGGA_uint8{	//�μ�NMEA-0183Э��
	uint8_t UTC[11];						//UTCʱ��
	uint8_t Latitude[11];				//γ��
	uint8_t La_hemi[2];					//γ�Ȱ���
	uint8_t	Longitude[12];			//����
	uint8_t	Lo_hemi[2];					//���Ȱ���
	uint8_t	GPS_state[2];				//��λ״̬
	uint8_t satellites[3];			//��ǰ��������
	uint8_t index[3];						//��������
	uint8_t Elevation[6];				//���θ߶�
	uint8_t Elevation_base[4];	//����ˮ׼
}UINT_GNGGA_DATA;
//************************ char��GNGGA�ṹ��  ******************************
struct GNGGA_char{//�μ�NMEA-0183Э��
	char UTC[11];						//UTCʱ�� hhmmss.ss
	char Latitude[11];			//γ��	ddff.fffff
	char La_hemi[2];				//γ�Ȱ��� N/S
	char Longitude[12];			//����	dddff.fffff
	char Lo_hemi[2];				//���Ȱ���	E/W
	char GPS_state[2];			//��λ״̬	0/1/2
	char satellites[3];			//��ǰ��������	nn
	char index[3];					//��������	x.xx
	char Elevation[6];			//���θ߶�	xxxx.x
	char Elevation_base[4];	//����ˮ׼	xx.x
}CHAR_GNGGA_DATA;
//************************ ���������������ͽṹ�嶨�� ***************************************
struct REAL_GNNGA{							
	double	UTC;						//UTCʱ������
	double 	s;							//UTCʱ��-��
	double 	Latitude;				//ά��-��
	int 		La_hemi;				//ά�Ȱ���,rule:1N,0S
	double	Longitude;			//����-��
	int 		Lo_hemi;				//���Ȱ���,rule:1E,0W
	int 		GPS_state;			//gps״̬��0-δ��λ��1-�ǲ�ֶ�λ��2-��ֶ�λ
	int 		satellites;			//��ǰ������
	double	index;					//����ָ��
	double	Elevation;			//���θ߶�-m
	double 	Elevation_base;	//����ˮ׼-m
}REAL_GNGGA_DATA;
	//************************ �ٶ�������ݽṹ�嶨�� ***************************************
struct SPEED_DATA_{
//��ŵ�ǰѭ������Ľ�������ں�2������ٶȡ�������ɺ�����2
	double time1;			//ʱ��-��
	double Latitude1;	//γ��-��
	double Longitude1;//����-��
//����ϴ�ѭ������Ľ�������ں�1������ٶȡ�
	double time2;			//ʱ��-��
	double Latitude2;	//γ��-��
	double Longitude2;//����-��
//��ŵ���ѭ������Ľ����
	double curren_speed;//��ǰ�ٶ�-m/s
}SPEED_DATA={0,0,0,0,0,0,0};//��ʼ��

	//************************���Ƚ��� �ṹ�嶨��***************************************
struct Longitude{
	unsigned long init; 		//���;���
	unsigned	int d;				//��
	unsigned	int f1;				//��-��������
	unsigned	int f2;				//��-С������
	double f;								//��-������
	double out;							//�������ս��,��λ��
}longitude;
				//************************���Ƚ��� �ṹ�嶨��***************************************
//γ�Ƚ��� �ṹ�嶨��
struct Latitude{
	int init;			//����γ��
	int d;				//��-����
	int f1;				//��-��������
	int f2;				//��-С������
	double f;			//��-������
	double out;		//γ�����ս��,��λ��
}latitude;				
	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//--------------------------------------------------------------������װ-------------------------------------------------------------------------
//**********************************L610��ʼ������**********************************
void l610init(void)
{
	printf("AT\r\n");//���ģ��
	HAL_Delay(100);
	printf("ATI\r\n");//��ѯģ��汾
	HAL_Delay(100);
	printf("AT+CPIN?\r\n");//��ѯSIM��
	HAL_Delay(100);
	printf("AT+CSQ\r\n");//��ѯ�ź�
	HAL_Delay(100);
	printf("AT+CGREG?\r\n");//��ѯPSע�����
	HAL_Delay(100);
	printf("AT+GTSET=\"IPRFMT\",5\r\n");//��ѯģ����Ϣ
	HAL_Delay(100);
	printf("AT+CGMR?\r\n");//��ѯ�汾��
	HAL_Delay(100);
	printf("AT+MIPCALL?\r\n");//��ѯ�Ƿ���IP
	HAL_Delay(100);
	printf("AT+MIPCALL=1\r\n");//����IP
	HAL_Delay(100);
	printf("AT+MIPCALL?\r\n");//��ѯ�Ƿ���IP
	HAL_Delay(100);
	printf("AT+TCDEVINFOSET=1,\"WTOL134TCU\",\"vehicle\",\"4gE7q66Gv65hk3NzmrvHVQ==\"\r\n");//����ƽ̨�豸��Ϣ
	HAL_Delay(100);
	printf("AT+TCMQTTCONN=1,10000,240,1,1\r\n");//�������Ӳ���������
	HAL_Delay(100);
	printf("AT+TCMQTTSUB=\"$thing/down/property/WTOL134TCU/vehicle\",1\r\n");//�����ϱ��������Ա�ǩ
	HAL_Delay(100);
}

//***********************************���ų�ʼ������**********************************
void NOTE(void) 
{
	printf("AT\r\n");//���ģ��
	HAL_Delay(100);
	printf("ATI\r\n");//��ѯģ��汾
	HAL_Delay(100);
	printf("AT+CPIN?\r\n");//��ѯSIM��
	HAL_Delay(100);
	printf("AT+CSQ\r\n");//��ѯ�ź�
	HAL_Delay(100);
	printf("AT+CGREG?\r\n");//��ѯPSע�����
	HAL_Delay(100);
	printf("AT+COPS?\r\n");//��ѯ��Ӫ��
	HAL_Delay(100);
		//printf("AT+CAVIMS=1\r\n");//���ſ�ʹ�ܣ���ͨ���ƶ����ã�
		//HAL_Delay(100);
	printf("AT+CSCA?\r\n");//��ѯ�������ĺ���
	HAL_Delay(100);
		//printf("AT+CSCA=\".....\"\r\n");//���޶������ĺ��룬��������
		//HAL_Delay(100);
	printf("AT+CPMS=\"SM\"\r\n");//���������ȴ���SM����
	HAL_Delay(100);
	printf("AT+CNMI=2,1,0,0,0\r\n");//�ϱ��յ��Ķ��ű��
	HAL_Delay(100);
	printf("AT+CMGF=1\r\n");//�ı�����ģʽ
	HAL_Delay(100);
	printf("AT+CSMP=17,167,0,0\r\n");//��Ч��24Сʱ
	HAL_Delay(100);
}

//***********************************���н��պ���**********************************
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
														if(input=='1')//����⵽��������
														{
															Locked=1;
															printf("AT+TCMQTTPUB=\"$thing/up/property/WTOL134TCU/vehicle\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"Locked\\\":1}}\"\r\n");
															
															if(exlocked==0)
															{
																//����
																printf("AT+CMGS=\"15859624566\"\r\n");//���պ���
																HAL_Delay(100);
																printf("Locked successfully.\r\n");//��������
																HAL_Delay(300);
																printf("%c",Ending);//����
																HAL_Delay(300);
																printf("AT+CMGD=1\r\n");//ɾ����һ���������ݣ��������
																HAL_Delay(200);
																
																exlocked=Locked;
															}
															
															
															/*�����ô���*/  //printf("Lock turns 1 detected");
															break;
														}
														if(input=='0')//����⵽��������
														{
															Locked=0;
															printf("AT+TCMQTTPUB=\"$thing/up/property/WTOL134TCU/vehicle\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"Locked\\\":0}}\"\r\n");
															
														if(exlocked==1)
															{
																//����
																printf("AT+CMGS=\"15859624566\"\r\n");//���պ���
																HAL_Delay(100);
																printf("Unlocked successfully.\r\n");//��������
																HAL_Delay(300);
																printf("%c",Ending);//����
																HAL_Delay(300);
																printf("AT+CMGD=1\r\n");//ɾ����һ���������ݣ��������
																HAL_Delay(200);
																
																exlocked=Locked;
															}	
															
															/*�����ô���*/  //printf("Lock turns 0 detected");
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

//***********************************���ݴ洢����**********************************
void GNGGA_save()
{
			i=0;
			j=0;
//****************** ����Ԥ�洢 ********************
		uint8_t data[70];
		for(i=0;i<70;i++)//�ַ������ȣ��ɸ�
			{
				HAL_UART_Receive(&huart6, &input, 1 ,10);//��ȡ
					data[i]=input;
				if(input==startsign_uint)//��⵽$�Զ�����
					break;
			}
//******************  UTCʱ��洢 ********************
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
					break;//�����','����
				}
			}
//****************** ά�ȴ洢 ********************/
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
					break;	//�����⵽','����
				}
			}
//****************** γ�Ȱ��� ********************
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
					break;	//�����⵽','����
				}
			}
//****************** ���� �洢 ********************
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
					break;		//�����⵽','����
				}
			}
//****************** ���Ȱ��� ********************
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
					break;		//�����⵽','����
				}
			}
//****************** GPS��λ״̬ ********************
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
					break;		//�����⵽','����
				}
			}
//****************** �������� �洢********************/
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
					break;		//�����⵽','����
				}
			}
//****************** �������� �洢 *******************
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
					break;		//�����⵽','����
				}
			}
//****************** ���θ߶� ********************/
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
					break;		//�����⵽','����
				}
			}
//****************** ����ˮ׼ ********************
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
					break;		//�����⵽','����
				}
			}
		}
//-----------------��λ��Ϣ��������------------------------
void GNGGA_analysis()
{
	//****************** UCTת�� 1 ********************
				char* p;//�����鶨��	
				//�ṹ�嶨��
				struct uct{						
					int time;
					int h;
					int m;
					int s1;
					int s2;
					double s;
				}uct;
			
				//uint8תchar
				for(i=0;i<10;i++)								
				{
					CHAR_GNGGA_DATA.UTC[i]=(char)UINT_GNGGA_DATA.UTC[i];
				}
			
				//�ַ�ת����
				REAL_GNGGA_DATA.UTC=strtof(CHAR_GNGGA_DATA.UTC,&p);
				uct.time=REAL_GNGGA_DATA.UTC*100.0;
				
				//���ݽ��� ����uct.s����λ�룬float
				uct.h =uct.time/1000000;
				uct.m =uct.time/10000;
				uct.m =uct.m%100;
				uct.s1 =uct.time/100;
				uct.s1 =uct.s1%100;
				uct.s2 =uct.time%100;
				uct.s = uct.h*3600.0+uct.m*60.0+uct.s1*1.0+uct.s2/1000;
				
				//ʱ���������洢
				REAL_GNGGA_DATA.UTC=uct.s;
				SPEED_DATA.time1=uct.s;//ʱ����ת��Ϊ��

//******************  γ�Ƚ���  ********************
				
				//char* p  ;�������Ѿ�����	
				//γ�� uint8תchar
				for(i=0;i<10;i++)								
				{CHAR_GNGGA_DATA.Latitude[i]=(char)UINT_GNGGA_DATA.Latitude[i];}
				
				//�ַ�ת����
				REAL_GNGGA_DATA.Latitude=strtof(CHAR_GNGGA_DATA.Latitude,&p);
				latitude.init =REAL_GNGGA_DATA.Latitude*100000.0;
				
				//γ�Ƚ���
				latitude.d=latitude.init/10000000.0;
				latitude.f1=latitude.init/100000.0;
				latitude.f1=latitude.f1%100;
				latitude.f2=latitude.init%100000;
				latitude.f=(double)latitude.f1*1.0+(double)latitude.f2/100000;
				latitude.f=latitude.f/60.0;
				latitude.out=(double)latitude.d+latitude.f;
				
				//γ�Ƚ�������洢
				REAL_GNGGA_DATA.Latitude=latitude.out;
				SPEED_DATA.Latitude1=latitude.out;
				
//******************  ���Ƚ��� ********************
				//char* p  ;�������Ѿ�����	
				
				//uint8תchar
				for(i=0;i<11;i++)								
				{CHAR_GNGGA_DATA.Longitude[i]=(char)UINT_GNGGA_DATA.Longitude[i];}
				
				//�ַ�ת����
				REAL_GNGGA_DATA.Longitude=strtof(CHAR_GNGGA_DATA.Longitude,&p);
				longitude.init =REAL_GNGGA_DATA.Longitude*100000;
				
				//γ�Ƚ���
				longitude.d=longitude.init/10000000.0;
				longitude.f1=longitude.init/100000.0;
				longitude.f1=longitude.f1%100;
				longitude.f2=longitude.init%100000;
				longitude.f=(double)longitude.f1*1.0+(double)longitude.f2/100000;
				longitude.f=(double)longitude.f/60.0;
				longitude.out=(double)longitude.d+(double)longitude.f;
				
				//���Ƚ�������洢
				REAL_GNGGA_DATA.Longitude=longitude.out;
				SPEED_DATA.Longitude1=longitude.out;
				
				//����ת��
				for(i=0;i<5;i++)								
				{CHAR_GNGGA_DATA.Elevation[i]=(char)UINT_GNGGA_DATA.Elevation[i];}
				REAL_GNGGA_DATA.Elevation=strtof(CHAR_GNGGA_DATA.Elevation,&p);
				
			//printf("REAL_GNGGA_DATA.UTC= %lf\n\n",REAL_GNGGA_DATA.UTC);
			//printf("REAL_GNGGA_DATA.Latitude= %lf\n\n",REAL_GNGGA_DATA.Latitude);
			//printf("REAL_GNGGA_DATA.Longitude %lf\n\n\n",REAL_GNGGA_DATA.Longitude);
//******************  �ٶȼ���  ********************
					
		/*ʱ��SPEED_DATA.time1=uct.s;
		//γ��SPEED_DATA.Latitude1=latitude.out;
		//����	SPEED_DATA.Longitude1=longitude.out;*/
				
				//������롢����뾶
					//������룬��λ��
						double distance;
					//����뾶����λ��
					double R_earth=6378137;
				
				//�Ƕ�ת����
						//����
							double rad_la1;//γ��1�Ļ���ֵ
							double rad_lo1;//����1�Ļ�����
							double rad_la2;//γ��2�Ļ���ֵ
							double rad_lo2;//����2�Ļ�����
							//���ĺ궨��Ϊpi
						//γ��
							rad_la1=(pi/180.0)*SPEED_DATA.Latitude1;
							rad_la2=(pi/180.0)*SPEED_DATA.Latitude2;
						//����
							rad_lo1=(pi/180.0)*SPEED_DATA.Longitude1;
							rad_lo2=(pi/180.0)*SPEED_DATA.Longitude2;
							
					
				//���� λ�� 
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
					
				//���� ʱ���
					double time_dif=fabs(SPEED_DATA.time1-SPEED_DATA.time2);
					
				//���� �ٶ�
					double speed_cal;
					speed_cal=distance/time_dif;
					
				//���� ��SPEED_DATA
				if(SPEED_DATA.Latitude2!=0)
					if(speed_cal<100)
					SPEED_DATA.curren_speed=speed_cal;
					
				//	printf("SPEED_DATA.curren_speed %lf\n\n\n",SPEED_DATA.curren_speed);
					
//******************  �� SPEED_DATA�е�1����2 ********************

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
		
	//************************ L610���ܳ�ʼ�� ***************************************
		if(counter==0)//��������
		{
			l610init();	//L610���ó�ʼ��
			NOTE();			//��������
			counter++;
		}
		if(counter>200)//ÿ200�γ�ʼ����������
		{
			l610init();//L610��ʼ��
			NOTE();//��������
			counter=0;//����������
		}
		else
		{counter+=1;}
//*************************  $GNGGA��ͷʶ����  **************************************
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
					//printf("GNGGA DETECTED");//������
					break;
				}
		}
		
			//���ݴ洢
			GNGGA_save();
//************************�洢�������*(������)**************************
			
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
		
			//���н���
			down_detect();
			//��λ��Ϣ����
			GNGGA_analysis();
			//���н���
			down_detect();
			
			//�ж�״̬�����ж�λ��Ϣ���ٶ�
					if(Locked==1)
						{printf("AT+TCMQTTPUB=\"$thing/up/property/WTOL134TCU/vehicle\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"Reset\\\":%d,\\\"Locked\\\":%d,\\\"Speed\\\":%.3lf,\\\"Elevation\\\":\\\"%lf\\\",\\\"Longitude\\\":\\\"%lf\\\",\\\"Latitude\\\":\\\"%lf\\\"}}\"\r\n",Reset,Locked,SPEED_DATA.curren_speed,REAL_GNGGA_DATA.Elevation,REAL_GNGGA_DATA.Longitude,REAL_GNGGA_DATA.Latitude);}
					else
						printf("AT+TCMQTTPUB=\"$thing/up/property/WTOL134TCU/vehicle\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"Reset\\\":%d,\\\"Speed\\\":%.3lf,\\\"Elevation\\\":\\\"%lf\\\",\\\"Longitude\\\":\\\"%lf\\\",\\\"Latitude\\\":\\\"%lf\\\"}}\"\r\n",Reset,SPEED_DATA.curren_speed,REAL_GNGGA_DATA.Elevation,REAL_GNGGA_DATA.Longitude,REAL_GNGGA_DATA.Latitude);
					
					char Ending = 0x1A;//���������޻س����з���
					if((SPEED_DATA.curren_speed>=speed_standard)&&(k<10)&&(Locked==1))//k���ڶ��ŷ��͵Ĵ�������
					{
						printf("AT+CMGS=\"15859624566\"\r\n");//���պ���
						HAL_Delay(100);
						printf("Your vehicle is moving, speed: %lf, itmay have been STOLEN!! Please check your WeChat official account TENCENT_lianlian.\r\n",SPEED_DATA.curren_speed);//��������
						HAL_Delay(300);
						printf("%c",Ending);//����
						HAL_Delay(300);
						printf("AT+CMGD=1\r\n");//ɾ����һ���������ݣ��������
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

