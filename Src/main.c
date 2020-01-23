
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"

/* USER CODE BEGIN Includes */

#define MAX_HZ 1500 //~4500 - 50Hz, 1500 - 150Hz

#define FlAdr (uint32_t)(0x08007D00)
#define Lamp_5 10
#define Lamp_2 4
#define Lamp_1 2
#define Lamp_05 1

#define Lmp10Min 0x01
#define LmpCnnct 0x02
#define LmpCalib 0x04
#define LmpCalEr 0x08
#define LmpOverr 0x10


#define AttPause 1
#define AttPauseAfter 2


#define MBReinitTime 60

#define Kf 50


const unsigned char auchCRCHi[] =
{
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40
};

const unsigned char auchCRCLo[] =
{
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40
};


const uint32_t USART_const [9] = {9600,4800,9600,14400,19200,38400,56000,57600,115200};
const uint8_t  TIMER_const [9] = {4,8,4,3,2,2,2,2,2};

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim21;
LPTIM_HandleTypeDef LptimHandle;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//TIM_HandleTypeDef htim2;
//TIM_HandleTypeDef htim21;
TIM_HandleTypeDef htim22;

//RTC_HandleTypeDef hrtc;

ADC_AnalogWDGConfTypeDef awdgc1;

RTC_TimeTypeDef Time1;
RTC_DateTypeDef Date1;


volatile uint8_t mb_err=0;
volatile uint8_t mb_err_prev=0;

uint16_t CRCCod;

volatile uint8_t lamp=0;
volatile uint8_t lamp_cnt=0;
volatile uint8_t lamp_f=0;
volatile uint8_t lamp_err=0;
volatile uint8_t lamp_double=0;
volatile uint8_t lamp_df=0;

volatile uint8_t mbReinitCnt=0;


uint16_t max=0;
uint16_t min=0xFFFF;

uint16_t MBPause=0;
uint16_t MBPauseCnt=0;

uint32_t zeros=0;

unsigned char res_buffer[30];					// приемный буфер
unsigned char write_buffer[80];					// буфер для передачи
volatile unsigned char res_wr_index;


float testy;

uint16_t reg_MB[30];
uint16_t reg_MB1[10];
uint16_t reg_MB2[8];

uint16_t reg_MB3[8];

float reg_MBf[3];

volatile uint8_t NeedChangeSpeed=0;

uint8_t Sin=0;
uint32_t Dot=0;
uint32_t PrevDot=60000;
uint32_t PrevDot4=15000;
uint32_t PrevDot42=45000;
uint32_t NowDot=60000;

uint32_t Dooot=0;
double Dot1=0;
double DotMax=0;
double DotMin=1250000;

double Dot12,DotMax2,DotMin2;
	
uint32_t Hz1,Hz2;

uint16_t ChCnt=0;
uint8_t ChF=0;

uint16_t hz[800];
uint16_t hz1[800];
uint16_t hz2[800];

uint8_t wtf=0;
uint16_t DotSum[Kf];

uint16_t Maxs[Kf];
uint16_t RMSs[Kf];
uint16_t MaxS;
uint16_t RMSS;
uint16_t Max_RMSs=0;
uint16_t Max_Max=0;


uint16_t TT1=0xC8;
uint16_t TT2=0xC8;
uint16_t TT3=0xC8;

double ampl,rms;

volatile uint8_t A90=0;

volatile uint8_t FlagA90=0;

volatile uint8_t AttP=0;
volatile uint8_t AttPAft=0;

volatile uint8_t AttPF=0;
volatile uint8_t AttPAftF=0;

uint16_t Mins[10];

uint16_t adc1[4];

uint8_t dmaflg=0;

uint16_t prev1,prev2=0xFFFF;
uint16_t now1,now2,now_vref;
uint16_t now10,now20;
uint16_t now11,now21;
uint16_t now12,now22;

uint16_t now3=0xFFFF;
uint8_t step=0;
uint16_t herz=0;

uint8_t now[2];

uint8_t Flag=1;
uint8_t FlagMB=1;
uint8_t FlagMB2=1;

uint32_t FBI[3][10];
uint32_t FB2[10];
uint32_t FB3[10];
uint16_t abcd[8];
uint32_t FBI0;
uint32_t FBI1;
uint32_t FBI2;
uint32_t FBI3;

uint32_t OHBOY = 0;

#define MB_ATT_ON  reg_MB[19]
#define MB_ATT_OFF reg_MB[20]
	
#define MB_ADR reg_MB[12]

#define MB_NAME reg_MB[11]

#define MB_RMS reg_MB2[0]
#define MB_RMSMAX reg_MB2[1]
#define MB_HZ reg_MB2[2]

#define MB_SPEED reg_MB[13]

#define MB_HZ_F reg_MB[10]
#define MB_HZ_I reg_MB[18]

#define MB_RMS_N_F reg_MB[7]
#define MB_RMS_N_I reg_MB[15]

#define MB_RMS_O_F reg_MB[9]
#define MB_RMS_O_I reg_MB[17]	

#define MB_AMPL_N_F reg_MB[6]
#define MB_AMPL_N_I reg_MB[14]

#define MB_AMPL_O_F reg_MB[8]
#define MB_AMPL_O_I reg_MB[16]	


#define MB_RMS_ZERO reg_MB[5]
#define MB_AMPL_ZERO reg_MB[4]	

#define MB_HZ_NOW reg_MB[3]

#define MB_RMS_NOW reg_MB[2]
#define MB_AMPL_NOW reg_MB[1]	


#define MB_RMS_T_fl  	 reg_MBf[0]
#define MB_RMSMAX_T_fl reg_MBf[1]
#define MB_HZ_T_fl     reg_MBf[2]


volatile uint8_t FORCE_ATT=0;



uint16_t MB_HZ_1=0;

union {
	uint16_t int1[2];
	float float1;
} union1;





const double tens[16]={1,10,100,1000,10000,100000,1000000,10000000,100000000,1000000000,10000000000,100000000000,
1000000000000,10000000000000,100000000000000,1000000000000000};


	uint8_t hz_st=0;
	uint8_t hz_st2=0;
	uint16_t hz_f=0;
	uint16_t hz_f2=0;


	volatile uint8_t bl_flag = 0;
	volatile uint8_t reset_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(uint8_t bd);
static void MX_TIM21_Init(void);
static void MX_RTC_Init(void);
static void MX_IWDG_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

unsigned int CRC16 (unsigned char *pucFrame, unsigned int usLen)
													// pucFrame - указатель на начало буфера (адрес)
													// usLen - длина пакета
{
	volatile unsigned char   ucCRCHi = 0xFF;					// старший байт црц
	volatile unsigned char   ucCRCLo = 0xFF;					// младший байт црц
	volatile int             iIndex;
	int i=0;

	while (usLen--)									// цикл, usLen уменьшается на 1 при каждой итерации
	{
		iIndex = ucCRCLo ^ pucFrame[i];				// ксорим ucCRCLo  и байт, на который указывает pucFrame.
		i++;										// полученное значение будет индексом iIndex в таблицах. pucFrame инкрементируется.
		ucCRCLo = ucCRCHi ^ (auchCRCHi[iIndex] );	// ксорим ucCRCHi и значение из таблицы aucCRCHi с индексом iIndex.
		ucCRCHi = ( auchCRCLo[iIndex] );			// ucCRCHi равно значению из таблицы aucCRCLo с индексом iIndex
	}
	return ( ucCRCHi << 8 | ucCRCLo );				// Возвращаем старший и младший байт CRC в виде 16-разрядного слова
}


static void MX_TIM2_Init(uint8_t bd)
{
	__HAL_RCC_TIM2_CLK_ENABLE();
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 32000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIMER_const[bd];
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	
	
	TIM2->PSC = 32000 - 1; 
	TIM2->ARR = TIMER_const[bd]; 
	TIM2->DIER |= TIM_DIER_UIE; 
	TIM2->CR1 |= TIM_CR1_OPM;
	//TIM14->CR1 |= TIM_CR1_CEN; 
	NVIC_SetPriority(TIM2_IRQn, 0); 
	NVIC_EnableIRQ(TIM2_IRQn);
}

static void MX_TIM21_Init(void)
{

	
	__HAL_RCC_TIM21_CLK_ENABLE();
  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 32000-1;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 500;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	
	TIM21->PSC = 32000-1; 
	TIM21->ARR = 500 ; 
	TIM21->DIER |= TIM_DIER_UIE; 
	//TIM21->CR1 |= TIM_CR1_OPM;
	TIM21->CR1 |= TIM_CR1_CEN; 
	HAL_NVIC_SetPriority(TIM21_IRQn, 0, 0); 
	NVIC_EnableIRQ(TIM21_IRQn);
}


static void MX_TIM22_Init(void)
{

	
	__HAL_RCC_TIM22_CLK_ENABLE();
  htim22.Instance = TIM22;
  htim22.Init.Prescaler = 32-1;
  htim22.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim22.Init.Period = 10000;
  htim22.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim22) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	
	TIM22->PSC = 32 - 1; 
	TIM22->ARR = 100000 ; 
	TIM22->DIER |= TIM_DIER_UIE; 
	//TIM22->CR1 |= TIM_CR1_OPM;
	TIM22->CR1 |= TIM_CR1_CEN; 
	/*HAL_NVIC_SetPriority(TIM22_IRQn, 0, 0); 
	NVIC_EnableIRQ(TIM22_IRQn);*/
}


static void USART2_ReInit(uint8_t bd)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = USART_const[bd];
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}



int16_t mo3(int16_t a, int16_t b, int16_t c)
{
  int16_t middle;
  if ((a <= b) && (a <= c))
	{
		middle = (b <= c) ? b : c;
  }
  else
	{
		if ((b <= a) && (b <= c))
		{
      middle = (a <= c) ? a : c;
    }
    else
		{
      middle = (a <= b) ? a : b;
    }
  }

   return middle;
}

void Alarm10(void)
{
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	RTC_AlarmTypeDef sAlarm;

  HAL_RTC_GetDate(&hrtc, &sDate, FORMAT_BIN);
	HAL_RTC_GetTime(&hrtc, &sTime, FORMAT_BIN);
	if(sTime.Minutes>49)
	{
		if(sTime.Hours<23)
		{
			sAlarm.AlarmTime.Hours = sTime.Hours+1;
		}
		else
		{
			sAlarm.AlarmTime.Hours = 0;
		}
			
		sAlarm.AlarmTime.Minutes = sTime.Minutes-50;
	}
	else
	{
		sAlarm.AlarmTime.Hours = sTime.Hours;
		sAlarm.AlarmTime.Minutes = sTime.Minutes;
	}
  sAlarm.AlarmTime.Seconds = sTime.Seconds;
  sAlarm.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_SUB1H;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, FORMAT_BIN);
}




uint16_t Rnd1(uint16_t RR)
{
	uint16_t AA;
	AA=RR%100;
	if(AA>=75)
	{
		return RR+100-AA;
	}
	else if(AA<=25)
	{
		return RR-AA;
	}
	else 
	{
		return RR-AA+50;
	} 
}




void LampSet(uint8_t type,uint8_t dbl)
{
	lamp=type;
	lamp_double=dbl;
	lamp_cnt=0;
}

void LampSetA(uint8_t err)
{
	if(err==0) LampSet(Lamp_5,0);
	//else if(err|LmpCalEr) LampSet(Lamp_1,1);
	//else if(err|LmpCalib) LampSet(Lamp_1,0);
	else if(err|LmpOverr) LampSet(Lamp_05,0);
	else if(err|LmpCnnct) LampSet(Lamp_2,1);
	else if(err|Lmp10Min) LampSet(Lamp_2,0);
}

void LampSet1(uint8_t type,uint8_t dbl)
{
	switch (lamp)
	{
		case Lamp_5:
		{
			lamp=type;
			lamp_double=dbl;
			lamp_cnt=0;
			break;
		}
		case Lamp_2:
		{
			lamp=type;
			lamp_double=dbl;
			lamp_cnt=0;
			break;
		}
		case Lamp_1:
		{
			if(type!=Lamp_2)
			{
				lamp=type;
				lamp_double=dbl;
				lamp_cnt=0;
			}
			break;
		}
		case Lamp_05:
		{
			if(type==Lamp_5)
			{
				lamp=type;
				lamp_double=dbl;
				lamp_cnt=0;
			}
			break;
		}
	}
}

void Erase_Flash(void)
{
	uint32_t PgError = 0;
	HAL_FLASH_Unlock();
	
	FLASH_EraseInitTypeDef Flash_eraseInitStruct;
	Flash_eraseInitStruct.TypeErase     = FLASH_TYPEERASE_PAGES;
	Flash_eraseInitStruct.PageAddress  = FlAdr;
	Flash_eraseInitStruct.NbPages        = 2;

	if(HAL_FLASHEx_Erase(&Flash_eraseInitStruct, &PgError) != HAL_OK)
	{
		 HAL_FLASH_Lock();
	}
	HAL_FLASH_Lock();
}

uint32_t check_calc(uint32_t *buffer, uint32_t buff_len) {
	uint32_t result, i;
	result=0;
	for(i = 0; i < buff_len; i++) {
    result ^= buffer[i]; 
  }
	return result;
}

void Write_Flash_Adr(uint32_t Adr)
{
	uint32_t Buf[10];
	uint32_t PgError = 0;
	Buf[0]=(uint32_t)(MB_ADR*0x10000+MB_SPEED);
	Buf[1]=(uint32_t)(MB_RMS_N_I*0x10000+MB_RMS_N_F);
	Buf[2]=(uint32_t)(MB_RMS_O_I*0x10000+MB_RMS_O_F);
	//Buf[3]=(uint32_t)(MB_AMPL_ZERO*0x10000+MB_RMS_ZERO);
	Buf[3]=(uint32_t)(MB_HZ_I*0x10000+MB_HZ_F);
	Buf[4]=(uint32_t)(MB_AMPL_N_I*0x10000+MB_AMPL_N_F);
	Buf[5]=(uint32_t)(MB_AMPL_O_I*0x10000+MB_AMPL_O_F);
	Buf[6]=(uint32_t)(MB_AMPL_ZERO*0x10000+MB_RMS_ZERO);
	Buf[7]=(uint32_t)(MB_NAME);
	Buf[8]=(uint32_t)(MB_ATT_OFF*0x10000+MB_ATT_ON);
	Buf[9]=check_calc(Buf,9);
	HAL_FLASH_Unlock();
	
	FLASH_EraseInitTypeDef Flash_eraseInitStruct;
	Flash_eraseInitStruct.TypeErase     = FLASH_TYPEERASE_PAGES;
	Flash_eraseInitStruct.PageAddress  = Adr;
	Flash_eraseInitStruct.NbPages        = 1;

	if(HAL_FLASHEx_Erase(&Flash_eraseInitStruct, &PgError) != HAL_OK)
	{
		 HAL_FLASH_Lock();
	}

	HAL_FLASH_Program(TYPEPROGRAM_WORD, Adr,Buf[0]);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, Adr+4,Buf[1]);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, Adr+8,Buf[2]);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, Adr+12,Buf[3]);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, Adr+16,Buf[4]);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, Adr+20,Buf[5]);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, Adr+24,Buf[6]);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, Adr+28,Buf[7]);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, Adr+32,Buf[8]);
	HAL_FLASH_Program(TYPEPROGRAM_WORD, Adr+36,Buf[9]);

	HAL_FLASH_Lock();
}

void Write_Flash(void)
{
	Write_Flash_Adr(FlAdr);
	Write_Flash_Adr(FlAdr+0x100);
	Write_Flash_Adr(FlAdr+0x200);
}


uint32_t FLASH_Read(uint32_t address)
{
	return (*(__IO uint32_t*)address);
}

uint16_t IntToBCD (uint32_t d)
{
	uint8_t st=0;
	uint8_t t1,t2,t3;
	uint16_t tt;
	while(d/1000>0)
	{
		d=d/10;
		st++;
	}
	tt=d;
	t1=tt%10;
	t2=tt/10%10;
	t3=tt/100;
	return t3*0x1000+t2*0x100+t1*0x10+st;
}

uint16_t BCDToInt (uint16_t d)
{
	uint8_t t1,t2,t3;
	d=d>>4;
	t1=d&0x000F;
	t2=(d>>4)&0x000F;
	t3=(d>>8)&0x000F;
	return t1+t2*10+t3*100;
}

uint8_t notmore100(uint32_t data, uint32_t prev_data)
{
	if(prev_data > 100 && data > 100)
	{
		if(data>prev_data+100||data<prev_data-100)
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}
	else
	{
		return 0;
	}
}

void MB04(void) //поменяны местами регистры
{

	
	float rms_f=0;
	float max_f=0;

	
	uint32_t MB_RMS_T=0;
	uint32_t MB_RMSMAX_T=0;
	uint32_t MB_HZ_T=0;
	uint8_t Max_Max_Att=0;
	
	Max_Max_Att=Max_Max>>12;
	Max_Max&=0x0FFF;
	
	
	ampl=Max_Max-MB_AMPL_ZERO;
	if(ampl<0){ampl=0.0;}
	
	
	
	if(Max_Max_Att)
	{
		union1.int1[1]=MB_AMPL_O_F; //1
		union1.int1[0]=MB_RMS_O_F;

		max_f=union1.float1;
			
		MB_RMSMAX_T_fl=((float)ampl*max_f/(float)MB_AMPL_O_I);
		MB_RMSMAX_T=(int)(MB_RMSMAX_T_fl*100000.0);
	}
	else
	{
		union1.int1[1]=MB_AMPL_N_F; //2
		union1.int1[0]=MB_RMS_N_F;
		max_f=union1.float1;

		MB_RMSMAX_T_fl=(float)ampl*max_f/(float)MB_AMPL_N_I;
		MB_RMSMAX_T=(int)(MB_RMSMAX_T_fl*100000.0);		
	}
	
	if(FlagA90)
	{
		rms=MaxS-MB_AMPL_ZERO;
		if(rms<0){rms=0.0;}

		if(A90)
		{
			union1.int1[1]=MB_AMPL_O_F; //3
			union1.int1[0]=MB_RMS_O_F;
			rms_f=union1.float1;
			
			MB_RMS_T_fl=((float)rms*rms_f/(float)MB_AMPL_O_I);
			MB_RMS_T=(int)(MB_RMS_T_fl*100000.0);
		}
		else
		{
			union1.int1[1]=MB_AMPL_N_F; //4
			union1.int1[0]=MB_RMS_N_F;
			rms_f=union1.float1;

			MB_RMS_T_fl=((float)rms*rms_f/(float)MB_AMPL_N_I);
			MB_RMS_T=(int)(MB_RMS_T_fl*100000.0);
		}	
	}
	else
	{
		rms=RMSS-MB_RMS_ZERO;
		if(rms<0){rms=0.0;}


		if(A90)
		{
			union1.int1[1]=MB_AMPL_O_F; //5
			union1.int1[0]=MB_RMS_O_F;
			rms_f=union1.float1;
			MB_RMS_T_fl=((float)rms*rms_f/(float)MB_RMS_O_I);
			MB_RMS_T=(int)(MB_RMS_T_fl*100000.0);
		}
		else
		{
			union1.int1[1]=MB_AMPL_N_F; //6
			union1.int1[0]=MB_RMS_N_F;
			rms_f=union1.float1;
			MB_RMS_T_fl=(float)rms*rms_f/(float)MB_RMS_N_I;
			MB_RMS_T=(int)(MB_RMS_T_fl*100000.0);
		}	
	}
	

	
	MB_RMS=IntToBCD(MB_RMS_T);

	MB_RMSMAX=IntToBCD(MB_RMSMAX_T);

	if(Dooot>0&&MB_RMS>0)
	{
		hz_f=BCDToInt(MB_HZ_F);
		hz_st=MB_HZ_F&0x000F;
		hz_f2=BCDToInt(MB_HZ_I);
		hz_st2=MB_HZ_I&0x000F;
		
		

		//MB_HZ_T=(int)((22470000.0/(float)Dooot)*(tens[hz_st]/tens[hz_st2])*((float)hz_f/(float)hz_f2));
		MB_HZ_1=(int)(22470000.0/(float)Dooot);
		
		
		//MB_HZ_T=MB_HZ_1*(tens[hz_st]/tens[hz_st2])*((float)hz_f/(float)hz_f2);
		
		MB_HZ_T_fl=MB_HZ_1*(tens[hz_st]/tens[hz_st2])*((float)hz_f/(float)hz_f2);
		MB_HZ_T=(int)MB_HZ_T_fl;
		
		MB_HZ_1=IntToBCD(MB_HZ_1);
		MB_HZ=IntToBCD(MB_HZ_T);
	}
	else
	{
		MB_HZ=0;
		MB_HZ_T_fl=0.0;
	}
	
	/*MB_RMS_T_fl/=100000.0;
	MB_RMSMAX_T_fl/=100000.0;*/
	MB_HZ_T_fl/=100000.0;
	
	union1.float1=MB_RMS_T_fl;
	reg_MB3[0]=union1.int1[0];
	reg_MB3[1]=union1.int1[1];

	union1.float1=MB_RMSMAX_T_fl;
	reg_MB3[2]=union1.int1[0];
	reg_MB3[3]=union1.int1[1];
	
	union1.float1=MB_HZ_T_fl;
	reg_MB3[4]=union1.int1[0];
	reg_MB3[5]=union1.int1[1];
	
	Max_Max=0;	
}


void Flash_Jump_Adress(uint32_t adress)															// Переход в область другой программы во флеш памяти
{
	__set_PRIMASK(1);																							// Отключаем глобальные прерывания(обязательно перед переходом)																						 					
	
	typedef 	void (*pFunction)(void);														// Объявляем тип функции-ссылки
	pFunction Jump_To_Application;																// Объявляем функцию-ссылку

  uint32_t JumpAddress = *(__IO uint32_t*) (adress + 4); 						// Адрес перехода на вектор (Reset_Handler) 		
  Jump_To_Application = (pFunction) JumpAddress;  							// Указатель на функцию перехода
	__set_MSP(*(volatile uint32_t*) adress);														// Указываем адрес вектора стека(Stack Pointer)	
  Jump_To_Application();                          							// Переходим на основную программу
}	

void My_System_Reset(void)
{
	if(reset_flag)// Регистр - флаг сделать ресет контроллеру
	{
		HAL_NVIC_SystemReset();
	}
}

void My_Jump_Boatloader(void)
{
	if(bl_flag)																	// Проверяем регистр перехода в загрузчик
	{
		// Останавливаем преобразования АЦП и отключаем его
		HAL_ADC_Stop_DMA(&hadc);
		
		// Останавливаем все задействованные таймеры
		HAL_TIM_Base_Stop_IT(&htim2);					 										// остановить таймер 2
		HAL_TIM_Base_Stop_IT(&htim21);					 			
		HAL_TIM_Base_Stop_IT(&htim22);					 							 			
		
		// Отключаем все прерывания
		HAL_NVIC_DisableIRQ(TIM2_IRQn);				  									// отключить прерывания таймера 2
		HAL_NVIC_DisableIRQ(TIM21_IRQn);				  	
		HAL_NVIC_DisableIRQ(TIM22_IRQn);				  									
		HAL_NVIC_DisableIRQ(USART2_IRQn);		 		  								// отключить прерывания USART 2		  			 		  			

		HAL_NVIC_DisableIRQ(RCC_IRQn);				  									// отключить прерывания Осцилятора
		HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
	
		
		// Деинициализация портов
		HAL_GPIO_DeInit(GPIOA, 0xff);
		HAL_GPIO_DeInit(GPIOB, 0xff);
		HAL_GPIO_DeInit(GPIOC, 0xff);

		
		// Деинициализация таймеров
		HAL_TIM_Base_DeInit(&htim2);
		HAL_TIM_Base_DeInit(&htim21);
		HAL_TIM_Base_DeInit(&htim22);
		
		// Деинициализация задействованного АЦП_1
		HAL_ADC_DeInit(&hadc);
		
		// Деинициализация приемо-передатчиков
		HAL_UART_DeInit(&huart2);
		HAL_RCC_DeInit();
		HAL_DeInit();
		
		__HAL_RCC_DMA1_CLK_DISABLE();
		
		// Выключаем тактирование портов GPIO
		//__GPIOA_CLK_DISABLE();
		//__GPIOB_CLK_DISABLE();
		//__GPIOC_CLK_DISABLE();
		//__GPIOD_CLK_DISABLE();
		
		Flash_Jump_Adress(0x08000000);														// Переходим в область памяти загрузчика
	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	__set_PRIMASK(1);									  											// Отключаем глобальные прерывания	
	SCB->VTOR = (uint32_t)0x08002800;  												// Переопределяем начало таблицы векторов прерываний 
	__set_PRIMASK(0);
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC_Init();
  //MX_TIM2_Init();
  MX_TIM21_Init();
  MX_RTC_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	

	
	MB_AMPL_ZERO=0;
	MB_RMS_ZERO=0;

	MB_RMS_N_I=0x1000;
	MB_RMS_N_F=0x1001;
	MB_RMS_O_I=0x1000;
	MB_RMS_O_F=0x1002;
	
	MB_AMPL_N_I=0x1000;
	MB_AMPL_N_F=0x1001;
	MB_AMPL_O_I=0x1000;
	MB_AMPL_O_F=0x1002;
	
	MB_ADR=247;
	MB_SPEED=2;
	//reg_MB[17]=10;
	
	MB_HZ_I=0x5001;
	MB_HZ_F=0x5001;
	
	MB_NAME=0;
	
	MB_ATT_ON=0x0FA0;
	MB_ATT_OFF=0x00FA;
	
	FBI[0][0]=FLASH_Read(FlAdr);
	uint8_t k, rightData;
	
	
	if((FBI[0][0]==0)||(FBI[0][0]==0xFFFFFFFF))
	{
		rightData=4; //wrong data - need write default
		Write_Flash();
	}
	else 
	{
		for(k=0;k<10;k++)
		{
			FBI[0][k]=FLASH_Read(FlAdr + k*4);
			FBI[1][k]=FLASH_Read(FlAdr + 0x100 + k*4);
			FBI[2][k]=FLASH_Read(FlAdr + 0x200 + k*4);
		}
		
		uint32_t crc32[3];
		for(k=0;k<3;k++)
		{
			crc32[k]=check_calc(FBI[k],9);
		}
		if (crc32[0]==FBI[0][9] && crc32[1]==FBI[1][9] && FBI[0][9]==FBI[1][9])
		{
			rightData=0;
		}
		else if (crc32[0]==FBI[0][9] && crc32[2]==FBI[2][9] && FBI[0][9]==FBI[2][9])
		{
			rightData=0;
		}
		else if (crc32[1]==FBI[1][9] && crc32[2]==FBI[2][9] && FBI[1][9]==FBI[2][9])
		{
			rightData=1;
		}
		else
		{
			rightData=4; //wrong data - need write default
			Write_Flash();		
		}
	}
		
	if(rightData<3)
	{
		MB_ADR=(uint8_t)(FBI[rightData][0]>>16);
		MB_SPEED=FBI[rightData][0]&0x0000FFFF;
		
		MB_RMS_N_I=FBI[rightData][1]>>16;
		MB_RMS_N_F=FBI[rightData][1]&0x0000FFFF;
		
		MB_RMS_O_I=FBI[rightData][2]>>16;
		MB_RMS_O_F=FBI[rightData][2]&0x0000FFFF;
		
		MB_AMPL_ZERO=FBI[rightData][6]>>16;
		MB_RMS_ZERO=FBI[rightData][6]&0x0000FFFF;
		
		MB_HZ_I=FBI[rightData][3]>>16;
		MB_HZ_F=FBI[rightData][3]&0x0000FFFF;
		
		MB_AMPL_N_I=FBI[rightData][4]>>16;
		MB_AMPL_N_F=FBI[rightData][4]&0x0000FFFF;
		
		MB_AMPL_O_I=FBI[rightData][5]>>16;
		MB_AMPL_O_F=FBI[rightData][5]&0x0000FFFF;
		
		MB_NAME=FBI[rightData][7]&0x0000FFFF;
		
		MB_ATT_OFF=FBI[rightData][8]>>16;
		MB_ATT_ON=FBI[rightData][8]&0x0000FFFF;
	}
	
	if(MB_HZ_I==0)
	{
		MB_HZ_I=8001;
		MB_HZ_F=8001;
		Write_Flash();
	}
	
	
	//MB_HZ_I=FBI[5]>>16;
	//MB_HZ_F=FBI[5]&0x0000FFFF;
	
	
	NVIC_SetPriority(USART2_IRQn, 0); 
  NVIC_EnableIRQ(USART2_IRQn);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE); 
	
	
	//MX_TIM21_Init();
	if(MB_SPEED>8){MB_SPEED=2;}
	USART2_ReInit(MB_SPEED);
	MX_TIM2_Init(MB_SPEED);


	HAL_ADC_Start_DMA(&hadc,(uint32_t*)&adc1,3);

	HAL_NVIC_SetPriority(RTC_IRQn, 0, 1); 

	HAL_NVIC_EnableIRQ(RTC_IRQn); 

	lamp=2;

	MBPause=1200;
	lamp_err=0;
	LampSetA(lamp_err);
	Flag=0;
  while (1)
  {

		HAL_IWDG_Refresh(&hiwdg);
		
		
		if(mbReinitCnt>120)
		{
			mbReinitCnt=0;
			//usart reinit
			if(MB_SPEED>8){MB_SPEED=2;}
			USART2_ReInit(MB_SPEED);
			MX_TIM2_Init(MB_SPEED);
			HAL_NVIC_SetPriority(USART2_IRQn, 0, 1);
			HAL_NVIC_EnableIRQ(USART2_IRQn);
			__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
		}
		
		My_Jump_Boatloader();
		Dot++;
		
		//if((Dot>PrevDot4-100&&Dot<PrevDot4+100)||(Dot>PrevDot42-100&&Dot<PrevDot42+100))
		//{
			now_vref=adc1[2];
			now1=adc1[1]; //ampl
			now2=adc1[0]; //rms
		//}
		/*now_vref=adc1[2];
		now1=adc1[1]; //ampl
		now2=adc1[0]; //rms*/
		

		if(Flag)
		{
			Flag=0;
			
			if(FlagMB)
			{
				FlagMB=0;
				Dot=0;
				FlagMB2=1;
			}
			else if(Dot<MAX_HZ)
			{
				PrevDot=Dot;
				OHBOY++;
				PrevDot4=Dot/4;
				PrevDot4=PrevDot4+Dot/2;
				Dot=0;
				HAL_Delay(2);
				//MaxS=0;
				//Max_Max=0;
				//Dooot=0;
				//RMSS=0;
			}
			/*else if(Dot<5)
			{
				
			}*/
			else //if(notmore100(Dot,PrevDot))
			{
				if(max<MB_ATT_OFF&&A90&&AttPAftF==0&&FORCE_ATT==0)
				{
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
					A90=0;
					FlagA90=1;
				}
				/*if(max>Max_Max)
				{
					Max_Max=max;
				}*/
				RMSS=now2;
				if(!FlagMB2)
				{
					MB_RMS_NOW=now2;
					MB_AMPL_NOW=max;
					if(A90)
					{
						MB_RMS_NOW|=0x1000;
						MB_AMPL_NOW|=0x1000;
					}
				}
				if(MB_AMPL_NOW>Max_Max)
				{
					Max_Max=MB_AMPL_NOW;
				}
				if(now2>Max_RMSs)
				{
					Max_RMSs=now2;
				}
				MaxS=max;
				
				
				
				
				
				if(max>0x0FFE)
				{
					lamp_f=1;
					lamp_err|=LmpOverr;
				}
				else if((lamp_err&LmpOverr)>0)
				{
					lamp_f=1;
					lamp_err&=~LmpOverr;
				}
				max=0;
				if(FlagMB2)
				{
					FlagMB2=0;
				}			
				else
				{
					Dooot=Dot;
					MB_HZ_NOW=Dot;
				}	
				if(Max_Max<0x0F)
				{
					Dooot=0;
				}
				
				Dot=0;
			}	
			/*else
			{
				max=0;
					
				PrevDot=Dot;
				PrevDot4=Dot/4;
				PrevDot4=PrevDot4+Dot/2;
				Dot=0;
				Dooot=0;
				RMSS=0;
			}*/
		}	
		if(now1>max)
		{
			max=now1;
		}
		if((now1>MB_ATT_ON||FORCE_ATT>0)&&A90==0)
		{
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
			A90=1;
			FlagA90=1;
			AttPAftF=1;
			//AttPF=1;
		}

		if(Dot>30000)
		{
			

			
			/*MB_RMS_NOW=now2;
			MB_AMPL_NOW=max;
			max=0;*/
			MB_RMS_NOW=now2;
			MB_AMPL_NOW=max;

			if(max<MB_ATT_OFF&&A90&&AttPAftF==0&&FORCE_ATT==0)
			{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
				A90=0;
				FlagA90=1;
			}
			//RMSS=now2;
			//MaxS=max;
			
			if(A90)
			{
				MB_RMS_NOW|=0x1000;
				MB_AMPL_NOW|=0x1000;
			}
			max=0;
			Dot=0;
			MB_HZ_NOW=Dot;//clear after test

			
			Dooot=0;
			RMSS=0; //вернуть обнуление
			//RMSS=now2;//стереть 
			MaxS=0;
			Max_RMSs=0;
			//MB04();
			Max_Max=0;
		}


  /* USER CODE END WHILE */

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;

  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	
	sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2){
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 288; //(37KHz / 128) - 1 = 288 
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);
  }

}


/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }


}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  //HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	/*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/* USER CODE BEGIN 4 */

void EXTI4_15_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
	if(Dot>100)
	{
		Flag=1;
	}
}

void TIM22_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim22);
	TIM22->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));

}


void USART2_IRQHandler(void)
{
	if((USART2->ISR & USART_ISR_RXNE) == USART_ISR_RXNE)
	{	
		
			TIM2->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
			TIM2->CNT=0;
			res_buffer[res_wr_index]=(uint8_t)(USART2->RDR);
			//HAL_UART_Receive(&huart2, &x, 1, 100);
			if(res_wr_index<29)
			{
				res_wr_index++;			
			}
			FlagMB=1;
			TIM2->CR1 |= TIM_CR1_CEN; 
	}
	HAL_UART_IRQHandler(&huart2);
}





void TIM2_IRQHandler(void)
{
	TIM2->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
	HAL_TIM_IRQHandler(&htim2);

	FlagMB=1;

	uint16_t ind,dt;
	
	uint8_t needFlashWrite=0;
	
	uint8_t snd_cnt=0;
	int i;
	if (res_buffer[0]==MB_ADR||res_buffer[0]==247)

  {
		
		mbReinitCnt=0;
		
	  CRCCod=CRC16(res_buffer, (res_wr_index));	// Расчет СRC
	  if (CRCCod==0)								// Проверка CRC в посылке
	  {											// Если верно - выполняем действие
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);
			MBPauseCnt=0;
			if(lamp_err&Lmp10Min)
			{
				lamp_f=1;
				lamp_err&=~Lmp10Min;
			}
		  switch (res_buffer[1]) {
		  case 0x03:							// Чтение регистров
		  {
			  if (res_buffer[0]==247&&(res_buffer[2]<1) && ((res_buffer[3]+res_buffer[4]*256+res_buffer[5]-1)<30))
			  {
					mb_err=0;
				  write_buffer[0]=res_buffer[0];					// Адрес устройства
				  write_buffer[1]=0x03;						// Та-же функция
				  write_buffer[2]=res_buffer[5]*2;			// Счетчик байт

				  for (i=0; i<res_buffer[5]; i++)				// Значения регистров
				  {
					  write_buffer[4+(2*i)]=(reg_MB[res_buffer[2]*0x100+res_buffer[3]+i])& 0x00FF;//%256;		// Младший байт (2-ой)
					  write_buffer[3+(2*i)]=(reg_MB[res_buffer[2]*0x100+res_buffer[3]+i])>> 8;///256;	// Старший байт (1-ый)
				  }		
					snd_cnt=write_buffer[2]+3;
			  }
				else if ((res_buffer[2]<1) && ((res_buffer[3]+res_buffer[4]*256+res_buffer[5]-12)<3))
			  {
					mb_err=0;
				  write_buffer[0]=res_buffer[0];					// Адрес устройства
				  write_buffer[1]=0x03;						// Та-же функция
				  write_buffer[2]=res_buffer[5]*2;			// Счетчик байт

				  for (i=0; i<res_buffer[5]; i++)				// Значения регистров
				  {
					  write_buffer[4+(2*i)]=(reg_MB[res_buffer[2]*0x100+res_buffer[3]+i])& 0x00FF;//%256;		// Младший байт (2-ой)
					  write_buffer[3+(2*i)]=(reg_MB[res_buffer[2]*0x100+res_buffer[3]+i])>> 8;///256;	// Старший байт (1-ый)
				  }		
					snd_cnt=write_buffer[2]+3;
			  }
			  else
			  {
					mb_err=1;
				  write_buffer[0]=res_buffer[0];					// адрес блока
				  write_buffer[1]=0x83;						// та-же функция + взведенный бит ошибки
				  write_buffer[2]=0x02;				// код ошибки - недопустимый адрес
					snd_cnt=3;
			  }
			  break;
		  }
			case 0x04:							// Чтение регистров
		  {
			  if ((res_buffer[2]==1) && ((res_buffer[3]+res_buffer[4]*256+res_buffer[5]-1)<8))
			  {
					mb_err=0;
					//ampl=MaxS;
					MB04();

				  write_buffer[0]=res_buffer[0];					// Адрес устройства
				  write_buffer[1]=0x04;						// Та-же функция
				  write_buffer[2]=res_buffer[5]*2;			// Счетчик байт

				  for (i=0; i<res_buffer[5]; i++)				// Значения регистров
				  {
					  write_buffer[4+(2*i)]=(reg_MB2[res_buffer[2]*0x100+res_buffer[3]+i-256])%256;		// Младший байт (2-ой)
					  write_buffer[3+(2*i)]=(reg_MB2[res_buffer[2]*0x100+res_buffer[3]+i-256])/256;	// Старший байт (1-ый)
				  }


					
					snd_cnt=write_buffer[2]+3;
			  }
				else if ((res_buffer[2]==2) && ((res_buffer[3]+res_buffer[4]*256+res_buffer[5]-1)<8))
			  {
					mb_err=0;
					//ampl=MaxS;
					MB04();

				  write_buffer[0]=res_buffer[0];					// Адрес устройства
				  write_buffer[1]=0x04;						// Та-же функция
				  write_buffer[2]=res_buffer[5]*2;			// Счетчик байт

				  for (i=0; i<res_buffer[5]; i++)				// Значения регистров
				  {
					  write_buffer[4+(2*i)]=(reg_MB3[res_buffer[2]*0x100+res_buffer[3]+i-512])%256;		// Младший байт (2-ой)
					  write_buffer[3+(2*i)]=(reg_MB3[res_buffer[2]*0x100+res_buffer[3]+i-512])/256;	// Старший байт (1-ый)
				  }
					
					snd_cnt=write_buffer[2]+3;

			  }
			  else
			  {
					mb_err=1;
				  write_buffer[0]=res_buffer[0];					// адрес блока
				  write_buffer[1]=0x84;						// та-же функция + взведенный бит ошибки
				  write_buffer[2]=0x02;				// код ошибки - недопустимый адрес
					snd_cnt=3;
					

			  }
			  break;
		  }
		  case 0x06:						//запись регистра
		  {
		  		if ((res_buffer[2]*0x100+res_buffer[3]>11)&&(res_buffer[2]*0x100+res_buffer[3]<14))//если возможна запись регистра
		  		{
						mb_err=0;
						//uint16_t ind,dt;
		  			write_buffer[0]=res_buffer[0];					// адрес блока
		  			write_buffer[1]=0x06;						// та-же функция
		  			write_buffer[2]=res_buffer[2];				// те же данные
		  			write_buffer[3]=res_buffer[3];
		  			write_buffer[4]=res_buffer[4];
		  			write_buffer[5]=res_buffer[5];
						snd_cnt=6;
						ind=res_buffer[2]*0x100+res_buffer[3];
						dt=res_buffer[4]*0x100+res_buffer[5];					
						if(ind==13)
						{
							if(dt>8){dt=8;}
							NeedChangeSpeed=1;
							//USART2_ReInit(dt);
							//MX_TIM2_Init(dt);
						}
						reg_MB[ind]=dt;	
						needFlashWrite=1;
						//Write_Flash();
		  		}
					else if(res_buffer[0]==247&&(((res_buffer[2]*0x100+res_buffer[3]>3)&&(res_buffer[2]*0x100+res_buffer[3]<21))||
						(res_buffer[2]*0x100+res_buffer[3]==0)))
					{
						
						mb_err=0;
						MB04();
						
						

						reg_MB[res_buffer[3]]=res_buffer[4]*0x100+res_buffer[5];
						
						
						if(res_buffer[3]==7||res_buffer[3]==9)
						{
							reg_MB[res_buffer[3]+8]=(MB_RMS_NOW&0x0FFF)-MB_RMS_ZERO;
							reg_MB[res_buffer[3]+7]=(MB_AMPL_NOW&0x0FFF)-MB_AMPL_ZERO;
							//reg_MB[res_buffer[3]-1]=res_buffer[4]*0x100+res_buffer[5];
						}
						/*else if(res_buffer[3]==9)
						{
							reg_MB[res_buffer[3]+8]=MB_RMS_NOW&0x0FFF;
							reg_MB[res_buffer[3]+7]=MB_AMPL_NOW&0x0FFF;
							reg_MB[res_buffer[3]-1]=res_buffer[4]*0x100+res_buffer[5];
						}*/
						else if(res_buffer[3]==10)
						{
							reg_MB[res_buffer[3]+8]=MB_HZ_1;
						}
						else if(res_buffer[3]==5)
						{
							MB_AMPL_ZERO=MB_AMPL_NOW&0x0FFF;
							MB_RMS_ZERO=MB_RMS_NOW&0x0FFF;
						}
						else if(res_buffer[3]==0)
						{
							MB_AMPL_ZERO=0;
							MB_RMS_ZERO=0;
						}
						write_buffer[0]=res_buffer[0];					// адрес блока
						write_buffer[1]=0x06;						// та-же функция
						write_buffer[2]=res_buffer[2];				// те же данные
						write_buffer[3]=res_buffer[3];
						write_buffer[4]=res_buffer[4];
						write_buffer[5]=res_buffer[5];
						snd_cnt=6;
						needFlashWrite=1;
						//Write_Flash();
						

						
					}
					else if(res_buffer[0]==247&&res_buffer[2]==0x00&&res_buffer[3]==0x50&&res_buffer[4]==0x50&&res_buffer[5]==0x50)
					{
						write_buffer[0]=res_buffer[0];					// адрес блока
						write_buffer[1]=0x06;						// та-же функция
						write_buffer[2]=res_buffer[2];				// те же данные
						write_buffer[3]=res_buffer[3];
						write_buffer[4]=res_buffer[4];
						write_buffer[5]=res_buffer[5];
						snd_cnt=6;

						
						FORCE_ATT = 0x01;
					}
					
					else if(res_buffer[0]==247&&res_buffer[2]==0x00&&res_buffer[3]==0x50&&res_buffer[4]==0xA0&&res_buffer[5]==0xA0)
					{
						write_buffer[0]=res_buffer[0];					// адрес блока
						write_buffer[1]=0x06;						// та-же функция
						write_buffer[2]=res_buffer[2];				// те же данные
						write_buffer[3]=res_buffer[3];
						write_buffer[4]=res_buffer[4];
						write_buffer[5]=res_buffer[5];
						snd_cnt=6;

						
						FORCE_ATT = 0x00;
					}			
					else if(res_buffer[0]==247&&res_buffer[2]==0x55&&res_buffer[3]==0x55&&res_buffer[4]==0x55&&res_buffer[5]==0x55)
					{
						write_buffer[0]=res_buffer[0];					// адрес блока
						write_buffer[1]=0x06;						// та-же функция
						write_buffer[2]=res_buffer[2];				// те же данные
						write_buffer[3]=res_buffer[3];
						write_buffer[4]=res_buffer[4];
						write_buffer[5]=res_buffer[5];
						snd_cnt=6;

						
						bl_flag = 1;
					}	
					else if(res_buffer[0]==247&&res_buffer[2]==0xFF&&res_buffer[3]==0xFF&&res_buffer[4]==0xFF&&res_buffer[5]==0xFF)
					{
						write_buffer[0]=res_buffer[0];					// адрес блока
						write_buffer[1]=0x06;						// та-же функция
						write_buffer[2]=res_buffer[2];				// те же данные
						write_buffer[3]=res_buffer[3];
						write_buffer[4]=res_buffer[4];
						write_buffer[5]=res_buffer[5];
						snd_cnt=6;						
						Erase_Flash();
					}	
					
		  		else
		  		{
						mb_err=1;
		  			write_buffer[0]=res_buffer[0];					// адрес устройства
		  			write_buffer[1]=0x86;						// та-же функция
		  			write_buffer[2]=0x02;				// код ошибки - недопустимый адрес
						snd_cnt=3;



		  		}

		  		  	break;
		  }
			case 0x10:
			{
				if(res_buffer[0]==247&&res_buffer[2]==0&&(res_buffer[3]==6||res_buffer[3]==8)&&res_buffer[4]==0&&res_buffer[5]==2
					&&res_buffer[6]==4)
				{
					write_buffer[0]=res_buffer[0];					// Адрес устройства
					write_buffer[1]=res_buffer[1];						// Та-же функция
					write_buffer[2]=res_buffer[2];		
					write_buffer[3]=res_buffer[3];
					write_buffer[4]=res_buffer[4];
					write_buffer[5]=res_buffer[5];
					reg_MB[res_buffer[3]+9]=(MB_RMS_NOW&0x0FFF)-MB_RMS_ZERO;
					reg_MB[res_buffer[3]+8]=(MB_AMPL_NOW&0x0FFF)-MB_AMPL_ZERO;
					reg_MB[res_buffer[3]]=res_buffer[7]*0x100+res_buffer[8];
					reg_MB[res_buffer[3]+1]=res_buffer[9]*0x100+res_buffer[10];
					snd_cnt=6;
					needFlashWrite=1;
					//Write_Flash();
				}
				else
				{
					mb_err=1;
					write_buffer[0]=res_buffer[0];					// адрес устройства
					write_buffer[1]=0x90;						// та-же функция
					write_buffer[2]=0x02;				// код ошибки - недопустимый адрес
					snd_cnt=3;

				}	
				break;
			}
			/*case 0x42:  //завод
			{
				write_buffer[0]=res_buffer[0];					// адрес блока
				write_buffer[1]=res_buffer[1];						// та-же функция 
				CRCCod=CRC16(write_buffer, 2);				// расчет CRC
				write_buffer[2] = CRCCod & 0x00FF;			// мл. байт CRC
				write_buffer[3] = CRCCod >> 8;				// ст. байт CRC
				
				MB_AMPL_ZERO=0;
				MB_RMS_ZERO=0;

				MB_RMS_N_I=0x1000;
				MB_RMS_N_F=0x1001;
				MB_RMS_O_I=0x1000;
				MB_RMS_O_F=0x1002;
				
				MB_AMPL_N_I=0x1000;
				MB_AMPL_N_F=0x1001;
				MB_AMPL_O_I=0x1000;
				MB_AMPL_O_F=0x1002;
				
				MB_ADR=247;
				MB_SPEED=2;
				//reg_MB[17]=10;
				
				MB_HZ_I=0x5001;
				MB_HZ_F=0x5001;
				Write_Flash();
				
				HAL_UART_Transmit(&huart2,write_buffer,4,100);
				break;
			}*/
			case 0x45:
			{
				
				write_buffer[0]=res_buffer[0];					// адрес блока
				write_buffer[1]=res_buffer[1];						// та-же функция 
				write_buffer[2]=res_buffer[2];						// та-же функция 
				snd_cnt=3;

				if(res_buffer[0]==247)
				{
					FORCE_ATT = res_buffer[2];
				}
				
				//HAL_UART_Transmit(&huart2,write_buffer,5,100);
				break;
			}
			default:
			{
				mb_err=1;
				write_buffer[0]=res_buffer[0];					// адрес блока
				write_buffer[1]=res_buffer[1]+0x80;						// та-же функция + взведенный бит ошибки
				write_buffer[2]=0x01;				// код ошибки - недопустимая функция
				snd_cnt=3;


				break;
			}
	  }
			CRCCod=CRC16(write_buffer, snd_cnt);				// расчет CRC

			write_buffer[snd_cnt] = CRCCod & 0x00FF;			// мл. байт CRC
			write_buffer[snd_cnt+1] = CRCCod >> 8;				// ст. байт CRC
			HAL_UART_Transmit(&huart2,write_buffer,snd_cnt+2,100);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET);
		
			if(NeedChangeSpeed)
			{
				NeedChangeSpeed=0;
				USART2_ReInit(dt);
				MX_TIM2_Init(dt);
			}
			if(needFlashWrite)
			{
				needFlashWrite=0;
				Write_Flash();
			}
	  }



  }
//HAL_UART_Transmit(&huart2,res_buffer,res_wr_index,100);
	
	if(mb_err!=mb_err_prev)
	{
		mb_err_prev=mb_err;
		if(mb_err)
		{
			lamp_f=1;
			lamp_err|=LmpCnnct;
		}
		else
		{
			lamp_f=1;
			lamp_err&=~LmpCnnct;
		}
	}
  res_wr_index=0;
	 HAL_NVIC_SetPriority(USART2_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

}

void TIM21_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim21);
	/*	if(lamp_f)
		{
			
			//LampSetA(lamp_err);
			if(lamp_err==0) 
			{
				lamp=10;
				lamp_double=0;
				lamp_cnt=0;
			}
			else if(lamp_err>=0x10) 
			{
				lamp=1;
				lamp_double=0;
				lamp_cnt=0;
			}
			else if(lamp_err>=0x02) 
			{
				lamp=4;
				lamp_double=1;
				lamp_cnt=0;
			}
			else if(lamp_err==0x01) 
			{
				lamp=4;
				lamp_double=0;
				lamp_cnt=0;
			}
			lamp_f=0;
		}
	
	lamp_cnt++;
	if(lamp_cnt>=lamp)
	{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);
		if(lamp_double)
		{
			HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, TT1, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
		}
		else
		{
			HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0x02C3, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
		}
		lamp_cnt=0;
	}
	if((lamp_err&Lmp10Min)==0)
	{
		MBPauseCnt++;
		if(MBPauseCnt>MBPause)
		{
			lamp_f=1;
			lamp_err|=Lmp10Min;
		}
	}*/
	
	mbReinitCnt++;
	
	if(FlagA90)
	{
		AttP++;
		if(AttP>AttPause)
		{
			FlagA90=0;
			AttP=0;
		}
	}
	if(AttPAftF)
	{
		AttPAft++;
		if(AttPAft>AttPauseAfter)
		{
			AttPAftF=0;
			AttPAft=0;
		}
	}
}

/*void RTC_IRQHandler(void)
{
	HAL_RTCEx_WakeUpTimerIRQHandler(&hrtc);
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc){
	HAL_RTCEx_DeactivateWakeUpTimer(hrtc);
	if(lamp_double)
	{
		if(lamp_df==0)
		{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET);
			lamp_df=1;
			HAL_RTCEx_SetWakeUpTimer_IT(hrtc, TT2, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
		}
		else if(lamp_df==1)
		{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);
			lamp_df=2;
			HAL_RTCEx_SetWakeUpTimer_IT(hrtc, TT3, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
		}
		else if(lamp_df==2)
		{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET);
			lamp_df=0;
		}
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET);
	}

}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) 
{ 
	lamp=10;
}

void LPTIM1_IRQHandler(void) 
{ 
	HAL_LPTIM_IRQHandler(&LptimHandle); 
	HAL_LPTIM_Counter_Stop_IT(&LptimHandle);
	//HAL_LPTIM_OnePulse_Stop_IT(&LptimHandle);

}*/

void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim) 
{ 
	//HAL_NVIC_DisableIRQ(LPTIM1_IRQn);
			
	//HAL_NVIC_ClearPendingIRQ(LPTIM1_IRQn); 
	
	//HAL_NVIC_DisableIRQ(LPTIM1_IRQn);
	/*lamp=10;
	uint8_t i=0;
	Dot1=0;
	FlagMB=1;
	for(i=0;i<Kf;i++)
	{
		Dot1+=DotSum[i];
	}
	if(DotMax<Dot1)
	{
		DotMax=Dot1;
	}
	if(DotMin>Dot1)
	{
		DotMin=Dot1;
	}
	Dot12=Dot1;*/
}






/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
