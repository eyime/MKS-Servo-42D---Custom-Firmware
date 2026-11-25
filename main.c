#include <n32l40x.h>
#include <stdlib.h>
#include <stdio.h>

#include <math.h>

// configuration files
#include "n32l40x_it.h"

// foc modules
#include "foc_time.h"
#include "foc_motor_od.h"
#include "foc_motor.h"
#include "foc_adc.h"
#include "foc_pwm.h"
#include "foc_encoder.h"
#include "foc_flash.h"
#include "foc_comm.h"
#include "foc_oled.h"
#include "foc_utils.h"

#define UNUSED(x) (void)(x)

// Set Hardware Clock PLL to 64 MHz
void RCC_Configuration(void) {	
	RCC_DeInit();
	RCC_ConfigHclk(RCC_SYSCLK_DIV1);
	RCC_ConfigPclk2(RCC_HCLK_DIV2);
	RCC_ConfigPclk1(RCC_HCLK_DIV4);

	RCC_EnableHsi(ENABLE);
	// Wait till HSI is ready
	if ( RCC_WaitHsiStable()!= SUCCESS ) {
		while (true);
	};

	// Configure PLL
	RCC_ConfigPll( RCC_PLL_HSI_PRE_DIV2, RCC_PLL_MUL_16, RCC_PLLDIVCLK_ENABLE);
	// Enable PLL
	RCC_EnablePll(ENABLE);
	// Wait till PLL is ready
	while ( RCC_GetFlagStatus(RCC_CTRL_FLAG_PLLRDF) != SET );
	
	FLASH_ClockInit();
	// Disable Prefetch Buffer
	FLASH_PrefetchBufSet(FLASH_PrefetchBuf_DIS);
	// Enable iCache
	FLASH_iCacheCmd(FLASH_iCache_EN);
	// Flash wait state
	FLASH_SetLatency(FLASH_LATENCY_1);

	// selsect HSI as RCC ADC1M CLK Source
	RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV16);  
	RCC_ConfigAdcPllClk(RCC_ADCPLLCLK_DIV1,ENABLE);
	RCC_ConfigAdcHclk(RCC_ADCHCLK_DIV1);

	// config RNG clock
	RCC_ConfigTrng1mClk(RCC_TRNG1MCLK_SRC_HSI, RCC_TRNG1MCLK_DIV16);
	RCC_EnableTrng1mClk(ENABLE);
	RCC_ConfigRngcClk(RCC_RNGCCLK_SYSCLK_DIV1);

	// Select PLLCLK as system clock source
	RCC_ConfigSysclk(RCC_SYSCLK_SRC_PLLCLK);
	// Wait till PLLCLK is used as system clock source
	while (RCC_GetSysclkSrc() != 0x0c);
}


// Interrupts configuration
void NVIC_Configuration(void) {
	// variable for int configuration
	NVIC_InitType NVIC_InitStructure;
	// Preemption Priority from 0 to 16, Subpriority to 0
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

	// USART1 as COMM for 485
	NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// TIM2 at 10 Khz
	NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

// Pins Configuration
void GPIO_Configuration(void) {
	GPIO_InitType GPIO_InitStructure;

	GPIO_InitStruct(&GPIO_InitStructure);
	// Enable the GPIO clock
  // GPIOA, GPIOB, GPIOD, AFIO
  RCC_EnableAPB2PeriphClk( RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_GPIOD | RCC_APB2_PERIPH_AFIO, ENABLE);
 
	// Pins to be configured as digital outputs
  GPIO_SetBits( GPIOA, GPIO_PIN_11 );
  GPIO_SetBits( GPIOB, GPIO_PIN_6 );
	GPIO_SetBits( GPIOD, GPIO_PIN_15 | GPIO_PIN_0 );
 
	// INPUTS ---------------------------------------
	// GPIOA ---------
	/********PA0->PA0.Input********/
	/********PA8->PA8.Input********/
	/********PA12->PA12.Input********/
	/********PA15->PA15.Input********/
	// Initialize input GPIOA
	GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_Input;
	GPIO_InitStructure.GPIO_Pull      = GPIO_Pull_Up;
	GPIO_InitStructure.GPIO_Alternate = GPIO_NO_AF;
	GPIO_InitStructure.Pin            = GPIO_PIN_15 | GPIO_PIN_12 | GPIO_PIN_8 | GPIO_PIN_0;
	GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
	// GPIOB ---------	 
	/********PB7->PB7.Input********/
	GPIO_InitStructure.Pin            = GPIO_PIN_7;
	GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);     
	
	// OUTPUTS ---------------------------------------
	/********PA11->PA11.Output********/
	/********PB6->PB6.Output********/
	/********PD0->PD0.Output********/
	/********PD15->PD15.Output********/
	// GPIOD ---------
	GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pull      = GPIO_Pull_Up;
	GPIO_InitStructure.GPIO_Slew_Rate = GPIO_Slew_Rate_High;
	GPIO_InitStructure.GPIO_Current   = GPIO_DC_2mA;
	GPIO_InitStructure.GPIO_Alternate = GPIO_NO_AF;
	GPIO_InitStructure.Pin            = GPIO_PIN_15 | GPIO_PIN_0;
	GPIO_InitPeripheral(GPIOD, &GPIO_InitStructure);
	// GPIOA ---------
	GPIO_InitStructure.Pin            = GPIO_PIN_11;
	GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
	// GPIOB ---------
	GPIO_InitStructure.Pin            = GPIO_PIN_6;
	GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
	
	// ANALOG INPUTS ----------------------------------------
	/********PA1->ADC.IN2********/
	/********PA2->ADC.IN3********/
	/********PA3->ADC.IN4********/
	// Initialize analog GPIO
	GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_Analog;
	GPIO_InitStructure.GPIO_Alternate = GPIO_NO_AF;
	GPIO_InitStructure.Pin            = GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1;
	GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

  // ALTERNATE FUNCTIONS FOR COMM 
	// USART1 -------------------------------------------
	/********PA9->USART1.TX********/
	GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pull      = GPIO_Pull_Up;
	GPIO_InitStructure.GPIO_Slew_Rate = GPIO_Slew_Rate_Low;
	GPIO_InitStructure.GPIO_Current   = GPIO_DC_2mA;
	GPIO_InitStructure.GPIO_Alternate = GPIO_AF4_USART1;
	GPIO_InitStructure.Pin            = GPIO_PIN_9;
	GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);     
	/********PA10->USART1.RX********/
	GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_Input;
	GPIO_InitStructure.GPIO_Pull      = GPIO_Pull_Up;
	GPIO_InitStructure.GPIO_Alternate = GPIO_AF4_USART1;
	GPIO_InitStructure.Pin            = GPIO_PIN_10;
	GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

	// ALTERNATE FUNCTIONS FOR ENCODER
	// SPI1 -------------------------------------------
	/********PB3->SPI1.SCK********/
	/********PB4->SPI1.MISO********/
	/********PB5->SPI1.MOSI********/
	// PB5 as MOSI (OUTPUT)
	GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pull      = GPIO_No_Pull;
	GPIO_InitStructure.GPIO_Slew_Rate = GPIO_Slew_Rate_Low;
	GPIO_InitStructure.GPIO_Current   = GPIO_DC_2mA;
	GPIO_InitStructure.GPIO_Alternate = GPIO_AF0_SPI1;
	GPIO_InitStructure.Pin            = GPIO_PIN_5;
	GPIO_InitPeripheral(GPIOB,&GPIO_InitStructure);
	// PB3 and PB4 as SCK and MISO
	GPIO_InitStructure.GPIO_Alternate = GPIO_AF1_SPI1;
	GPIO_InitStructure.Pin            = GPIO_PIN_4 | GPIO_PIN_3;
	GPIO_InitPeripheral(GPIOB,&GPIO_InitStructure);

	// ALTERNATE FUNCTIONS FOR PWM
	/********PA6->TIM3.CH1********/
	/********PA7->TIM3.CH2********/
	/********PB0->TIM3.CH3********/
	/********PB1->TIM3.CH4********/
  GPIO_InitStruct(&GPIO_InitStructure);
  // GPIOA: PA6 and PA7 Configuration: TIM3 Channel1, Channel2, as alternate function push-pull
	//        PA6 is Channel1
	//        PA7 is Channel2
  GPIO_InitStructure.Pin            = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pull      = GPIO_No_Pull;
	GPIO_InitStructure.GPIO_Slew_Rate = GPIO_Slew_Rate_Low;
  GPIO_InitStructure.GPIO_Current   = GPIO_DC_2mA;
  GPIO_InitStructure.GPIO_Alternate = GPIO_AF2_TIM3;
  GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
  // GPIOB: PB0 and PB1 configuration
	//        PB0 is Channel3
	//        PB1 is Channel4
  GPIO_InitStructure.Pin            = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pull      = GPIO_No_Pull;
	GPIO_InitStructure.GPIO_Slew_Rate = GPIO_Slew_Rate_Low;
  GPIO_InitStructure.GPIO_Current   = GPIO_DC_2mA;
  GPIO_InitStructure.GPIO_Alternate = GPIO_AF2_TIM3;
  GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
     
  // ALTERNATE FUNCTIONS FOR I2C1
	/********PA4->I2C1.SCL********/
	/********PA5->I2C1.SDA********/
	GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Pull      = GPIO_Pull_Up;
	GPIO_InitStructure.GPIO_Slew_Rate = GPIO_Slew_Rate_Low;
	GPIO_InitStructure.GPIO_Current   = GPIO_DC_2mA;
	GPIO_InitStructure.GPIO_Alternate = GPIO_AF7_I2C1;
	GPIO_InitStructure.Pin            = GPIO_PIN_4 | GPIO_PIN_5;
	GPIO_InitPeripheral(GPIOA,&GPIO_InitStructure);
}

// USART1 Configuration
void USART_Configuration(void) {
	// variable for USART configuration
	USART_InitType 			USART_InitStructure;

	// Enable USART1 clock
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_USART1, ENABLE);	
	// USART1 configuration ------------------------------------------------------
	// USART1: always settup for printf or 485
	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.BaudRate            = 115200;
	USART_InitStructure.WordLength          = USART_WL_8B;
	USART_InitStructure.StopBits            = USART_STPB_1;
	USART_InitStructure.Parity              = USART_PE_NO;
	USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
	USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;
	// Configure USART1
	USART_Init(USART1, &USART_InitStructure);
	// Configuration interrupts for USART1
	// USART1 as 485 comm, so, configure interrupts
	// Clear flags
	USART_ClrFlag(USART1, USART_FLAG_RXDNE | USART_FLAG_TXC );
	// Update int flags
	USART_ConfigInt(USART1, USART_INT_TXDE, DISABLE);
	USART_ConfigInt(USART1, USART_INT_TXC, DISABLE);
	USART_ConfigInt(USART1, USART_INT_RXDNE, ENABLE);
	// Enable the USART1
	USART_Enable(USART1, ENABLE);
}

// Counter max value
#define _MAX_VAL_ 800
void TIM_Configuration(void) {
	TIM_TimeBaseInitType TIM_TimeBaseStructure;
	OCInitType           TIM_OCInitStructure;

	// Enable TIM3 clock
	// TIM3 source clock at 32 MHz
	RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM2 | RCC_APB1_PERIPH_TIM3, ENABLE);

	/*********************** TIM3 configuration *********************/
  // TIM3 at 20 KHz in center aligment Mode
  TIM_InitTimBaseStruct(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.Period    = _MAX_VAL_ - 1;
  TIM_TimeBaseStructure.Prescaler = 0;
  TIM_TimeBaseStructure.ClkDiv    = 0;
  TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_CENTER_ALIGN1;
  TIM_InitTimeBase(TIM3, &TIM_TimeBaseStructure);
  // TIM3 channels configuration
	// Inverted PWM because start with LOW polarity
	// This is for allowing ADC measurement
  TIM_InitOcStruct(&TIM_OCInitStructure);
  TIM_OCInitStructure.OcMode       = TIM_OCMODE_PWM1;
  TIM_OCInitStructure.OutputState  = TIM_OUTPUT_STATE_ENABLE;
  TIM_OCInitStructure.Pulse        = _MAX_VAL_;  // PWM to 0%
  TIM_OCInitStructure.OcPolarity   = TIM_OC_POLARITY_LOW;
	// Init channels output
	// Channel 1 -> PA6
  TIM_InitOc1(TIM3, &TIM_OCInitStructure);            // PA6
  TIM_OCInitStructure.OutputState  = TIM_OUTPUT_STATE_ENABLE; // Repeated
	TIM_ConfigOc1Preload(TIM3, TIM_OC_PRE_LOAD_ENABLE);	// PA6 autoload
	// Channel 2 -> PA7
	TIM_InitOc2(TIM3, &TIM_OCInitStructure);            // PA7
  TIM_OCInitStructure.OutputState  = TIM_OUTPUT_STATE_ENABLE; // Repeated
	TIM_ConfigOc2Preload(TIM3, TIM_OC_PRE_LOAD_ENABLE);	// PA7 autoload
	// Channel 3 -> PB0
	TIM_InitOc3(TIM3, &TIM_OCInitStructure);            // PB0
  TIM_OCInitStructure.OutputState  = TIM_OUTPUT_STATE_ENABLE; // Repeated
	TIM_ConfigOc3Preload(TIM3, TIM_OC_PRE_LOAD_ENABLE);	// PB0 autoload
	// Channel 4 -> PB1
	TIM_InitOc4(TIM3, &TIM_OCInitStructure);            // PB1
  TIM_OCInitStructure.OutputState  = TIM_OUTPUT_STATE_ENABLE; // Repeated
	TIM_ConfigOc4Preload(TIM3, TIM_OC_PRE_LOAD_ENABLE);	// PB1 autoload
	// config TIM3 autolader Preload
	TIM_ConfigArPreload(TIM3,ENABLE);
	// Selects the TIM3 Trigger Output Reset Mode
	TIM_SelectOutputTrig(TIM3, TIM_TRGO_SRC_UPDATE);
	// config TIM3Master Slave Mode
	TIM_SelectMasterSlaveMode(TIM3, TIM_MASTER_SLAVE_MODE_DISABLE);
	// TIM3 counter enable
	TIM_Enable(TIM3,ENABLE);
	
	/*********************** TIM2 configuration *********************/
  // TIM2 at 10 KHz
  TIM_InitTimBaseStruct(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.Period    = 999;
  TIM_TimeBaseStructure.Prescaler = 31;
  TIM_TimeBaseStructure.ClkDiv    = 0;
  TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;
  TIM_InitTimeBase(TIM2, &TIM_TimeBaseStructure);
	// Enable interrupt
	TIM_ConfigInt(TIM2, TIM_INT_UPDATE, ENABLE);
	// Autoload 
	TIM_ConfigArPreload(TIM2, ENABLE);
	// Enable TIM2
	TIM_Enable(TIM2, ENABLE);	
}

void SPI_Configuration(void) {
	// Enable SPI1 clock
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_SPI1, ENABLE);

	/***************** SPI1 configuration ***************/
	// Clock at 32 MHz
	// Set SPI as MODE3 at 16 MHz (32/2)
	SPI_InitType SPI_InitStructure;
	SPI_InitStruct(&SPI_InitStructure);
	SPI_InitStructure.DataDirection = SPI_DIR_DOUBLELINE_FULLDUPLEX;
	SPI_InitStructure.SpiMode       = SPI_MODE_MASTER;
	SPI_InitStructure.DataLen       = SPI_DATA_SIZE_16BITS;
	SPI_InitStructure.CLKPOL        = SPI_CLKPOL_HIGH;
	SPI_InitStructure.CLKPHA        = SPI_CLKPHA_SECOND_EDGE;
	SPI_InitStructure.NSS           = SPI_NSS_SOFT;
	SPI_InitStructure.BaudRatePres  = SPI_BR_PRESCALER_2;
	SPI_InitStructure.FirstBit      = SPI_FB_MSB;
	SPI_InitStructure.CRCPoly       = 15;
	SPI_Init(SPI1, &SPI_InitStructure);
	// Enable SPI1
	SPI_Enable(SPI1, ENABLE);
}

void I2C_Configuration(void) {
	// Enable I2C1 clock
	RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_I2C1, ENABLE);
	/*********************** I2C1 configuration *********************/
	I2C_InitType I2C_InitStructure;
  I2C_DeInit(I2C1);
	I2C_InitStruct(&I2C_InitStructure);
  I2C_InitStructure.BusMode     = I2C_BUSMODE_I2C;
  I2C_InitStructure.OwnAddr1    = 0x00;
  I2C_InitStructure.AckEnable   = I2C_ACKEN;
  I2C_InitStructure.AddrMode    = I2C_ADDR_MODE_7BIT;
  //i2c1_master.FmDutyCycle = I2C_FMDUTYCYCLE_2;
  I2C_InitStructure.ClkSpeed    = 100000; // 100K
	// Enables  the specified I2C Clock stretching
  I2C_Init(I2C1, &I2C_InitStructure);
	I2C_EnableExtendClk(I2C1, ENABLE);
	I2C_Enable(I2C1, ENABLE);
}

void ADC_Configuration(void) {
	// Enable ADC Clock
	RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC,ENABLE);
	// ADC configuration
	// Multichannel mode: PA1, PA2 and PA3
	ADC_InitType   ADC_InitStructure;
	ADC_InitStruct(&ADC_InitStructure);
	ADC_InitStructure.MultiChEn      = ENABLE;
	ADC_InitStructure.ContinueConvEn = DISABLE;
	ADC_InitStructure.ExtTrigSelect  = ADC_EXT_TRIGCONV_T3_TRGO;
	ADC_InitStructure.DatAlign       = ADC_DAT_ALIGN_R;
	ADC_InitStructure.ChsNumber      = 3;   // 3 channel conversion
	ADC_Init(ADC, &ADC_InitStructure);
	// ADC Pins Configuration
	// PA1: Channel number 1
	ADC_ConfigRegularChannel(ADC, ADC_CH_2_PA1, 1, ADC_SAMP_TIME_1CYCLES5);
	// PA2: Channel number 2
	ADC_ConfigRegularChannel(ADC, ADC_CH_3_PA2, 2, ADC_SAMP_TIME_1CYCLES5);
	// PA3: Channel number 3
	ADC_ConfigRegularChannel(ADC, ADC_CH_4_PA3, 3, ADC_SAMP_TIME_1CYCLES5);
	// Enable ADC external trigger conversion (TIM3 UPDATE)
	ADC_EnableExternalTrigConv(ADC, ENABLE);
	
	// Config SAMPSEL
	ADC_InitTypeEx ADC_InitStructEx;
	// 303 style for fast ADC
	ADC_InitStructEx.Samp303Style=true;
	// Set Adc Clock bits for syncronous AHB conversions
	ADC_InitStructEx.ClkMode = ADC_CTRL3_CKMOD_AHB;
	// Set Adc to 12 bits resolution
	ADC_InitStructEx.ResBit  = ADC_CTRL3_RES_12BIT;
	ADC_InitEx(ADC, &ADC_InitStructEx);
	// Enable ADC DMA
	ADC_EnableDMA(ADC, ENABLE);
	// Enable ADC
	ADC_Enable(ADC, ENABLE);
	
	// Check ADC Ready
	while(ADC_GetFlagStatusNew(ADC,ADC_FLAG_RDY) == RESET);
	// Start ADC calibration
	ADC_StartCalibration(ADC);
	// Check the end of ADCcalibration
	while (ADC_GetCalibrationStatus(ADC));
}

// Local variable for ADC converted values
// First 3 bytes: ADC readings at start PWM
// Last  3 bytes: not used, ADC readings at center PWM
__IO uint16_t ADC_Values[6];

void DMA_Configuration(void) {
	DMA_InitType DMA_InitStructure;
	// Enable DMA clock
	RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_DMA,ENABLE);

  // DMA Channel1 Configuration 
  // Copy ADC converted value (ADC->DAT) to local mmemory
	// Six readings: 3 at start, 3 at middle
  DMA_DeInit(DMA_CH1);
  DMA_InitStructure.PeriphAddr     = (uint32_t)&ADC->DAT;
  DMA_InitStructure.MemAddr        = (uint32_t)ADC_Values;
  DMA_InitStructure.Direction      = DMA_DIR_PERIPH_SRC;
  DMA_InitStructure.BufSize        = 6;
  DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_DISABLE;
  DMA_InitStructure.DMA_MemoryInc  = DMA_MEM_INC_ENABLE;
  DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_SIZE_HALFWORD;
  DMA_InitStructure.MemDataSize    = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.CircularMode   = DMA_MODE_CIRCULAR;
  DMA_InitStructure.Priority       = DMA_PRIORITY_VERY_HIGH;
  DMA_InitStructure.Mem2Mem        = DMA_M2M_DISABLE;
  DMA_Init(DMA_CH1, &DMA_InitStructure);
  DMA_RequestRemap(DMA_REMAP_ADC1, DMA, DMA_CH1, ENABLE);
  // Enable DMA Channel
  DMA_EnableChannel(DMA_CH1, ENABLE);
}


//
// Motor Object Dictionary 
Motor_OD_Entry_t object_dictionary[OD_NUM_REGS];


// MAIN
int main() {
	// N32 configuration files
	// Clock at 64MHz using HSI
	RCC_Configuration();
	// USART1 Interrupt for COMM
	NVIC_Configuration();
	// Pins configuration
	GPIO_Configuration();
	// USART baudrate and configuration
	USART_Configuration();
	// TIM3 for PWM generation, TIM2 for counting us
	TIM_Configuration();
	// SPI for encoder readings
	SPI_Configuration();
	// I2C for OLED writing
	I2C_Configuration();
	// DMA for translating ADC reading to local variable
	DMA_Configuration();
	// ADC automatically start at each PWM
	ADC_Configuration();
	
	// Init motor OD with pointer to real variables
	motor_init( object_dictionary );
	// Init adc with pointer to real variables
	adc_init( ADC_Values, object_dictionary );
	// Init PWM input to variables in OD
	pwm_init( object_dictionary );
	// Init encoder with pointer to real variables
	encoder_init( object_dictionary );
	// Init comm
	com485_init( object_dictionary );
	// Init flash
	flash_init( object_dictionary );
	
	// Restore data from flash
	motor_load_flash_data();

	// Init Oled
	oled_init();
	oled_clearAll();
	
	// Led always on, it will off when error occurs
	GPIO_SetBits( GPIOD, GPIO_PIN_0 );

	// While 
	while (true) {
		// initial time
		uint64_t main_start = time_get_us();
		// control loop
		motor_control_loop();
		// proccess command from user
		motor_command_loop();
		// wait for 100 us between loops
		int16_t main_wait = 100 - (int16_t)(time_get_us() - main_start);
		if ( main_wait > 0 ) time_delay_us( main_wait );
	}

  return 0;
}
