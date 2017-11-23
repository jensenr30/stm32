/**
 ******************************************************************************
 * @file    GPIO/GPIO_IOToggle/Src/main.c
 * @author  MCD Application Team
 * @version V1.0.0
 * @date    06-May-2016
 * @brief   This example describes how to configure and use GPIOs through
 *          the STM32F4xx HAL API.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
#include "stm32f4xx.h"
#include "stm32f4xx_nucleo_144.h"
//#include "../../../../app/key.h"
#include <stdlib.h>

/** @addtogroup STM32F4xx_HAL_Examples
 * @{
 */

/** @addtogroup GPIO_IOToggle
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static GPIO_InitTypeDef GPIO_Struct;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

// shift register pins
// port C pins
#define SR_DATA  GPIO_PIN_1	// SER (74hc595 pin 14)	data in
#define SR_LATCH GPIO_PIN_4	// RCK (74hc595 pin 12) register clock (update output)
#define SR_CLOCK GPIO_PIN_5	// SCK (74hc595 pin 11) data clock in
#define SR_GPIO GPIOC		// this is the gpio register that the shift register is on

// input pins
// port C pins
#define IN_1 GPIO_PIN_8
#define IN_2 GPIO_PIN_9
#define IN_3 GPIO_PIN_10
#define IN_4 GPIO_PIN_11
#define IN_5 GPIO_PIN_12
#define IN_6 GPIO_PIN_2
#define IN_7 GPIO_PIN_2
#define IN_8 GPIO_PIN_3

// input variables
#define INPUT_PINS 8
GPIO_PinState input_state[INPUT_PINS];
//GPIO_TypeDef input_pin_port[INPUT_PINS];

#define input_pin_port_1 GPIOC
#define input_pin_port_2 GPIOC
#define input_pin_port_3 GPIOC
#define input_pin_port_4 GPIOC
#define input_pin_port_5 GPIOC
#define input_pin_port_6 GPIOD
#define input_pin_port_7 GPIOG
#define input_pin_port_8 GPIOG

#define song_period ((uint16_t)512)
#define hit_time ((uint16_t)1)
#define note_quarter note_hit(4)
#define note_sixteenth note_hit(16)
#define note_sixty_fourth note_hit(64)




void clock_out(GPIO_TypeDef* GPIOx, uint16_t clockPin, uint16_t dataPin, uint16_t latchPin, uint8_t bits, uint32_t data)
{
	uint8_t i;
	uint32_t mask = 1;
	for (i = 0; i < bits; i++)
	{
		// set data
		if ((mask & data) != 0)
		{
			GPIOx->BSRR = dataPin;
		}
		else
		{
			GPIOx->BSRR = (uint32_t) dataPin << 16U;
		}
		// clock data in
		GPIOx->BSRR = clockPin;
		GPIOx->BSRR = (uint32_t) clockPin << 16U;
		// get next bit
		mask <<= 1;
	}
	// cycle the latch to clock all the bits in
	GPIOC->BSRR = latchPin;
	GPIOC->BSRR = (uint32_t) latchPin << 16U;
}


void note_hit(uint32_t n)
{
	clock_out(GPIOC, SR_CLOCK, SR_DATA, SR_LATCH, 8, 0xff);
	HAL_Delay(hit_time);
	clock_out(GPIOC, SR_CLOCK, SR_DATA, SR_LATCH, 8, 0x00);
	HAL_Delay((song_period / (uint16_t) n) - hit_time);
}

//=============================================================================
// TIM3 Interrupt Handler
//=============================================================================
void TIM3_IRQHandler(void)
{
	if (TIM3->SR & TIM_SR_UIF) // if UIF flag is set
	{
		TIM3->SR &= ~TIM_SR_UIF; // clear UIF flag
		SR_GPIO->ODR ^= SR_CLOCK; // toggle pin state
	}
}



int main(void)
{
	/* STM32F4xx HAL library initialization:
	 - Configure the Flash prefetch
	 - Systick timer is configured by default as source of time base, but user 
	 can eventually implement his proper time base source (a general purpose 
	 timer for example or other time source), keeping in mind that Time base 
	 duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
	 handled in milliseconds basis.
	 - Set NVIC Group Priority to 4
	 - Low Level Initialization
	 */
	HAL_Init();
	/* Configure the system clock to 100 MHz */
	SystemClock_Config();
	
	// enable the timer 3 clock
	__HAL_RCC_TIM3_CLK_ENABLE();
	
	TIM3->PSC = 23999;	        // Set prescaler to 24 000 (PSC + 1)
	TIM3->ARR = 1000;	          // Auto reload value 1000
	TIM3->DIER = TIM_DIER_UIE; // Enable update interrupt (timer level)
	TIM3->CR1 = TIM_CR1_CEN;   // Enable timer
	
	// enable timer 3 interrupt handler
	NVIC_EnableIRQ(TIM3_IRQn);
	
	// enable port C
	__HAL_RCC_GPIOC_CLK_ENABLE();
	// setup struct
	GPIO_Struct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_Struct.Pull = GPIO_NOPULL;
	GPIO_Struct.Speed = GPIO_SPEED_HIGH;
	
	// enable
	GPIO_Struct.Pin = SR_DATA;
	HAL_GPIO_Init(GPIOC, &GPIO_Struct);
	GPIO_Struct.Pin = SR_LATCH;
	HAL_GPIO_Init(GPIOC, &GPIO_Struct);
	GPIO_Struct.Pin = SR_CLOCK;
	HAL_GPIO_Init(GPIOC, &GPIO_Struct);
	
	// test led
	GPIO_Struct.Pin = GPIO_PIN_3;
	HAL_GPIO_Init(GPIOC, &GPIO_Struct);
	
	// enable port d
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;
	//enable port g
	__HAL_RCC_GPIOG_CLK_ENABLE()
	;
	// setup struct
	GPIO_Struct.Mode = GPIO_MODE_INPUT;
	GPIO_Struct.Pull = GPIO_NOPULL;
	GPIO_Struct.Speed = GPIO_SPEED_HIGH;
	
	// input pins
	GPIO_Struct.Pin = IN_1;
	HAL_GPIO_Init(input_pin_port_1, &GPIO_Struct);
	GPIO_Struct.Pin = IN_2;
	HAL_GPIO_Init(input_pin_port_2, &GPIO_Struct);
	GPIO_Struct.Pin = IN_3;
	HAL_GPIO_Init(input_pin_port_3, &GPIO_Struct);
	GPIO_Struct.Pin = IN_4;
	HAL_GPIO_Init(input_pin_port_4, &GPIO_Struct);
	GPIO_Struct.Pin = IN_5;
	HAL_GPIO_Init(input_pin_port_5, &GPIO_Struct);
	GPIO_Struct.Pin = IN_6;
	HAL_GPIO_Init(input_pin_port_6, &GPIO_Struct);
	GPIO_Struct.Pin = IN_7;
	HAL_GPIO_Init(input_pin_port_7, &GPIO_Struct);
	GPIO_Struct.Pin = IN_8;
	HAL_GPIO_Init(input_pin_port_8, &GPIO_Struct);
	
	// clear the LEDs
	clock_out(GPIOC, SR_CLOCK, SR_DATA, SR_LATCH, 8, 0);
	
	// uint16_t T = 512;
	// main loop
	while (1)
	{
		/*
		// do nothing
		note_hit(4);
		 
		note_hit(4);
		 
		note_hit(4);
		 
		note_hit(16);
		note_hit(16);
		note_hit(16);
		note_hit(16);
		 
		note_hit(4);
		 
		note_hit(4);
		 
		note_hit(8);
		note_hit(8);
		 
		note_hit(24);
		note_hit(24);
		note_hit(24);
		note_hit(24);
		note_hit(24);
		note_hit(24);
		*/
	}
}

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow : 
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 100000000
 *            HCLK(Hz)                       = 100000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 2
 *            APB2 Prescaler                 = 1
 *            HSE Frequency(Hz)              = 8000000
 *            PLL_M                          = 8
 *            PLL_N                          = 200
 *            PLL_P                          = 2
 *            PLL_Q                          = 7
 *            PLL_R                          = 2
 *            VDD(V)                         = 3.3
 *            Main regulator output voltage  = Scale1 mode
 *            Flash Latency(WS)              = 3
 * @param  None
 * @retval None
 */
static void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	HAL_StatusTypeDef ret = HAL_OK;
	
	/* Enable Power Control clock */
	__HAL_RCC_PWR_CLK_ENABLE()
	;
	
	/* The voltage scaling allows optimizing the power consumption when the device is 
	 clocked below the maximum system frequency, to update the voltage scaling value 
	 regarding system frequency refer to product datasheet.  */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	
	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 200;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	RCC_OscInitStruct.PLL.PLLR = 2;
	ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
	
	if (ret != HAL_OK)
	{
		while (1)
		{
			;
		}
	}
	
	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
	 clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);
	if (ret != HAL_OK)
	{
		while (1)
		{
			;
		}
	}
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
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{	
	}
}
#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
