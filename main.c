/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_dac.h"

#include "fsl_common.h"
#include "clock_config.h"
#include "pin_mux.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_DAC_BASEADDR DAC0

#define ENABLE_CLOCK_PORTB 0x400
#define ENABLE_CLOCK_PORTE 0x2000
#define ENABLE_CLOCK_PORTC 0x800
#define ENABLE_CLOCK_PORTA 0x200
static const uint32_t pc6_sw2=0x40;//6
static const uint32_t pa4_sw3=0x10;//4
#define PIN_AS_GPIO 0x100  //valor 001 para mux en bits 10 a 8 del registro PCR pag 276
#define PIN_LED_ROJO  22  //bit numero 22 en el puerto B
#define PIN_LED_AZUL  21  //bit numero 21 en el puerto B
#define PIN_LED_VERDE 26  // bit numero 26 en el puerto E
#define PIN_SW2 6         // numero 6 del puerto C
#define PIN_SW3 4         //numero 4 del puerto A
#define set_pullup 0x3  //bits de pull select and pull enable

void set_gpio();
float hn[]={
		0.07840464525404556,
		0.17707825519483075,
		0.22014353249171387,
		0.2759015644497544,
		0.22014353249171387,
		0.17707825519483075,
		0.07840464525404556
};
float sampleADC[7]={0};
uint16_t ADC_result(void);
void delay(uint16_t delay);
void init_sws();
uint32_t sw3_pressed();
uint32_t sw2_pressed();
void init_ADC();
void init_DAC();
void set_DAC_output(uint16_t Dac_Value);
void debug_ADC_and_DAC();
uint16_t convolution();
uint16_t imput_equals_output();
uint16_t operation_to_audio(uint8_t operation);
enum {CONVOLUTION, INPUT_EQUALS_OUTPUT}dsp_operation_t;

int main(void)
{
	init_sws();
	init_DAC();
	init_ADC();
	//dac_config_t dacConfigStruct;
	// uint16_t dacValue;

	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();
	//int8_t operacion=INPUT_EQUALS_OUTPUT;
	int8_t operacion=CONVOLUTION;
	debug_ADC_and_DAC();//test 12 bit DAC and 12 bit ADC
	set_DAC_output(2000);
	uint16_t diminish_DAC_amplitude=0;
	//uint16_t amplitudeControl=0;
	while (1)
	{

		uint16_t DAC_input=operation_to_audio(operacion)-diminish_DAC_amplitude;
		printf("DAc input %i\n",DAC_input);//debug only

		if(sw3_pressed())//if sw3 is pressed no convolution is efectuated
		{

			while(sw3_pressed())//while sw3 is pressed
			{
				if(sw2_pressed())
				{
					while(sw2_pressed());
					diminish_DAC_amplitude+=50;
					if(1000<=diminish_DAC_amplitude)
					{
						diminish_DAC_amplitude=0;
					}
				}
				else
				{
					operacion=INPUT_EQUALS_OUTPUT;
				}
			}

		}
		if(sw2_pressed())// if sw2 is pressed convolution is efectuated
		{
			while(sw2_pressed())
			{
				if(sw3_pressed())
				{
					diminish_DAC_amplitude=0;
				}
				else
				{
					operacion=CONVOLUTION;
				}
			}

		}

		set_DAC_output(2000);//debug porpuses
		delay(16000);//debug porpuses
		//set_DAC_output(DAC_input)// sentece to output to dac
	}

}


uint16_t ADC_result(void)
{
	uint16_t adc_result;
	ADC0->SC1[0] = ADC_SC1_ADCH(20);//20 is the ADC channel,17
	while( (ADC0->SC1[0] & ADC_SC1_COCO_MASK) == 0);//waiting  for ADC to complete conversion :coco flag is set
	adc_result = ADC0->R[0];
	return adc_result;
}
void delay(uint16_t delay)
{
	volatile uint16_t counter;

	for(counter=delay; counter > 0; counter--)
	{
		__asm("nop");
	}
}
void init_sws()
{
	// habilitando sw2   y sw3
	SIM->SCGC5|=ENABLE_CLOCK_PORTA;
	SIM->SCGC5|=ENABLE_CLOCK_PORTC;

	// habilitando pines de Sw2 (pc6)	y sw3 (pa4) como gpio
	PORTA->PCR[PIN_SW3]=PIN_AS_GPIO;
	PORTC->PCR[PIN_SW2]=PIN_AS_GPIO;
	// ahora queda habilitar las pull up de los sw
	PORTC->PCR[PIN_SW2]|=set_pullup;
	PORTA->PCR[PIN_SW3]|=set_pullup;
	//es buena practica asegurarse que los pines sean entrada
	GPIOC->PDDR&=~(pc6_sw2);
	GPIOA->PDDR&=~(pa4_sw3);
}
uint32_t sw2_pressed()
{
	uint32_t input_value=GPIOC->PDIR;
	input_value= input_value & pc6_sw2;
	if(0 != input_value)
	{
		return 0;
	}

	return 1;
}
uint32_t sw3_pressed()

{
	uint32_t check=GPIOA->PDIR;
	check= check & pa4_sw3;
	if(0 != check)
	{
		return 0;
	}
	return 1;
}
void init_ADC()
{

	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;
	//												 //12 bit res
	ADC0->CFG1 = ADC_CFG1_ADIV(0)|ADC_CFG1_ADLSMP_MASK |ADC_CFG1_MODE(1)|ADC_CFG1_ADICLK(0);
	ADC0->CFG2 = 0;
	ADC0->SC2 = 0;
	ADC0->SC3 = ADC_SC3_AVGE_MASK |ADC_SC3_AVGS(3);
}
void init_DAC()
{
	SIM->SCGC2 = 0x1000;
	DAC0->C0 = 0xC0;
	DAC0->DAT[0].DATL = 0;
	DAC0->DAT[0].DATH = 0;


}
uint16_t convolution()
{
	sampleADC[6]=(float)ADC_result();
	sampleADC[5]=(float)ADC_result();
	sampleADC[4]=(float)ADC_result();
	sampleADC[3]=(float)ADC_result();
	sampleADC[2]=(float)ADC_result();
	sampleADC[2]=(float)ADC_result();
	sampleADC[1]=(float)ADC_result();

	float yn=0;
	yn+=hn[0]*sampleADC[6];
	yn+=hn[1]*sampleADC[5];
	yn+=hn[2]*sampleADC[4];
	yn+=hn[3]*sampleADC[3];
	yn+=hn[4]*sampleADC[2];
	yn+=hn[5]*sampleADC[1];
	yn+=hn[6]*sampleADC[0];
	//yn is a floating point value
	printf("yn is %f\n",yn);
	return (uint16_t)yn;

}
uint16_t imput_equals_output()
{

	return ADC_result() ;
}
uint16_t operation_to_audio(uint8_t operation)
{
	uint16_t valueToDAC;
	switch (operation)
	{
	case 0:
		valueToDAC = convolution();
		break;
	case 1:
		valueToDAC = imput_equals_output();
		break;
	default:
		valueToDAC = imput_equals_output();

	}
	return valueToDAC;
}
void set_DAC_output(uint16_t Dac_Value)
{
	DAC0->DAT[0].DATL = 0x00FF & Dac_Value;
	DAC0->DAT[0].DATH = Dac_Value>>8;
	//DAC0->DAT[0].DATL = 0xFF;
	//DAC0->DAT[0].DATH = 0x07;
}
void debug_ADC_and_DAC()
{
	set_DAC_output(500);
	printf(" adc value %i\n",ADC_result());
	set_DAC_output(1000);
	printf(" adc value %i\n",ADC_result());
	set_DAC_output(2000);
	printf(" adc value %i\n",ADC_result());
	set_DAC_output(3000);
	printf(" adc value %i\n",ADC_result());
	set_DAC_output(3500);
	printf(" adc value %i\n",ADC_result());
	set_DAC_output(4000);
	printf(" adc value %i\n",ADC_result());
}
