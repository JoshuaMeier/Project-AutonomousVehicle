/*
 * Copyright 2016-2018 NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    Project-AutonomousVehicle.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL43Z4.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */
#include"FreeRTOS.h"	// RTOS header file
#include"task.h"		// RTOS task header file
#include"semphr.h"		// RTOS semaphore header file
#include"fsl_slcd.h"

void Print2LCD();

SemaphoreHandle_t SemSensorReader;
/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */
void vTaskConverter(void *pv){
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;


    PORTE->PCR[22] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[22] |= PORT_PCR_MUX(0);
    PORTE->PCR[30] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[30] |= PORT_PCR_MUX(0);

    ADC0->SC2 = 1; // select correct voltage reference
    ADC0->CFG1 = ADC_CFG1_MODE(3); // 16-bit conversions

    NVIC_EnableIRQ(ADC0_IRQn);

    __enable_irq();

    while(1){
	    ADC0->SC1[0] = ADC_SC1_AIEN_MASK | ADC_SC1_ADCH(0b11); // enable ADC interrupt and request conversion on specified channel
    	//Takes the semaphore
    	xSemaphoreTake(SemSensorReader, portMAX_DELAY);

    	int VoADC = (ADC0->R[0] * 3123)/65536;

    	char txt1[5];
    	snprintf(txt1, 5, "%4d", VoADC);
    	Print2LCD(txt1);

    	vTaskDelay( pdMS_TO_TICKS(200) );
    }
}

//ADC0 handler
void ADC0_IRQHandler(void){
	BaseType_t woken = pdFALSE;
	ADC0->SC1[0] = ADC_SC1_ADCH(0b11111); // clear the interrupt flag
	xSemaphoreGiveFromISR(SemSensorReader, &woken);
	portYIELD_FROM_ISR(woken);
}

void vApplicationIdleHook(void) {
	__asm volatile ("wfe"); // wait for interrupt; CPU waits in low power mode
}

int main(void) {
  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

    SemSensorReader = xSemaphoreCreateBinary();

    __enable_irq();

    xTaskCreate(vTaskConverter, "TASK A", 300,0,0,NULL);
    vTaskStartScheduler();	//returns only if the scheduler cannot start

    while(1) {	}
    return 0 ;
}
