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
#include "FreeRTOS.h"	// RTOS header file
#include "task.h"		// RTOS task header file
#include "semphr.h"		// RTOS semaphore header file
#include "fsl_slcd.h" 	// LCD header file

/* TODO: insert other definitions and declarations here. */
void Print2LCD();

SemaphoreHandle_t semMotors;

int CnV0 = 1400;
int CnV1 = 2000;

void vMoveForward(void*pv) {

	//xSemaphoreTake(semMotors, portMAX_DELAY);

    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;

    SIM->SOPT2 &= SIM_SOPT2_TPMSRC_MASK;
    // Options: 00 Clock disabled; 01 IRC48M; 10 OSCERCLK; 11 MCGIRCLK
    MCG->C1 |= MCG_C1_IRCLKEN_MASK; SIM->SOPT2 |= SIM_SOPT2_TPMSRC(0b11);
    TPM2->CONF |= TPM_CONF_DBGMODE(0b11); // allow the timer to run in debug mode

    TPM2->MOD = 18181; // the timer counter will count from 0 to period
    TPM2->SC = TPM_SC_CMOD(1) | TPM_SC_PS(3);
    TPM2->CONTROLS[0].CnSC= TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
    TPM2->CONTROLS[0].CnV=CnV0;

    TPM2->MOD = 18181; // the timer counter will count from 0 to period
    TPM2->SC = TPM_SC_CMOD(1) | TPM_SC_PS(3);
    TPM2->CONTROLS[1].CnSC= TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
    TPM2->CONTROLS[1].CnV=CnV1;


    PORTE->PCR[22] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[22] |= PORT_PCR_MUX(3);

    PORTE->PCR[23] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[23] |= PORT_PCR_MUX(3);
    while(1) {
    	//TPM2->CONTROLS[0].CnV=CnV0;
    	//TPM2->CONTROLS[1].CnV=CnV1;

    	//taskYIELD();
    }
}

void vApplicationIdleHook(void) {
	__asm volatile ("wfe"); // wait for interrupt; CPU waits in low power mode
}

/*
 * @brief   Application entry point.
 */
int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();


    semMotors = xSemaphoreCreateBinary();
    __enable_irq();

    xTaskCreate(vMoveForward, "Move Forward", 300, 1, 0, NULL);
    vTaskStartScheduler();

    /* Enter an infinite loop, just incrementing a counter. */
    return 0 ;
}
