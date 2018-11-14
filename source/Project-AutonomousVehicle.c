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
SemaphoreHandle_t SemSensorReader;
SemaphoreHandle_t semMotors;

int CnV0 = 1400;
int CnV1 = 2000;

double sensor1Input = 0.0;
double sensor2Input = 0.0;


void stopMovement(void);
void rightMovment(void);
void leftMovment(void);
void forwardMovment(void);


/* This task initializes the TPM as well as calls the functions for the different movement styles
 * forwardMovement = move the left motors counter clockwise and the right motor clockwise at the same rate.
 * leftMovment = move the 2 motors clockwise at the same rate
 * rightMovement = move the 2 motors counter clockwise at the same rate
 * */
void vMovement(void*pv) {

	//xSemaphoreTake(semMotors, portMAX_DELAY);

    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK | SIM_SCGC6_TPM0_MASK;

    PORTE->PCR[30] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[30] |= PORT_PCR_MUX(3);

    PORTE->PCR[22] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[22] |= PORT_PCR_MUX(3);

    SIM->SOPT2 &= SIM_SOPT2_TPMSRC_MASK;
    // Options: 00 Clock disabled; 01 IRC48M; 10 OSCERCLK; 11 MCGIRCLK
    MCG->C1 |= MCG_C1_IRCLKEN_MASK; SIM->SOPT2 |= SIM_SOPT2_TPMSRC(0b11);
    TPM2->CONF |= TPM_CONF_DBGMODE(0b11); // allow the timer to run in debug mode

    TPM2->MOD = 18181; // the timer counter will count from 0 to period
    TPM2->SC = TPM_SC_CMOD(1) | TPM_SC_PS(3);
    TPM2->CONTROLS[0].CnSC= TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;

    TPM0->MOD = 18181; // the timer counter will count from 0 to period
    TPM0->SC = TPM_SC_CMOD(1) | TPM_SC_PS(3);
    TPM0->CONTROLS[3].CnSC= TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;


    while(1) {
		/* sensor1 is blocked within a certain voltage */
		if(getVotlage() < 1000.0) {
			stopMovement();
	    	/* sensor1 and sensor2 are within a certain voltage */
	    	if(0) {
	    		// turn left
	    		leftMovment();
	    	/* sensor1 is within a certain voltage and sensor2 is not */
	    	} else if(0) {
	    		// turn right
	    		rightMovment();
	    	}
		} else {
			// move forward
			forwardMovment();
		}
    	taskYIELD();
    }
}

void forwardMovment(void) {
    TPM2->CONTROLS[0].CnV=1200;
    TPM0->CONTROLS[3].CnV=1900;
}

void leftMovment(void) {
    TPM2->CONTROLS[0].CnV=1400;
    TPM0->CONTROLS[3].CnV=1400;
}

void rightMovment(void) {
    TPM2->CONTROLS[0].CnV=600;
    TPM0->CONTROLS[3].CnV=600;
}
void stopMovement(void) {
	TPM2->CONTROLS[0].CnV=1440;
	TPM0->CONTROLS[3].CnV=1440;
}
void vApplicationIdleHook(void) {
	__asm volatile ("wfe"); // wait for interrupt; CPU waits in low power mode
}

/*
 * @brief   Application entry point.
 */
void vTaskConverter(void *pv){
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;


    PORTE->PCR[29] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[29] |= PORT_PCR_MUX(0);

    PORTE->PCR[23] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[23] |= PORT_PCR_MUX(0);

    ADC0->SC2 = 1; // select correct voltage reference
    ADC0->CFG1 = ADC_CFG1_MODE(3); // 16-bit conversions
    ADC0->CFG2 = ADC_CFG2_MUXSEL(0);

    NVIC_EnableIRQ(ADC0_IRQn);
    __enable_irq();

    while(1){
    	double dataChanel1[5];
    	double dataChanel2[5];

    	for(int i=0; i<5; i++){

    	    			//Sensor #1 initialization
    	    			ADC0->SC2 = 1; // select correct voltage reference
    	    			ADC0->CFG1 = ADC_CFG1_MODE(3); // 16-bit conversions
    	    			ADC0->CFG2= ADC_CFG2_MUXSEL(0);

    	    			//Sensor #1
    	    		    ADC0->SC1[0] = ADC_SC1_ADCH(0b111); // conversion on specified channel

    	    		    int data1 = (ADC0->R[0] * 3123)/65536;

    	    		    PRINTF("V1 = %d\n",data1);

    	    		    dataChanel1[i] =(ADC0->R[0] * 3123)/65536;


    	    		    //Sensor #2 initialization

    	    		    ADC0->SC2 = 1; // select correct voltage reference
    	    		    ADC0->CFG1 = ADC_CFG1_MODE(3); // 16-bit conversions
    	    		    ADC0->CFG2= ADC_CFG2_MUXSEL(1);

    	    		    //Sensor #2
    	    		    ADC0->SC1[0] = ADC_SC1_ADCH(0b100);

    	    		    dataChanel2[i] =(ADC0->R[0] * 3123)/65536;


    	    			}
    	//Takes the semaphore
    	xSemaphoreTake(SemSensorReader, portMAX_DELAY);


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


int main(void) {
  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

    SemSensorReader = xSemaphoreCreateBinary();
    semMotors = xSemaphoreCreateBinary();
    __enable_irq();
  
    xTaskCreate(vTaskConverter, "Task Converter", 300,0,0,NULL);
    xTaskCreate(vMovement, "Move Forward", 300, 0, 0, NULL);
    vTaskStartScheduler();

    /* Enter an infinite loop, just incrementing a counter. */  
    while(1) {	}  
  return 0 ;
}
