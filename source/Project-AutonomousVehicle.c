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
#include "math.h"


/* TODO: insert other definitions and declarations here. */
int readMag(int *x, int *y, int *z); // writes to x, y, z the magnetometer data
void Print2LCD();

uint16_t period = 8000;

SemaphoreHandle_t semMag;
SemaphoreHandle_t SemSensorReader;
SemaphoreHandle_t semMotors;
SemaphoreHandle_t semMutx;
SemaphoreHandle_t mtx;

int CnV0 = 1400;
int CnV1 = 2000;

double sensor1Input = 0.0;
double sensor2Input = 0.0;

int xm=0;
int ym=0;
int zm=0;

int offsetX = (-2312-2473)/2;
int offsetY = (1885+1626)/2;
int offsetZ = (-2640-2095)/2;

void vTaskComData(void *pv);
void vTaskMagnetometer(void *pv);
void vMovement(void *pv);
void vApplicationIdleHook(void);
void vTaskConverter(void *pv);
void stopMovement(void);
void rightMovment(void);
void leftMovment(void);
void forwardMovment(void);
void ADC0_IRQHandler(void);
void TPM0_IRQHandler(void);


void vTaskComData(void *pv){
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;// enable TPM0 and DMAMUX

	MCG->C1 |= MCG_C1_IRCLKEN_MASK; // enable the MCGIRCLK clock (it has 2 or 8 MHz)
	SIM->SOPT2 &= SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(0b11); // set MCGIRCLK as the TPM clock source

	TPM1->MOD = period; // the timer counter will count from 0 to CNTMAX

	// DMA transfers will be requested when the timer counter reaches CNTMAX:
	TPM1->SC = TPM_SC_CMOD(1) | TPM_SC_TOIE_MASK | TPM_SC_PS(5) | TPM_STATUS_CH1F(2);

	NVIC_EnableIRQ(TPM0_IRQn);
	//NVIC_SetPriority(TPM0_IRQn,2);
	__enable_irq();


	while(1) {
		int xc = xm - offsetX;
		int zc = zm - offsetZ;
		int yc = ym - offsetY;
		taskYIELD();
	}
}


void vTaskMagnetometer(void *pv){
	while(1){
		//Takes the semaphore
		xSemaphoreTake(semMag, portMAX_DELAY);
		//Takes the semaphore
	//	xSemaphoreTake(mtx, portMAX_DELAY);
		//reads the magnetometer
		readMag(&xm, &ym, &zm);
		//Gives the semaphore back
	//	xSemaphoreGive(mtx);
		taskYIELD();
	}
}

/* This task initializes the TPM as well as calls the functions for the different movement styles
 * forwardMovement = move the left motors counter clockwise and the right motor clockwise at the same rate.
 * leftMovment = move the 2 motors clockwise at the same rate
 * rightMovement = move the 2 motors counter clockwise at the same rate
 * */
void vMovement(void*pv) {

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
		if(sensor1Input > 1700.0) {
			//stopMovement();
	    	/* sensor1 and sensor2 are within a certain voltage */
	    	if(sensor2Input > 1300.0) {
	    		// turn left
	    		leftMovment();
	    	/* sensor1 is within a certain voltage and sensor2 is not */
	    	} else if( sensor2Input < 1300.0) {
	    		// turn right
	    		rightMovment();
	    	}
		} else {
			// move forward
			forwardMovment();
		}
		vTaskDelay( pdMS_TO_TICKS(500) );
    	taskYIELD();
    }
}

void forwardMovment(void) {
    TPM2->CONTROLS[0].CnV=1900;
    TPM0->CONTROLS[3].CnV=1300;
}

void leftMovment(void) {
	while( 1 /* change to reading the Magnetometer */) {
	    TPM2->CONTROLS[0].CnV=1900;
	    TPM0->CONTROLS[3].CnV=1900;
	}
}

void rightMovment(void) {
	while( 1 /*offsetX > -2000*/) {
	    TPM2->CONTROLS[0].CnV=1300;
	    TPM0->CONTROLS[3].CnV=1300;
	}
}
void stopMovement(void) {
	TPM2->CONTROLS[0].CnV=1475;
	TPM0->CONTROLS[3].CnV=1450;
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
    	//Takes the semaphore
    	double dataChanel1[5];
    	double dataChanel2[5];

    	for(int i=0; i<5; i++){
    		//Sensor #1 initialization
    		ADC0->SC2 = 1; // select correct voltage reference
    		ADC0->CFG1 = ADC_CFG1_MODE(3); // 16-bit conversions
    		ADC0->CFG2= ADC_CFG2_MUXSEL(0);

    		//Sensor #1
    		ADC0->SC1[0] = ADC_SC1_ADCH(0b111); // conversion on specified channel
    		dataChanel1[i] = (double) (ADC0->R[0] * 3123.0)/65536.0;

    		//PRINTF("chanel1 = %f\n", dataChanel1[i]);

    		//Sensor #2 initialization
    		ADC0->SC2 = 1; // select correct voltage reference
    		ADC0->CFG1 = ADC_CFG1_MODE(3); // 16-bit conversions
    		ADC0->CFG2= ADC_CFG2_MUXSEL(1);

    		//Sensor #2
    		ADC0->SC1[0] = ADC_SC1_ADCH(0b100);
    		dataChanel2[i] = (double) (ADC0->R[0] * 3123.0)/65536.0;

    		//PRINTF("chanel2 = %f\n", dataChanel1[i]);
    	}

    	sensor1Input = (dataChanel1[1] + dataChanel1[2] + dataChanel1[3] + dataChanel1[4] + dataChanel1[5]) / 5.0;
    	sensor2Input = (dataChanel2[1] + dataChanel2[2] + dataChanel2[3] + dataChanel2[4] + dataChanel2[5]) / 5.0;

    	//PRINTF("chanel1 average = %f\n", sensor1Input);
    	//PRINTF("chanel2 average = %f\n", sensor2Input);

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

void TPM0_IRQHandler(void){
	TPM1->SC |= TPM_SC_TOF_MASK;
	BaseType_t woken = pdFALSE;
	xSemaphoreGiveFromISR(semMag,0);
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
    semMag = xSemaphoreCreateBinary();
    semMotors = xSemaphoreCreateBinary();

    //semMutx = xSemaphoreCreateMutex();
    mtx = xSemaphoreCreateMutex();
    __enable_irq();
  
    xTaskCreate(vTaskMagnetometer, "TASK Magnetometer", 300,0,0,NULL);
    xTaskCreate(vTaskComData, "TASK Computing Data", 300,1,0,NULL);
    xTaskCreate(vTaskConverter, "Task Converter", 300,1,0,NULL);
    xTaskCreate(vMovement, "Move Forward", 300, 1, 0, NULL);
    vTaskStartScheduler();

    /* Enter an infinite loop, just incrementing a counter. */  
    while(1) {	}  
  return 0 ;
}
