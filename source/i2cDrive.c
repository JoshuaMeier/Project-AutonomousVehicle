/*
 * "And he commanded us to preach unto the people, and to testify that it is he which was ordained of
 * God to be the Judge of quick and dead. To him give all the prophets witness, that through his name
 * whosoever believeth in him shall receive remission of sins." (Act 10:42-43)
 *
 * This file provides several functions for reading the magnetometer and the accelerometer of FRDM-KL43.
 * The algorithm uses a partial implementation of I2C; more work may be needed for transferring data on
 * the I2C at a higher rate.
 */

#include <stdio.h>
#include <math.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL43Z4.h"
#include "fsl_debug_console.h"

#define MAGADDRESS 		0x0E	// 0xE according to the sensor manual, but 0x1E according to KL43 manual
#define ACCELADDRESS	0x1D

#define I2C_ERROR 		1
#define I2C_UNDEFINED	3
#define I2C_WRITE 		2
#define I2C_READ		4
#define I2C_SUCCESS 	8
#define I2C_START		16

#define BUFSIZE 16
static uint8_t bytes2write, bytes2read, writeCount, readCount, writeBuffer[BUFSIZE], readBuffer[BUFSIZE];

static uint8_t state;

static int init = 0;
static int initMag = 0;
static int initAcc = 0;

void reinit() {
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;
    PORTE->PCR[24] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[25] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[24] |= PORT_PCR_MUX(5); // select I2C0_SCL
    PORTE->PCR[25] |= PORT_PCR_MUX(5); // select I2C0_SDA
    NVIC_EnableIRQ(I2C0_IRQn);
    NVIC_SetPriority(I2C0_IRQn, 1);
    __enable_irq();
}

void initI2C() {
	if(!init) {
		reinit();
		init = 1;
	}
}

#define I32(x)	( (int32_t)( *(int16_t*)(x) ) )

int IR32(uint8_t *x) { // same as I32, but for the opposite endianness
	uint8_t y[2];
	y[0] = x[1]; y[1] = x[0];
	return I32(y);
}

int i2cTransfer(uint8_t *write, uint8_t writeNum, uint8_t *read, uint8_t readNum) {
	int i;
	if(writeNum <= 1 || !write)
		return I2C_ERROR; // at least 2 bytes are expected
	else
		bytes2write = writeNum;
	if(readNum <= 0)
		bytes2read = 0;
	else
		bytes2read = readNum;

	if(bytes2read > BUFSIZE || bytes2write > BUFSIZE)
		return I2C_ERROR;

	for(i = 0; i < BUFSIZE && i < writeNum; i++)
		writeBuffer[i] = write[i];

    state = I2C_START; //I2C_WRITE;
    writeCount = 0; readCount = 0;
    I2C0->F = 0x1C; // 0x1C: the default setting is too fast for the magnetometer
    //I2C0->F = 0x33; // makes data transfer slow
    I2C0->C1 = I2C_C1_IICEN_MASK | I2C_C1_TX_MASK; // turn on I2C in transmission mode
    I2C0->S |= I2C_S_ARBL_MASK; // clear flags, if set
    I2C0->FLT |= I2C_FLT_STOPF_MASK | I2C_FLT_STARTF_MASK | I2C_FLT_SSIE_MASK; // clear flags and ensure START/STOP generates interrupt
    I2C0->S |= I2C_S_IICIF_MASK;
    I2C0->C1 |= I2C_C1_IICIE_MASK; // enable interrupts
    I2C0->C1 |= I2C_C1_MST_MASK ; // turn on master mode; this will also generate a START signal
    //I2C0->D = writeBuffer[0]; // send the first data byte

    // Wait for data transfer to complete
    while(state != I2C_SUCCESS && !(state & 1));
    I2C0->C1 &= ~I2C_C1_IICIE_MASK; // turn off I2C interrupts; this is actually important; somehow snprintf causes problems without this line (?!)

    if(state == I2C_SUCCESS && read && readCount)
    	for(i = 0; i < readCount && i <BUFSIZE; i++)
    		read[i] = readBuffer[i];

    return state;
}

/*
 * Sets the offset of the magnetometer. Expects an array of 3 elements listing the offset for x, y, and z in this order.
 * How to determine offsets: offsets in the x, y, z directions: measure Xmax, Xmin (flip board),
 * take offset[0] = average, shift one position to the left (see format of offset registers)
 */

int magOffset(int16_t *offset) {
	int ret;
	uint8_t buf[BUFSIZE];
	uint8_t devadd = MAGADDRESS;

	initI2C();

    buf[0] = (devadd << 1); // the address of the magnetometer + 0 (write)
    buf[1] = 0x10; // access CTRL_REG1
    buf[2] = 0x01 | 1<<3 | 1<<4; // write 1 to this register to put the device in active mode and average 128 samples
    //buf[3] = 0x11; // access CTRL_REG2
    buf[3] = 0x80; // bit 7 = 1 --> enable auto reset in CTRL_REG2
    i2cTransfer(buf, 4, 0, 0);

    ret = (state == I2C_SUCCESS);
    if(!ret) return ret;

    uint8_t* p = (uint8_t*) offset;

    buf[0] = (devadd << 1); // the address of the magnetometer + 0 (write)
    buf[1] = 0x09; // msb of X offset
    buf[2] = p[1];
    buf[3] = p[0];
    buf[4] = p[3];
    buf[5] = p[2];
    buf[6] = p[5];
    buf[7] = p[4];
    i2cTransfer(buf, 8, 0, 0);

    ret = (state == I2C_SUCCESS);
    initMag = 1;
    return ret;
}


int readMag(int *x, int *y, int *z) {
	int ret;
	uint8_t buf[BUFSIZE];

	if(!initMag) { // if the magnetometer has not been initialized
		int16_t offset[3];
		offset[0] = 0; offset[1] = 0; offset[2] = 0;
		magOffset(offset);
	}

	uint8_t devadd = MAGADDRESS;
	buf[0] = (devadd << 1); // the address of the magnetometer + 0 (write)
	buf[1] = 0x01; // access register at address 1 first
	buf[2] = (devadd << 1) | 1; // the address of the magnetometer + 1 (read)
	i2cTransfer(buf, 3, 0, 6);
	if(x) *x = IR32(readBuffer);
	if(y) *y = IR32(readBuffer+2);
	if(z) *z = IR32(readBuffer+4);

	ret = (state == I2C_SUCCESS);
    return ret;
}


int initAccel() {
	int ret;
	uint8_t buf[BUFSIZE];
	uint8_t devadd = ACCELADDRESS;

	initI2C();

    buf[0] = (devadd << 1); // the address of the magnetometer + 0 (write)
    buf[1] = 0x2a; // access CTRL_REG1
    buf[2] = 1; // write 1 to this register to put the device in active mode
    i2cTransfer(buf, 3, 0, 0);

    ret = (state == I2C_SUCCESS);

	return ret;
}


int readAccel(int *x, int *y, int *z) {
	int ret;
	uint8_t buf[BUFSIZE];
	uint8_t devadd = ACCELADDRESS;

	if(!initAcc) {
		initAccel();
		initAcc = 1;
	}

	buf[0] = (devadd << 1); // the address of the magnetometer + 0 (write)
	buf[1] = 0x01; // access register at address 1 first
	buf[2] = (devadd << 1) | 1; // the address of the magnetometer + 1 (read)
	i2cTransfer(buf, 3, 0, 6);
	if(x) *x = IR32(readBuffer);
	if(y) *y = IR32(readBuffer+2);
	if(z) *z = IR32(readBuffer+4);

	ret = (state == I2C_SUCCESS);
    return ret;
}
/*
 * The following assumes that:
 * - START and STOP do not generate interrupt requests (FLT[SSIE] = 0).
 * - The first START has been generated and the first byte written to the data register.
 * - state == I2C_WRITE
 *
 * 1) Write all remaining data bytes (an interrupt is received when all previous bytes were written)
 */
void I2C0_IRQHandler() {

	if(I2C0->FLT & I2C_FLT_STOPF_MASK)
		I2C0->FLT |= I2C_FLT_STOPF_MASK;
	if(I2C0->FLT & I2C_FLT_STARTF_MASK) {
		I2C0->FLT |= I2C_FLT_STARTF_MASK;
		if(state != I2C_START)
			state = I2C_ERROR;
	}

	if(I2C0->S & I2C_S_ARBL_MASK) { // only one master, arbitration loss should never happen!
		state = I2C_ERROR;
		I2C0->S |= I2C_S_ARBL_MASK;
		I2C0->C1 &= ~I2C_C1_MST_MASK; // leave master mode
		I2C0->S |= I2C_S_IICIF_MASK; // clear the flag
		return;
	}

	I2C0->S |= I2C_S_IICIF_MASK; // clear the flag

	if(state == I2C_WRITE || state == I2C_START) { // if sending bytes
		if(state == I2C_WRITE && writeCount == bytes2write - 1 && bytes2read > 0) { // send restart signal only if not already sent
			I2C0->C1 |= I2C_C1_RSTA_MASK;
			state = I2C_START;
			return;
			//I2C0->D = writeBuffer[2]; // this will also clear TCF
			//writeCount++;
		}
		state = I2C_WRITE;
		if(writeCount == bytes2write && bytes2read > 0) {
			state = I2C_READ;
			I2C0->C1 &= ~I2C_C1_TX_MASK; // select receive
			readCount = 0;
			if(bytes2read == 1)
				I2C0->C1 |= I2C_C1_TXAK_MASK; // master should send NAK when the byte is received
			readBuffer[0] = I2C0->D; // initiate a read cycle
		}
		else if(writeCount < bytes2write) {
			I2C0->D = writeBuffer[writeCount]; // this will also clear TCF
			writeCount++;
		}
		else if(writeCount == bytes2write) {
			I2C0->C1 &= ~I2C_C1_MST_MASK; // send a stop signal
			state = I2C_SUCCESS;
		}
		else {
			state = I2C_UNDEFINED;
		}
	}
	else if(state == I2C_READ) {
		if(readCount == bytes2read - 2)
			I2C0->C1 |= I2C_C1_TXAK_MASK; // send a NAK after the next byte is transmitted
		if(readCount < bytes2read - 1) {
			readBuffer[readCount] = I2C0->D; // this also initiates the next read cycle
			readCount++;
		}
		else {
			I2C0->C1 &= ~I2C_C1_MST_MASK; // send stop signal
			I2C0->C1 |= I2C_C1_TX_MASK; // select transmit
			readBuffer[bytes2read-1] = I2C0->D;
			readCount = bytes2read;
			state = I2C_SUCCESS;
		}
	}
	else
		state = I2C_UNDEFINED;
}

#define DET(x, y, z, u)	((x)*(u)-(y)*(z))
#define pi				3.141673464
#define RAD2DEG			180/pi

/*
 * Calculates u x v and writes the result to w
 */

void crossProduct(double *u, double *v, double *w) {
	w[0] = DET(u[1], u[2], v[1], v[2]);
	w[1] = DET(v[0], v[2], u[0], u[2]); // same as - DET(u[0], u[2], v[0], v[2]);
	w[2] = DET(u[0], u[1], v[0], v[1]);
}

void normalizeXYZ(double *v) {
	double mag;
	mag = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
	v[0] /= mag; v[1] /= mag; v[2] /= mag;
}


void rpyangsub(double *refX, double *refY, double *refZ, double *angles) {
	double phi;
	phi = angles[2];
	angles[2] = RAD2DEG*phi;
	angles[1] = RAD2DEG*atan2(-refZ[0], refY[0]*sin(phi) + refX[0]*cos(phi));
	angles[0] = RAD2DEG*atan2(-refY[2]*cos(phi) + refX[2]*sin(phi), refY[1]*cos(phi) - refX[1]*sin(phi));
}

/*
 * void findRPY(int xm, int ym, int zm, int xa, int ya, int za, double *angles, double *angles2);
 *
 * Finds the roll, pitch, yaw angles of the board with respect to the reference frame.
 * xm, ..., za are the magnetometer and accelerometer readings with respect to the frame
 * of the board. (The user may choose what is that frame.)
 * Assumes a reference frame with x in the North direction and z in the opposite direction of
 * gravity. There are two solutions, which are saved to 'angles' and 'angles2'.
 * The meaning of the solution is that angles[0,1,2] are the rotation angles about x, y, z
 * in degrees, that is, the roll, pitch, and yaw angles. Starting with the reference frame,
 * by performing rotation about z, y, and x, in this order, the orientation of the board is
 * obtained. If angles2 = 0, the second solution is not calculated.
 *
 * The algorithm is as follows: find the direction of the z axis of the reference frame
 * with respect to the frame of the board; it is in the opposite direction of gravity.
 * Then, take the cross product of the z axis direction and the direction of (xm, ym, zm).
 * The result is the direction of the y axis of the reference frame with respect to the
 * frame of the board. Then the direction of the x axis can be calculated. Then the
 * roll, pitch, and yaw angles are calculated.
 */

void findRPY(int xm, int ym, int zm, int xa, int ya, int za, double *angles, double *angles2) {
	if(!xm && !ym && !zm) return;
	if(!xa && !ya && !za) return;
	if(!angles) return;

	double refZ[3]; // direction of Z axis of reference frame
	refZ[0] = -xa; refZ[1] = -ya; refZ[2] = -za; normalizeXYZ(refZ);

	double refY[3], temp[3], refX[3];
	temp[0] = xm; temp[1] = ym; temp[2] = zm;
	crossProduct(refZ, temp, refY); normalizeXYZ(refY);
	crossProduct(refY, refZ, refX); // normalizeXYZ(refX) not needed, sice Y and Z are already normalized

	angles[2] = atan2(refY[0], refX[0]);
	if(angles2) { // calculate the second solution
		if(angles[2] < 0)  	angles2[2] += pi;
		else 				angles2[2] -= pi;
		rpyangsub(refX, refY, refZ, angles2);
	}
	rpyangsub(refX, refY, refZ, angles); // calculate the first solution
}
