// basic.h
// 06 October 2016 jowong@g.hmc.edu
// library for basic GPIO, system timer, and SPI operations

#ifndef _INCLUDE_BASIC_
#define _INCLUDE_BASIC_

#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>

/////////////////////////////////////////////////////////////////////
// Constants
/////////////////////////////////////////////////////////////////////

//GPIO FSEL Types
#define INPUT	0
#define OUTPUT	1
#define ALT0	4
#define ALT1	5
#define ALT2	6
#define ALT3	7
#define ALT4	3
#define ALT5	2

#define GPFSEL   ((volatile unsigned int *) (gpio + 0))
#define GPSET    ((volatile unsigned int *) (gpio + 7))
#define GPCLR    ((volatile unsigned int *) (gpio + 10))
#define GPLEV    ((volatile unsigned int *) (gpio + 13))
#define SPICS	 ((volatile unsigned int *) (spi + 0))
#define SPIFIFO	 ((volatile unsigned int *) (spi + 1))
#define SPICLK	 ((volatile unsigned int *) (spi + 2))
#define I2CCTRL	 ((volatile unsigned int *) (i2c + 0))
#define I2CSTAT	 ((volatile unsigned int *) (i2c + 1))
#define I2CDLEN	 ((volatile unsigned int *) (i2c + 2))
#define I2CA	 ((volatile unsigned int *) (i2c + 3))
#define I2CFIFO	 ((volatile unsigned int *) (i2c + 4))
#define I2CCLK	 ((volatile unsigned int *) (i2c + 5))

// Physical addresses
#define BCM2836_PERI_BASE        0x3F000000
#define GPIO_BASE               (BCM2836_PERI_BASE + 0x200000)
#define TIME_BASE		(BCM2836_PERI_BASE + 0x3000)
#define SPI_BASE		(BCM2836_PERI_BASE + 0x204000)
#define I2C_BASE		(BCM2836_PERI_BASE + 0x804000)
#define BLOCK_SIZE (4*1024)
#define TIME_BLOCK_SIZE	34
#define SPI_BLOCK_SIZE 24
#define I2C_BLOCK_SIZE 24

// Pointers that will be memory mapped when pioInit() is called
volatile unsigned int *gpio; //pointer to base of gpio
volatile unsigned int *sys_timer; //pointer to base of system timer
volatile unsigned int *spi; // pointer to base of spi
volatile unsigned int *i2c; // pointer to base of i2c

void pioInit() {
	int  mem_fd;
	void *reg_map;

	// /dev/mem is a psuedo-driver for accessing memory in the Linux filesystem
	if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
	      printf("can't open /dev/mem \n");
	      exit(-1);
	}

	reg_map = mmap(
	  NULL,             //Address at which to start local mapping (null means don't-care)
      BLOCK_SIZE,       //Size of mapped memory block
      PROT_READ|PROT_WRITE,// Enable both reading and writing to the mapped memory
      MAP_SHARED,       // This program does not have exclusive access to this memory
      mem_fd,           // Map to /dev/mem
      GPIO_BASE);       // Offset to GPIO peripheral

	if (reg_map == MAP_FAILED) {
      printf("gpio mmap error %d\n", (int)reg_map);
      close(mem_fd);
      exit(-1);
    }

	gpio = (volatile unsigned *)reg_map;

	reg_map = mmap(
	  NULL,             //Address at which to start local mapping (null means don't-care)
      TIME_BLOCK_SIZE,       //Size of mapped memory block
      PROT_READ|PROT_WRITE,// Enable both reading and writing to the mapped memory
      MAP_SHARED,       // This program does not have exclusive access to this memory
      mem_fd,           // Map to /dev/mem
      TIME_BASE);       // Offset to system timer peripheral

	if (reg_map == MAP_FAILED) {
      printf("timer mmap error %d\n", (int)reg_map);
      close(mem_fd);
      exit(-1);
    }

	sys_timer = (volatile unsigned *)reg_map;

	reg_map = mmap(
	  NULL,             //Address at which to start local mapping (null means don't-care)
      SPI_BLOCK_SIZE,       //Size of mapped memory block
      PROT_READ|PROT_WRITE,// Enable both reading and writing to the mapped memory
      MAP_SHARED,       // This program does not have exclusive access to this memory
      mem_fd,           // Map to /dev/mem
      SPI_BASE);       // Offset to SPI peripheral

	if (reg_map == MAP_FAILED) {
      printf("spi mmap error %d\n", (int)reg_map);
      close(mem_fd);
      exit(-1);
    }

	spi = (volatile unsigned *)reg_map;

	reg_map = mmap(
	  NULL,             //Address at which to start local mapping (null means don't-care)
      I2C_BLOCK_SIZE,       //Size of mapped memory block
      PROT_READ|PROT_WRITE,// Enable both reading and writing to the mapped memory
      MAP_SHARED,       // This program does not have exclusive access to this memory
      mem_fd,           // Map to /dev/mem
      I2C_BASE);       // Offset to I2C peripheral

	if (reg_map == MAP_FAILED) {
      printf("i2c mmap error %d\n", (int)reg_map);
      close(mem_fd);
      exit(-1);
    }

	i2c = (volatile unsigned *)reg_map;
}

void pinMode(int pin, int function) {
	int reg = pin/10;
	int offset = pin%10 * 3;
	GPFSEL[reg] &= ~((0b111 & ~function) << offset);
	GPFSEL[reg] |= ((0b111 & function) << offset);
}

int digitalRead(int pin) {
	int reg = pin/32;
	int depth = pin%32;
	return (GPLEV[reg] >> depth) & 1;
}

void digitalWrite(int pin, int value) {
	int reg = pin/32;
	int depth = pin%32;
	if(value == 0) {
		GPCLR[reg] |= (0x1 << depth);
	} else {
		GPSET[reg] |= (value << depth);
	}
}

void delayMicrosecs(unsigned int micros) {
	sys_timer[5] = sys_timer[1]+micros;
	sys_timer[0] = 0b0100;
	while(!(sys_timer[0] & 0b0100));
}

int getTime() {
	return sys_timer[1];
}

void spiInit(int frequency, int settings) {
	pinMode(8, ALT0);
	pinMode(9, ALT0);
	pinMode(10, ALT0);
	pinMode(11, ALT0);
	SPICLK[0] = 250000000/frequency;
	SPICS[0] = settings;
	SPICS[0] |= 1 << 7;
}

char spiSendReceive(char send) {
	SPIFIFO[0] = send;
	while(!((SPICS[0] >> 16) & 1));
	return SPIFIFO[0];
}

short spiSendReceive2(short send) {
	short receive;
	SPICS[0] |= 1 << 7;
	receive = spiSendReceive(send);
	SPICS[0] &= 0xFFFFFF7F;
	return receive;
}

short spiSendReceive16(short send) {
	short receive;
	SPICS[0] |= 1 << 7;
	receive = spiSendReceive((send & 0xFF00) >> 8);
	receive = (receive << 8) | spiSendReceive(send & 0xFF);
	SPICS[0] &= 0xFFFFFF7F;
	return receive;
}

void i2cInit(int frequency) {
	pinMode(2, ALT0);
	pinMode(3, ALT0);
	I2CCLK[0] = 250000000/frequency;
}

void i2cSlaveAdr(char adr) {
	I2CA[0] = adr;
}

void i2cClearBits() {
	I2CCTRL[0] &= 0x00000020;
	I2CSTAT[0] &= 0x00000302;
}

int i2cPowerOn(char subadr, char value) {
	printf("%d\n", subadr);
	printf("%d\n", value);
	I2CDLEN[0] = 2;
	I2CCTRL[0] = 0x00008000;
	I2CFIFO[0] = subadr;
	I2CFIFO[0] = value;
	I2CCTRL[0] = 0x00008080;

/*	while(!(I2CSTAT[0] & 0x00000001)) {
		if(I2CSTAT[0] & 0x00000002) break;
	}

	if((I2CSTAT[0] & 0x00000001)) {
		I2CFIFO[0] = value;
	}
*/
	printf("%d\n", I2CSTAT[0]);
	return 1;
}

int i2cRead(char subadr, char * data, short numBytes) {
	int i = 0;
	int i2c_byte_wait_us = ((float)I2CCLK[0]/250000000)*9*1000000;

	//Set data length = 1
	I2CDLEN[0] = 1;
	// Enable BSC
	I2CCTRL[0] = 0x00008000;
	// Write subaddress to FIFO
	I2CFIFO[0] = subadr;
	// Enable BSC and ST: BSC necessary?
	I2CCTRL[0] = 0x00008080;


	// Poll TA until active
	while(!(I2CSTAT[0] & 0x00000001)) {
		//printf("entered\n");
		if(I2CSTAT[0] & 0x00000002)
			break;
		//printf("polling\n");
	}
	// Set data length = 6 bytes
	I2CDLEN[0] = numBytes;
	// Change to read, enable BSC and ST: BSC necessary?
	I2CCTRL[0] = 0x00008081;
	// Wait for bytes to start coming back
	delayMicrosecs(i2c_byte_wait_us * 3);
	// Collect data until DONE
/*	while(!(I2CSTAT[0] & 0x00000002)) {
		while(i < numBytes && (I2CSTAT[0] & 0x00000020)) {
			char c = *I2CFIFO;
			data[i] = c;
			i++;
		}
		if(I2CSTAT[0] & 0x00000100) {
			printf("failed to ack\n");
		}
		if(I2CSTAT[0] & 0x00000200) {
			printf("hold scl too long\n");
		}
	}
	// Collect remaining number of bytes after DONE
	while(i < numBytes && (I2CSTAT[0] & 0x00000020)) {
		char c = *I2CFIFO;
		data[i] = c;
		i++;
	}/*
*/	while(i < numBytes) {
		if(I2CSTAT[0] & 0x00000020) {
			//data[i] = I2CFIFO[0];
			char c = *I2CFIFO;
			data[i] = c;
			i++;
		}
	}
	int result = I2CSTAT[0];
	printf("%d\n", result);
	I2CSTAT[0] &= 0x00000002;
	return 1;
}

int printStuff() {
	return 3;
}
#endif
