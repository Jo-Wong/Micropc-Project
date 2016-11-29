// spi.h
// Send 4 8-bit numbers over SPI

#ifndef _INCLUDE_SPI_
#define _INCLUDE_SPI_
////////////////////////////////////////////////
// #includes
////////////////////////////////////////////////
#include <stdio.h>
#include "EasyPIO.h"
////////////////////////////////////////////////
// Constants
////////////////////////////////////////////////
#define RESET_FPGA 25
#define SPI_SETTINGS  (1<<25) // LEN_LONG
////////////////////////////////////////////////
// Function Prototypes
////////////////////////////////////////////////
void writeOut(char* motor_numbers);
////////////////////////////////////////////////
// Main
////////////////////////////////////////////////

void writeMotor(char* motor_numbers) {
	spiInit(256000, 0);
	pinMode(RESET_FPGA, OUTPUT); //set to output
	writeOut(motor_numbers);
}

////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////
/*
char spiSendReceive32(uint32_t send){
    SPI0FIFO = send;            // send data to slave
    while(!SPI0CSbits.DONE);    // wait until SPI transmission complete
    return SPI0FIFO;            // return received data
}
*/
void writeOut(char* motor_numbers)
{
  // Send pulse per write 
  digitalWrite(RESET_FPGA, 0);
  digitalWrite(RESET_FPGA, 1);
  digitalWrite(RESET_FPGA, 0);
  
  for(int i=0; i<4; ++i)
  {
	spiSendReceive(motor_numbers[i]);
  }
}
#endif