// test.c
// Send 4 8-bit numbers over SPI
////////////////////////////////////////////////

// #includes
#include "BASIC.h"

// Constants
#define RESET_FPGA 25
#define SPI_SETTINGS  (1<<25) // LEN_LONG

// Function Prototypes
void setup_ESCs(void);
void reset_ESCs(void);
void writeOut(char* motor_numbers);

// Functions
void setup_ESCs(void)
{
	char motor_numbers[4] = {0, 0, 0, 0};
	pioInit();
	spiInit(256000, 0);
	pinMode(RESET_FPGA, OUTPUT);
	writeOut(motor_numbers);
}

void reset_ESCs(void)
{
	char motor_numbers[4] = {255, 255, 255, 255};
	char motor_numbers2[4] = {0, 0, 0, 0};

	writeOut(motor_numbers);
	delayMicrosecs(200000); // wait 2 seconds
	writeOut(motor_numbers2);	
}

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