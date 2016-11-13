#include <stdio.h>
#include "BASIC.h"

void main(void) {
	pioInit();
	char accData[6];
	char gyroData[6];

	i2cInit(300000);
	i2cClearBits();
	i2cSlaveAdr(0b0011001);
	while(!i2cRead(0b10101000, accData, 6)){};

	i2cClearBits();
	i2cSlaveAdr(0b1101011);
	while(!i2cRead(0b10101000, gyroData, 6)){};

	printf("accData = %d, %d, %d\n", ((accData[1] << 8) | accData[0]),
		((accData[3] << 8) | accData[2]),
		((accData[5] << 8) | accData[4]));

	printf("gyroData = %d, %d, %d\n", ((gyroData[1] << 8) | gyroData[0]),
		((gyroData[3] << 8) | gyroData[2]),
		((gyroData[5] << 8) | gyroData[4]));

	printf("accData = %d, %d, %d, %d, %d, %d\n", accData[0], accData[1],
		accData[2], accData[3], accData[4], accData[5]);

	printf("gyroData = %d, %d, %d, %d, %d, %d\n", gyroData[0], gyroData[1],
		gyroData[2], gyroData[3], gyroData[4], gyroData[5]);
}
