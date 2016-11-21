#include <stdio.h>
#include "BASIC.h"

void main(void) {
	pioInit();
	char accData[6] = {1,2,3,4,5,6};
	char gyroData[6] = {7,8,9,10,11,12};

/*	i2cInit(200000);
	i2cClearBits();
	i2cSlaveAdr(0b0011001); //25 = 0011001
	i2cPowerOn(0b00100000, 0b00010111); //32 = 00100000, 23 = 00010111

	i2cInit(200000);
	i2cClearBits();
	i2cSlaveAdr(0b1101011); //25 = 0011001
	i2cPowerOn(0b00100000, 0b00001111); //32 = 00100000, 23 = 00010111
*/
	//while(1){
//	printf("gyro---\n");
	i2cInit(200000);
	i2cClearBits();
	i2cSlaveAdr(0b0011001);
	while(!i2cRead(0b10101000, accData, 6)){};

	i2cInit(200000);
	i2cClearBits();
	i2cSlaveAdr(0b1101011);
	while(!i2cRead(0b10101000, gyroData, 6)){};
	//}
	printf("accData = %d, %d, %d\n", ((accData[1] << 2) | accData[0] >> 6),
		((accData[3] << 2) | accData[2] >> 6),
		((accData[5] << 2) | accData[4] >> 6));

	printf("gyroData = %d, %d, %d\n", ((gyroData[1] << 2) | gyroData[0] >> 6),
		((gyroData[3] << 2) | gyroData[2] >> 6),
		((gyroData[5] << 2) | gyroData[4] >> 6));

	printf("accData = %d, %d, %d, %d, %d, %d\n", accData[0], accData[1],
		accData[2], accData[3], accData[4], accData[5]);

	printf("gyroData = %d, %d, %d, %d, %d, %d\n", gyroData[0], gyroData[1],
		gyroData[2], gyroData[3], gyroData[4], gyroData[5]);
}
