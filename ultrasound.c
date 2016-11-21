#include <stdio.h>
#include "BASIC.h"

void main(void) {
	pioInit();
	pinMode(21, OUTPUT);
	pinMode(20, INPUT);

	digitalWrite(21, 1);
	delayMicrosecs(10);
	digitalWrite(21, 0);

	while(!digitalRead(20)){};

	int time1 = getTime();
	while(digitalRead(20)){};
	int time2 = getTime();

	float range = (float) (time2 - time1) / 1000000 * 340 / 2;
	printf("range = %f\n", range);

	delayMicrosecs(60*1000);
}
