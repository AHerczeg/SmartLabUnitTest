#include <stdio.h>
#include "TestFunctions.h"

int main()
{
	printf("Test started \n");

	TestFunctions tf;

	tf.sensorTest();

	tf.regularTest();

	while (true){}
}