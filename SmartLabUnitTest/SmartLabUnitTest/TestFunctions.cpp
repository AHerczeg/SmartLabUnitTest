#include "TestFunctions.h"
#include <stdio.h>
#include <string>
#include "SensorReader.h"
#include <iostream>
#include "time.h"


TestFunctions::TestFunctions()
{
}

int TestFunctions::sensorTest()
{

	SensorReader sr;
	bool passed = true;
	std::string failed;

	printf("Sensor test started\n");
	printf("\n    Standard values: ");

	
	for (int temp = -10; temp < 40; temp+=5)
	{
		sr.setTemperature(temp);
		for (int hum = 0; hum < 100; hum+=10)
		{
			sr.setHumidity(hum);
			for (int light = 0; light < 100; light+=10)
			{
				sr.setLight(light);
				for (float sound = 0; sound < 3.3; sound+=0.3)
				{
					sr.setSound(sound);
					for (int motion = 1; motion >= 0; motion--)
					{
						sr.setMotion(motion);
						std::string loop = sr.loop();
						if (loop != "{\"CoreID\":\"CoreId001\", \"Motion\": " + std::to_string(motion) + ", \"Temp\":" + std::to_string(temp) + ".000000, \"Humidity\":" + std::to_string(hum) + ".000000, \"Light\":" + std::to_string(light) + ".000000, \"Sound\":" + std::to_string(sound) + "}")
						{
							passed = false;
							printf("ERROR: ");
							std::cout << loop + "\n";
							break;
						}
						sr.resetValues();
					}
				}
			}
		}
	}
	
	if (passed)
		printf("Passed");
	else
		printf("Failed");
	passed = true;

	printf("\n\n    Random values: ");


	sr.resetValues();

	srand(time(NULL));

	for (int i = 0; i < 1000; i++)
	{

		float temp = -20.0 + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (70.0)));
		float hum = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (100.0)));
		float light = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (100.0)));
		float sound = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (100.0)));

		sr.setTemperature(temp);
		sr.setHumidity(hum);
		sr.setLight(light);
		sr.setSound(sound);
		std::string loop = sr.loop();

		if (loop != "{\"CoreID\":\"CoreId001\", \"Temp\":" + std::to_string(temp) + ", \"Humidity\":" + std::to_string(hum) + ", \"Light\":" + std::to_string(light) + ", \"Sound\":" + std::to_string(sound) + "}")
		{
			std::cout << loop + "\n";
			passed = false;
		}
		sr.resetValues();
	}

	if (passed)
		printf("Passed");
	else
		printf("Failed");

	passed = true;

	printf("\n\n    Threshold values: ");

	float temp = 0;
	float hum = 0;
	float light = 0;
	float sound = 0;

	sr.setTemperature(temp);
	sr.setHumidity(hum);
	sr.setLight(light);
	sr.setSound(sound);

	float oldTemp = 0;
	float oldHum = 0;
	float oldLight = 0;
	float oldSound = 0;

	sr.loop();

	for (int i = 0; i < 1000; i++)
	{
		if (i == 17)
			i = 17;
		temp += -1.0 + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (2.0)));
		hum += -1.0 + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (2.0)));
		light += -10.0 + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (20.0)));
		sound += -10.0 + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (20.0)));
		
		if (hum < 0)
			hum = 0;
		if (hum > 100)
			hum = 100;
		if (light < 0)
			light = 0;
		if (light > 100)
			light = 100;
		if (sound < 0)			
			sound = 0;
		if (sound > 100)
			sound = 100;

		sr.setTemperature(temp);
		sr.setHumidity(hum);
		sr.setLight(light);
		sr.setSound(sound);
		sr.loop();
		if (sr.getSensorTriggered())
		{
			bool correctCall = false;
			if (abs(oldTemp - temp) > (0.5))
			{
				correctCall = true;
				oldTemp = temp;
			}
			if (abs(oldHum - hum) > (0.5))
			{
				correctCall = true;
				oldHum = hum;
			}
			if (abs(oldLight - light) > (0.5 * 10))
			{
				correctCall = true;
				oldLight = light;
			}
			if (abs(oldSound*1000 - sound*1000) > (0.5 * 46))
			{
				correctCall = true;
				oldSound = sound;
			}
			passed = correctCall;
			if (!passed)
				break;
		}
	}

	if (passed)
		printf("Passed");
	else
		printf("Failed");

	printf("\n\nSensor test finished\n\n");

	return 1;
}

int TestFunctions::regularTest()
{	
	SensorReader sr;
	int counter = 0;

	printf("Regular test started \n");

	printf("\n    Regular test: ");

	sr.setMinute(27);
	sr.loop();
	for (int min = 0; min < 60; min++)
	{
		for (int i = 0; i < 10; i++)
		{
			sr.setMinute(min);
			if (sr.loop() != "")
				counter++;
		}
	}
	if (counter == 2)
		printf("Passed");
	else
		printf("Failed");
	printf("\n\nRegular test finished");

	return 1;

}
