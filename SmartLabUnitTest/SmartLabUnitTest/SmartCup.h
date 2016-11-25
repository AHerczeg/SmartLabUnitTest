#pragma once
#include <string>
#include "RestClient.h"

class SmartCup
{
public:

	SmartCup();

	int getMinute();

	int digitalRead();

	void setTemperature(float newTemperature);

	void setHumidity(float newHumidity);

	void setLight(float newLight);

	void setSound(float newSound);

	void setMotion(int newMotion);

	void setMinute(int newMinute);

	std::string loop(void);

	std::string getCoreID();

	int readWeatherSi7020();

	void readSi1132Sensor();

	float readSoundLevel();

	void setup();

	std::string getClientRequest(RestClient client);

	void resetValues();

	bool getSensorTriggered();

private:

};
