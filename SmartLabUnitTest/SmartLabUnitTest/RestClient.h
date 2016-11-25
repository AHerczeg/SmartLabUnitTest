#pragma once
#include <string>

class RestClient
{
public:
	RestClient(std::string string, int port);

	std::string post(const char* path, std::string string, std::string *);

	std::string request;

	std::string getRequest();

private:

};

