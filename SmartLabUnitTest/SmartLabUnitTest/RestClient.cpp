#include "RestClient.h"
#include <string>


RestClient::RestClient(std::string, int port)
{
	request = "";
}


std::string RestClient::post(const char* path, std::string string, std::string * response)
{
	request = string;
	return string;
}

std::string RestClient::getRequest()
{
	return request;
}
