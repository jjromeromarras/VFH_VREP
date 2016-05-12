#include "MyLogger.h"
#include <fstream>

MyLogger::MyLogger()
{
	active = false;
}

bool MyLogger::Init()
{
	try
	{
		char *file_log4cpp_init = "log4cpp.properties";		
		log4cpp::PropertyConfigurator::configure(file_log4cpp_init);
		this->initial = true;
	}
	catch (log4cpp::ConfigureFailure& f)
	{
		std::cout << "Configure Problem" << f.what() << std::endl;
		this->initial = false;
		return false;
	}
	return true;
}

void  MyLogger::SetActive(bool value)
{
	this->active = value;
}

MyLogger::~MyLogger()
{
	log4cpp::Category::shutdown();
}

void MyLogger::LogDebug(const char * message)
{
	if (active && initial)
	{
		log4cpp::Category & myLogger = log4cpp::Category::getInstance("MyLogger");
		myLogger.debug(message);
	}
}

void MyLogger::LogInfo(const char * message)
{
	if (active && initial)
	{
		log4cpp::Category & myLogger = log4cpp::Category::getInstance("MyLogger");
		myLogger.info(message);
	}
}
