#include "perception_utils/logger.h"

using namespace suturo_perception;

Logger::Logger(std::string m) : module(m){}

void Logger::log(level lvl, std::string s)
{ 
	switch(lvl)
  {
    case DEBUG:
      std::cout << "[DEBUG] " << "[" << module << "]  " << s << std::endl;
    break;
    case INFO:
      std::cout << "[INFO] "  << "[" << module << "]  " << s << std::endl;
    break;
    case WARN:
      std::cout << "[WARN] "  << "[" << module << "]  " << s << std::endl;
    break;
    case ERROR:
      std::cout << "[ERROR] " << "[" << module << "]  " << s << std::endl;
    break;
  }
}

void Logger::logDebug(const std::string& s)
{
  log(DEBUG, s);
}

void Logger::logInfo(const std::string& s)
{
  log(INFO, s);
}

void Logger::logWarn(const std::string& s)
{
  log(WARN, s);
}

void Logger::logError(const std::string& s)
{
  log(ERROR, s);
}

// timelogging for profiling
void Logger::logTime(boost::posix_time::ptime s, boost::posix_time::ptime e, std::string text)
{
    boost::posix_time::time_duration d = e - s;
    float diff = (float)d.total_microseconds() / (float)1000;
    logInfo((boost::format("Time for %s: %s ms") % text % diff).str());
}

// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
