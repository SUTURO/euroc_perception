#ifndef PERCEPTION_UTILS_LOGGER_H
#define PERCEPTION_UTILS_LOGGER_H

#include <iostream>
#include <string>
#include "boost/date_time/posix_time/posix_time.hpp"

#ifndef HAVE_NO_ROS
#include "ros/ros.h"
#endif



namespace suturo_perception
{
  class Logger
  {
    private:
      std::string module;
      enum level {DEBUG, INFO, WARN, ERROR};
      void log(level, std::string s);

    public:
      // Constructor to instantiate a logger for a module
      Logger(){module = "";};
      Logger(std::string moduleName);
      /**
       * log methods for different levels
       */
      void logDebug(const std::string& s);
      void logInfo (const std::string& s);
      void logWarn (const std::string& s);
      void logError(const std::string& s);
      // time logging helper
      void logTime(boost::posix_time::ptime s, boost::posix_time::ptime e, std::string text);
     
  };
}
#endif
// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
