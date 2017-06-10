#ifndef ROS2_CONSOLE_CONSOLE_HPP
#define ROS2_CONSOLE_CONSOLE_HPP


// standard headers
#include <stdio.h>
#include <iostream>


namespace ros2_console
{

// Define colors
#ifdef WIN32
    #define COLOR_NORMAL ""
    #define COLOR_RED ""
    #define COLOR_GREEN ""
    #define COLOR_YELLOW ""
#else
    #define COLOR_NORMAL "\033[0m"
    #define COLOR_RED "\033[31m"
    #define COLOR_GREEN "\033[32m"
    #define COLOR_YELLOW "\033[33m"
#endif

// Define prefixes for each log level
#define DEBUG_PREFIX COLOR_GREEN "[DEBUG]: "
#define INFO_PREFIX COLOR_NORMAL "[ INFO]: "
#define WARN_PREFIX COLOR_YELLOW "[ WARN]: "
#define ERROR_PREFIX COLOR_RED   "[ERROR]: "
#define FATAL_PREFIX COLOR_RED   "[FATAL]: "
#define END_SUFFIX COLOR_NORMAL "\n"

// Regular log macros
#define ROS_DEBUG(format, ...) printf(DEBUG_PREFIX format END_SUFFIX, __VA_ARGS__);
#define ROS_INFO(format, ...) printf(INFO_PREFIX format END_SUFFIX, __VA_ARGS__);
#define ROS_WARN(format, ...) printf(WARN_PREFIX format END_SUFFIX, __VA_ARGS__);
#define ROS_ERROR(format, ...) printf(ERROR_PREFIX format END_SUFFIX, __VA_ARGS__);
#define ROS_FATAL(format, ...) printf(FATAL_PREFIX format END_SUFFIX, __VA_ARGS__);

// Named log macros
#define ROS_DEBUG_NAMED(name, format, ...) printf(DEBUG_PREFIX "[" name "]: " format END_SUFFIX, __VA_ARGS__);
#define ROS_INFO_NAMED(name, format, ...) printf(INFO_PREFIX "[" name "]: " format END_SUFFIX, __VA_ARGS__);
#define ROS_WARN_NAMED(name, format, ...) printf(WARN_PREFIX "[" name "]: " format END_SUFFIX, __VA_ARGS__);
#define ROS_ERROR_NAMED(name, format, ...) printf(ERROR_PREFIX "[" name "]: " format END_SUFFIX, __VA_ARGS__);
#define ROS_FATAL_NAMED(name, format, ...) printf(FATAL_PREFIX "[" name "]: " format END_SUFFIX, __VA_ARGS__);

// Conditional log macros
#define ROS_DEBUG_COND(cond, format, ...) \
  if (cond) { ROS_DEBUG(format, __VA_ARGS__); }
#define ROS_INFO_COND(cond, format, ...) \
  if (cond) { ROS_INFO(format, __VA_ARGS__); }
#define ROS_WARN_COND(cond, format, ...) \
  if (cond) { ROS_WARN(format, __VA_ARGS__); }
#define ROS_ERROR_COND(cond, format, ...) \
  if (cond) { ROS_ERROR(format, __VA_ARGS__); }
#define ROS_FATAL_COND(cond, format, ...) \
  if (cond) { ROS_FATAL(format, __VA_ARGS__); }

// Conditional and Named log macros
#define ROS_DEBUG_COND_NAMED(cond, name, format, ...)		\
  if (cond) { ROS_DEBUG_NAMED(name, format, __VA_ARGS__);}
#define ROS_INFO_COND_NAMED(cond, name, format, ...)		\
  if (cond) { ROS_INFO_NAMED(name, format, __VA_ARGS__); }
#define ROS_WARN_COND_NAMED(cond, name, format, ...)		\
  if (cond) { ROS_WARN_NAMED(name, format, __VA_ARGS__); }
#define ROS_ERROR_COND_NAMED(cond, name, format, ...)		\
  if (cond) { ROS_ERROR_NAMED(name, format, __VA_ARGS__); }
#define ROS_FATAL_COND_NAMED(cond, name, format, ...)		\
  if (cond) { ROS_FATAL_NAMED(name, format, __VA_ARGS__); }

// Throttle log macros
#define ROS_DEBUG_THROTTLE(rate, format, ...) ROS_DEBUG(format, __VA_ARGS__);
#define ROS_INFO_THROTTLE(rate, format, ...) ROS_INFO(format, __VA_ARGS__);
#define ROS_WARN_THROTTLE(rate, format, ...) ROS_WARN(format, __VA_ARGS__);
#define ROS_ERROR_THROTTLE(rate, format, ...) ROS_ERROR(format, __VA_ARGS__);
#define ROS_FATAL_THROTTLE(rate, format, ...) ROS_FATAL(format, __VA_ARGS__);

// Delayed throttle log macros
#define ROS_DEBUG_DELAYED_THROTTLE(rate, format, ...) ROS_DEBUG(format, __VA_ARGS__);
#define ROS_INFO_DELAYED_THROTTLE(rate, format, ...) ROS_INFO(format, __VA_ARGS__);
#define ROS_WARN_DELAYED_THROTTLE(rate, format, ...) ROS_WARN(format, __VA_ARGS__);
#define ROS_ERROR_DELAYED_THROTTLE(rate, format, ...) ROS_ERROR(format, __VA_ARGS__);
#define ROS_FATAL_DELAYED_THROTTLE(rate, format, ...) ROS_FATAL(format, __VA_ARGS__);

// Throttle named log macros
#define ROS_DEBUG_THROTTLE_NAMED(rate, name, format, ...) ROS_DEBUG_NAMED(name, format, __VA_ARGS__);
#define ROS_INFO_THROTTLE_NAMED(rate, name, format, ...) ROS_INFO_NAMED(name, format, __VA_ARGS__);
#define ROS_WARN_THROTTLE_NAMED(rate, name, format, ...) ROS_WARN_NAMED(name, format, __VA_ARGS__);
#define ROS_ERROR_THROTTLE_NAMED(rate, name, format, ...) ROS_ERROR_NAMED(name, format, __VA_ARGS__);
#define ROS_FATAL_THROTTLE_NAMED(rate, name, format, ...) ROS_FATAL_NAMED(name, format, __VA_ARGS__);

// Delayed throttle named log macros
#define ROS_DEBUG_DELAYED_THROTTLE_NAMED(rate, name, format, ...) ROS_DEBUG_NAMED(name, format, __VA_ARGS__);
#define ROS_INFO_DELAYED_THROTTLE_NAMED(rate, name, format, ...) ROS_INFO_NAMED(name, format, __VA_ARGS__);
#define ROS_WARN_DELAYED_THROTTLE_NAMED(rate, name, format, ...) ROS_WARN_NAMED(name, format, __VA_ARGS__);
#define ROS_ERROR_DELAYED_THROTTLE_NAMED(rate, name, format, ...) ROS_ERROR_NAMED(name, format, __VA_ARGS__);
#define ROS_FATAL_DELAYED_THROTTLE_NAMED(rate, name, format, ...) ROS_FATAL_NAMED(name, format, __VA_ARGS__);
  
// Stream log macros
#define ROS_DEBUG_STREAM(Msg) std::cerr << DEBUG_PREFIX << Msg << END_SUFFIX;
#define ROS_INFO_STREAM(Msg) std::cerr << INFO_PREFIX << Msg << END_SUFFIX;
#define ROS_WARN_STREAM(Msg) std::cerr << WARN_PREFIX << Msg << END_SUFFIX;
#define ROS_ERROR_STREAM(Msg) std::cerr << ERROR_PREFIX << Msg << END_SUFFIX;
#define ROS_FATAL_STREAM(Msg) std::cerr << FATAL_PREFIX << Msg << END_SUFFIX;

// Stream named log macros
#define ROS_DEBUG_STREAM_NAMED(name, Msg) ROS_DEBUG_STREAM("[" << name << "]: " << Msg);
#define ROS_INFO_STREAM_NAMED(name, Msg) ROS_INFO_STREAM("[" << name << "]: " << Msg);
#define ROS_WARN_STREAM_NAMED(name, Msg) ROS_WARN_STREAM("[" << name << "]: " << Msg);
#define ROS_ERROR_STREAM_NAMED(name, Msg) ROS_ERROR_STREAM("[" << name << "]: " << Msg);
#define ROS_FATAL_STREAM_NAMED(name, Msg) ROS_FATAL_STREAM("[" << name << "]: " << Msg);

// Stream conditional log macros
#define ROS_DEBUG_STREAM_COND(cond, Msg) \
  if (cond) { ROS_DEBUG_STREAM(Msg); }
#define ROS_INFO_STREAM_COND(cond, Msg) \
  if (cond) { ROS_INFO_STREAM(Msg); }
#define ROS_WARN_STREAM_COND(cond, Msg) \
  if (cond) { ROS_WARN_STREAM(Msg); }
#define ROS_ERROR_STREAM_COND(cond, Msg) \
  if (cond) { ROS_ERROR_STREAM(Msg); }
#define ROS_FATAL_STREAM_COND(cond, Msg) \
  if (cond) { ROS_FATAL_STREAM(Msg); }

// Stream conditional log and named macros
#define ROS_DEBUG_STREAM_COND_NAMED(cond, name, Msg) \
  if (cond) { ROS_DEBUG_STREAM("[" << name << "]: " << Msg); }
#define ROS_INFO_STREAM_COND_NAMED(cond, name, Msg)		\
  if (cond) { ROS_INFO_STREAM("[" << name << "]: " << Msg); }
#define ROS_WARN_STREAM_COND_NAMED(cond, name, Msg)		\
  if (cond) { ROS_WARN_STREAM("[" << name << "]: " << Msg); }
#define ROS_ERROR_STREAM_COND_NAMED(cond, name, Msg)		\
  if (cond) { ROS_ERROR_STREAM("[" << name << "]: " << Msg); }
#define ROS_FATAL_STREAM_COND_NAMED(cond, name, Msg)		\
  if (cond) { ROS_FATAL_STREAM("[" << name << "]: " << Msg); }

// Stream throttle log macros
#define ROS_DEBUG_STREAM_THROTTLE(rate, Msg) ROS_DEBUG_STREAM(Msg);
#define ROS_INFO_STREAM_THROTTLE(rate, Msg) ROS_INFO_STREAM(Msg);
#define ROS_WARN_STREAM_THROTTLE(rate, Msg) ROS_WARN_STREAM(Msg);
#define ROS_ERROR_STREAM_THROTTLE(rate, Msg) ROS_ERROR_STREAM(Msg);
#define ROS_FATAL_STREAM_THROTTLE(rate, Msg) ROS_FATAL_STREAM(Msg);

// Stream delayed throttle log macros
#define ROS_DEBUG_STREAM_DELAYED_THROTTLE(rate, Msg) ROS_DEBUG_STREAM(Msg);
#define ROS_INFO_STREAM_DELAYED_THROTTLE(rate, Msg) ROS_INFO_STREAM(Msg);
#define ROS_WARN_STREAM_DELAYED_THROTTLE(rate, Msg) ROS_WARN_STREAM(Msg);
#define ROS_ERROR_STREAM_DELAYED_THROTTLE(rate, Msg) ROS_ERROR_STREAM(Msg);
#define ROS_FATAL_STREAM_DELAYED_THROTTLE(rate, Msg) ROS_FATAL_STREAM(Msg);

// Stream throttle and named log macros
#define ROS_DEBUG_STREAM_THROTTLE_NAMED(rate, name, Msg) ROS_DEBUG_STREAM_NAMED(name, Msg);
#define ROS_INFO_STREAM_THROTTLE_NAMED(rate, name, Msg) ROS_INFO_STREAM_NAMED(name, Msg);
#define ROS_WARN_STREAM_THROTTLE_NAMED(rate, name, Msg) ROS_WARN_STREAM_NAMED(name, Msg);
#define ROS_ERROR_STREAM_THROTTLE_NAMED(rate, name, Msg) ROS_ERROR_STREAM_NAMED(name, Msg);
#define ROS_FATAL_STREAM_THROTTLE_NAMED(rate, name, Msg) ROS_FATAL_STREAM_NAMED(name, Msg);

// Stream delayed throttle and named log macros
#define ROS_DEBUG_STREAM_DELAYED_THROTTLE_NAMED(rate, name, Msg) ROS_DEBUG_STREAM_NAMED(name, Msg);
#define ROS_INFO_STREAM_DELAYED_THROTTLE_NAMED(rate, name, Msg) ROS_INFO_STREAM_NAMED(name, Msg);
#define ROS_WARN_STREAM_DELAYED_THROTTLE_NAMED(rate, name, Msg) ROS_WARN_STREAM_NAMED(name, Msg);
#define ROS_ERROR_STREAM_DELAYED_THROTTLE_NAMED(rate, name, Msg) ROS_ERROR_STREAM_NAMED(name, Msg);
#define ROS_FATAL_STREAM_DELAYED_THROTTLE_NAMED(rate, name, Msg) ROS_FATAL_STREAM_NAMED(name, Msg);
  

}  // end of the ros2_console namespace


#endif // ROS2_CONSOLE_CONSOLE_HPP
