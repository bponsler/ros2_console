#ifndef ROS2_CONSOLE_ASSERT_HPP
#define ROS2_CONSOLE_ASSERT_HPP


#include <stdlib.h>


#define ROS_ASSERT(expr) if (!(expr)) { abort(); }
#define ROS_BREAK() abort();


#endif  // ROS2_CONSOLE_ASSERT_HPP
