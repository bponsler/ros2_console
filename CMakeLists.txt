cmake_minimum_required(VERSION 3.5)
project(ros2_console)

# Add support for C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# List of all necessary header directories
set(INCLUDE_DIRS
  include
  ${rclcpp_INCLUDE_DIRS}
)
include_directories(${INCLUDE_DIRS})

set(LIBS
  ${rclcpp_LIBRARIES}
)

#### ros2_time library
add_library(${PROJECT_NAME} SHARED
  src/console.cpp
)
target_link_libraries(${PROJECT_NAME} ${LIBS})
set(LIBS ${LIBS} ${PROJECT_NAME})


############################################################
# Define tests
#

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    
    ament_add_gtest(ros2_console_tests test/console_tests.cpp)
    target_link_libraries(ros2_console_tests ${LIBS})
endif()


############################################################
# Define installation rules
#

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

# Export include dependencies, directories, and libraries
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME} ${LIBS})

# Must be at the bottom
ament_package()
