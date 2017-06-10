// project headers
#include <ros2_console/console.hpp>

// gtest includes
#include <gtest/gtest.h>


TEST(TimeTestSuite, testRegular)
{
  ROS_DEBUG("Hey: %s %d", "there", 100);
  ROS_INFO("Hey: %s %d", "there", 100);
  ROS_WARN("Hey: %s %d", "there", 100);
  ROS_ERROR("Hey: %s %d", "there", 100);
  ROS_FATAL("Hey: %s %d", "there", 100);
}


TEST(TimeTestSuite, testRegularNamed)
{
  ROS_DEBUG_NAMED("test_name", "Hey: %s %d", "there", 100);
  ROS_INFO_NAMED("my_name", "Hey: %s %d", "there", 100);
  ROS_WARN_NAMED("their_name", "Hey: %s %d", "there", 100);
  ROS_ERROR_NAMED("your_name", "Hey: %s %d", "there", 100);
  ROS_FATAL_NAMED("ThisIsMyName", "Hey: %s %d", "there", 100);
}

TEST(TimeTestSuite, testRegularCond)
{
  ROS_DEBUG_COND(true, "Hey: %s %d", "there", 100);
  ROS_INFO_COND(false, "Hey: %s %d", "there", 100);
  ROS_WARN_COND(1, "Hey: %s %d", "there", 100);
  ROS_ERROR_COND(0, "Hey: %s %d", "there", 100);
  ROS_FATAL_COND(123, "Hey: %s %d", "there", 100);
}

TEST(TimeTestSuite, testRegularCondNamed)
{
  ROS_DEBUG_COND_NAMED(true, "a_name", "Hey: %s %d", "there", 100);
  ROS_INFO_COND_NAMED(false, "b_name", "Hey: %s %d", "there", 100);
  ROS_WARN_COND_NAMED(1, "name_c", "Hey: %s %d", "there", 100);
  ROS_ERROR_COND_NAMED(0, "name_d", "Hey: %s %d", "there", 100);
  ROS_FATAL_COND_NAMED(123, "name_e", "Hey: %s %d", "there", 100);
}

TEST(TimeTestSuite, testRegularThrottle)
{
  ROS_DEBUG_THROTTLE(3, "Hey: %s %d", "there", 100);
  ROS_INFO_THROTTLE(2, "Hey: %s %d", "there", 100);
  ROS_WARN_THROTTLE(5, "Hey: %s %d", "there", 100);
  ROS_ERROR_THROTTLE(10, "Hey: %s %d", "there", 100);
  ROS_FATAL_THROTTLE(5, "Hey: %s %d", "there", 100);
}

TEST(TimeTestSuite, testRegularDelayedThrottle)
{
  ROS_DEBUG_DELAYED_THROTTLE(3, "Hey: %s %d", "there", 100);
  ROS_INFO_DELAYED_THROTTLE(2, "Hey: %s %d", "there", 100);
  ROS_WARN_DELAYED_THROTTLE(5, "Hey: %s %d", "there", 100);
  ROS_ERROR_DELAYED_THROTTLE(10, "Hey: %s %d", "there", 100);
  ROS_FATAL_DELAYED_THROTTLE(5, "Hey: %s %d", "there", 100);
}

TEST(TimeTestSuite, testRegularThrottleNamed)
{
  ROS_DEBUG_THROTTLE_NAMED(3, "name", "Hey: %s %d", "there", 100);
  ROS_INFO_THROTTLE_NAMED(2, "name", "Hey: %s %d", "there", 100);
  ROS_WARN_THROTTLE_NAMED(5, "name", "Hey: %s %d", "there", 100);
  ROS_ERROR_THROTTLE_NAMED(10, "name", "Hey: %s %d", "there", 100);
  ROS_FATAL_THROTTLE_NAMED(5, "name", "Hey: %s %d", "there", 100);
}

TEST(TimeTestSuite, testRegularDelayedThrottleNamed)
{
  ROS_DEBUG_DELAYED_THROTTLE_NAMED(3, "name", "Hey: %s %d", "there", 100);
  ROS_INFO_DELAYED_THROTTLE_NAMED(2, "name", "Hey: %s %d", "there", 100);
  ROS_WARN_DELAYED_THROTTLE_NAMED(5, "name", "Hey: %s %d", "there", 100);
  ROS_ERROR_DELAYED_THROTTLE_NAMED(10, "name", "Hey: %s %d", "there", 100);
  ROS_FATAL_DELAYED_THROTTLE_NAMED(5, "name", "Hey: %s %d", "there", 100);
}

TEST(TimeTestSuite, testStream)
{
  ROS_DEBUG_STREAM("Hey: " << "there" << " " << 100);
  ROS_INFO_STREAM("Hey: " << "there" << " " << 100);
  ROS_WARN_STREAM("Hey: " << "there" << " " << 100);
  ROS_ERROR_STREAM("Hey: " << "there" << " " << 100);
  ROS_FATAL_STREAM("Hey: " << "there" << " " << 100);
}

TEST(TimeTestSuite, testStreamNamed)
{
  ROS_DEBUG_STREAM_NAMED("name1", "Hey: " << "there" << " " << 100);
  ROS_INFO_STREAM_NAMED("name2", "Hey: " << "there" << " " << 100);
  ROS_WARN_STREAM_NAMED("name3", "Hey: " << "there" << " " << 100);
  ROS_ERROR_STREAM_NAMED("name4", "Hey: " << "there" << " " << 100);
  ROS_FATAL_STREAM_NAMED("name5", "Hey: " << "there" << " " << 100);
}

TEST(TimeTestSuite, testStreamCond)
{
  ROS_DEBUG_STREAM_COND(0, "Hey: " << "there" << " " << 100);
  ROS_INFO_STREAM_COND(1, "Hey: " << "there" << " " << 100);
  ROS_WARN_STREAM_COND(true, "Hey: " << "there" << " " << 100);
  ROS_ERROR_STREAM_COND(false, "Hey: " << "there" << " " << 100);
  ROS_FATAL_STREAM_COND(123, "Hey: " << "there" << " " << 100);
}

TEST(TimeTestSuite, testStreamCondNamed)
{
  ROS_DEBUG_STREAM_COND_NAMED(0, "name5", "Hey: " << "there" << " " << 100);
  ROS_INFO_STREAM_COND_NAMED(1, "name4", "Hey: " << "there" << " " << 100);
  ROS_WARN_STREAM_COND_NAMED(true, "name3", "Hey: " << "there" << " " << 100);
  ROS_ERROR_STREAM_COND_NAMED(false, "name2", "Hey: " << "there" << " " << 100);
  ROS_FATAL_STREAM_COND_NAMED(123, "name1", "Hey: " << "there" << " " << 100);
}

TEST(TimeTestSuite, testStreamThrottle)
{
  ROS_DEBUG_STREAM_THROTTLE(9, "Hey: " << "there" << " " << 100);
  ROS_INFO_STREAM_THROTTLE(0, "Hey: " << "there" << " " << 100);
  ROS_WARN_STREAM_THROTTLE(4, "Hey: " << "there" << " " << 100);
  ROS_ERROR_STREAM_THROTTLE(3, "Hey: " << "there" << " " << 100);
  ROS_FATAL_STREAM_THROTTLE(123, "Hey: " << "there" << " " << 100);
}

TEST(TimeTestSuite, testStreamDelayedThrottle)
{
  ROS_DEBUG_STREAM_DELAYED_THROTTLE(9, "Hey: " << "there" << " " << 100);
  ROS_INFO_STREAM_DELAYED_THROTTLE(0, "Hey: " << "there" << " " << 100);
  ROS_WARN_STREAM_DELAYED_THROTTLE(4, "Hey: " << "there" << " " << 100);
  ROS_ERROR_STREAM_DELAYED_THROTTLE(3, "Hey: " << "there" << " " << 100);
  ROS_FATAL_STREAM_DELAYED_THROTTLE(123, "Hey: " << "there" << " " << 100);
}

TEST(TimeTestSuite, testStreamThrottleNamed)
{
  ROS_DEBUG_STREAM_THROTTLE_NAMED(9, "name", "Hey: " << "there" << " " << 100);
  ROS_INFO_STREAM_THROTTLE_NAMED(0, "name", "Hey: " << "there" << " " << 100);
  ROS_WARN_STREAM_THROTTLE_NAMED(4, "name", "Hey: " << "there" << " " << 100);
  ROS_ERROR_STREAM_THROTTLE_NAMED(3, "name", "Hey: " << "there" << " " << 100);
  ROS_FATAL_STREAM_THROTTLE_NAMED(123, "name", "Hey: " << "there" << " " << 100);
}

TEST(TimeTestSuite, testStreamDelayedThrottleNamed)
{
  ROS_DEBUG_STREAM_DELAYED_THROTTLE_NAMED(9, "name", "Hey: " << "there" << " " << 100);
  ROS_INFO_STREAM_DELAYED_THROTTLE_NAMED(0, "name", "Hey: " << "there" << " " << 100);
  ROS_WARN_STREAM_DELAYED_THROTTLE_NAMED(4, "name", "Hey: " << "there" << " " << 100);
  ROS_ERROR_STREAM_DELAYED_THROTTLE_NAMED(3, "name", "Hey: " << "there" << " " << 100);
  ROS_FATAL_STREAM_DELAYED_THROTTLE_NAMED(123, "name", "Hey: " << "there" << " " << 100);
}

// Run all tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
