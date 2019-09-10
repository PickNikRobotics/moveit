// ROS
#include <ros/ros.h>

// Testing
#include <gtest/gtest.h>

// Main class
#include <moveit/moveit_cpp/moveit_cpp.h>

namespace moveit
{
namespace planning_interface
{
class MoveItCppTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    nh_ptr = std::make_unique<ros::NodeHandle>("~");
    moveit_cpp_ptr = std::make_shared<MoveitCpp>(*nh_ptr);
  }

  void TearDown() override
  {
  }

protected:
  std::unique_ptr<ros::NodeHandle> nh_ptr;
  MoveitCppPtr moveit_cpp_ptr;
};

// TODO(JafarAbdi) delete
TEST_F(MoveItCppTest, DummyTest)
{
  ASSERT_EQ(1, 1);
}
}  // namespace planning_interface
}  // namespace moveit

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "MoveItCpp_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  int result = RUN_ALL_TESTS();

  spinner.stop();
  ros::shutdown();
  return result;
}
