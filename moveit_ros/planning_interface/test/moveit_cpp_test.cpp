// ROS
#include <ros/ros.h>

// Testing
#include <gtest/gtest.h>

// Main class
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

namespace moveit
{
namespace planning_interface
{
class MoveItCppTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    nh_ = ros::NodeHandle("~");
    moveit_cpp_ptr = std::make_shared<MoveitCpp>(nh_);
    // Initialize scene and planning components
    //    moveit_cpp_ptr->getPlanningSceneMonitor()->startWorldGeometryMonitor();
    //    moveit_cpp_ptr->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(500);
    //    moveit_cpp_ptr->getPlanningSceneMonitor()->setStateUpdateFrequency(500);

    planning_component_ptr = std::make_shared<PlanningComponent>("manipulator", moveit_cpp_ptr);
  }

protected:
  ros::NodeHandle nh_;
  MoveitCppPtr moveit_cpp_ptr;
  PlanningComponentPtr planning_component_ptr;
};

TEST_F(MoveItCppTest, GetCurrentStateTest)
{
  auto robot_model = moveit_cpp_ptr->getRobotModel();
  auto robot_state = std::make_shared<robot_state::RobotState>(robot_model);
  EXPECT_TRUE(moveit_cpp_ptr->getCurrentState(robot_state, 0.0));
}

TEST_F(MoveItCppTest, NameOfPlanningGroupTest)
{
  EXPECT_STREQ(planning_component_ptr->getName().c_str(), "manipulator");
}
}  // namespace planning_interface
}  // namespace moveit

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "moveit_cpp_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  int result = RUN_ALL_TESTS();

  spinner.stop();
  ros::shutdown();
  return result;
}
