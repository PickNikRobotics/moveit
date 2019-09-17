// ROS
#include <ros/ros.h>

// Testing
#include <gtest/gtest.h>

// Main class
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
// Msgs
#include <geometry_msgs/PointStamped.h>

namespace moveit
{
namespace planning_interface
{
class MoveItCppTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    nh_ = ros::NodeHandle("/moveit_cpp_test");
    moveit_cpp_ptr = std::make_shared<MoveitCpp>(nh_);
    planning_component_ptr = std::make_shared<PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
    robot_model_ptr = moveit_cpp_ptr->getRobotModel();
    jmg_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);

    target_pose1.header.frame_id = "panda_link0";
    target_pose1.pose.orientation.w = 1.0;
    target_pose1.pose.position.x = 0.28;
    target_pose1.pose.position.y = -0.2;
    target_pose1.pose.position.z = 0.5;

    start_pose.orientation.w = 1.0;
    start_pose.position.x = 0.55;
    start_pose.position.y = 0.0;
    start_pose.position.z = 0.6;

    target_pose2.orientation.w = 1.0;
    target_pose2.position.x = 0.55;
    target_pose2.position.y = -0.05;
    target_pose2.position.z = 0.8;
  }

protected:
  ros::NodeHandle nh_;
  MoveitCppPtr moveit_cpp_ptr;
  PlanningComponentPtr planning_component_ptr;
  robot_model::RobotModelConstPtr robot_model_ptr;
  const moveit::core::JointModelGroup* jmg_ptr;
  const std::string PLANNING_GROUP = "panda_arm";
  geometry_msgs::PoseStamped target_pose1;
  geometry_msgs::Pose target_pose2;
  geometry_msgs::Pose start_pose;
};

TEST_F(MoveItCppTest, GetCurrentStateTest)
{
  ros::Duration(1.0).sleep();  // Otherwise joint_states will result in an invalid robot state
  auto robot_model = moveit_cpp_ptr->getRobotModel();
  auto robot_state = std::make_shared<robot_state::RobotState>(robot_model);
  EXPECT_TRUE(moveit_cpp_ptr->getCurrentState(robot_state, 0.0));
  //  std::vector<double> joints_vals;
  //  robot_state->copyJointGroupPositions(PLANNING_GROUP, joints_vals);
  //  EXPECT_NEAR(joints_vals[0], 0.0, 0.001);     // panda_joint1
  //  EXPECT_NEAR(joints_vals[1], -0.785, 0.001);  // panda_joint2
  //  EXPECT_NEAR(joints_vals[2], 0.0, 0.001);     // panda_joint3
  //  EXPECT_NEAR(joints_vals[3], -2.356, 0.001);  // panda_joint4
  //  EXPECT_NEAR(joints_vals[4], 0.0, 0.001);     // panda_joint5
  //  EXPECT_NEAR(joints_vals[5], 1.571, 0.001);   // panda_joint6
  //  EXPECT_NEAR(joints_vals[6], 0.785, 0.001);   // panda_joint7
}

// TODO(JafarAbdi) unnecessary
TEST_F(MoveItCppTest, NameOfPlanningGroupTest)
{
  EXPECT_STREQ(planning_component_ptr->getName().c_str(), "panda_arm");
}

TEST_F(MoveItCppTest, TestSetStartStateToCurrentState)
{
  planning_component_ptr->setStartStateToCurrentState();
  // TODO(JafarAbdi) shouldn't be here v
  planning_component_ptr->setGoal(target_pose1, "panda_link8");

  ASSERT_TRUE(planning_component_ptr->plan());
  // TODO(JafarAbdi) adding testing to the soln state
}

TEST_F(MoveItCppTest, TestSetGoalFromPoseStamped)
{
  planning_component_ptr->setStartStateToCurrentState();

  geometry_msgs::PoseStamped target_pose1;
  planning_component_ptr->setGoal(target_pose1, "panda_link8");

  ASSERT_TRUE(planning_component_ptr->plan());
}

TEST_F(MoveItCppTest, TestSetStartStateFromRobotState)
{
  auto start_state = *(moveit_cpp_ptr->getCurrentState());
  start_state.setFromIK(jmg_ptr, start_pose);

  planning_component_ptr->setStartState(start_state);
  planning_component_ptr->setGoal(target_pose1, "panda_link8");

  ASSERT_TRUE(planning_component_ptr->plan());
}

TEST_F(MoveItCppTest, TestSetGoalFromRobotState)
{
  auto target_state = *(moveit_cpp_ptr->getCurrentState());

  target_state.setFromIK(jmg_ptr, target_pose2);

  planning_component_ptr->setGoal(target_state);

  ASSERT_TRUE(planning_component_ptr->plan());
}
}  // namespace planning_interface
}  // namespace moveit

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "moveit_cpp_test");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  int result = RUN_ALL_TESTS();

  spinner.stop();
  ros::shutdown();
  return result;
}
