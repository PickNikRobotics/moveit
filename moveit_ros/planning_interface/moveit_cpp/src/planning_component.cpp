/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Henning Kayser */

#include <stdexcept>
#include <sstream>
#include <memory>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <moveit_msgs/PlaceAction.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit_msgs/QueryPlannerInterfaces.h>
#include <moveit_msgs/GetCartesianPath.h>
#include <moveit_msgs/GraspPlanning.h>
#include <moveit_msgs/GetPlannerParams.h>
#include <moveit_msgs/SetPlannerParams.h>

#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <ros/console.h>
#include <ros/ros.h>

namespace moveit
{
namespace planning_interface
{
constexpr char LOGNAME[] = "planning_component";

PlanningComponent::PlanningComponent(const std::string& group_name, const MoveitCppPtr& moveit_context)
  : group_name_(group_name), nh_(moveit_context->getNodeHandle()), moveit_cpp_(moveit_context)
{
  joint_model_group_ = moveit_cpp_->getRobotModel()->getJointModelGroup(group_name);
  if (!joint_model_group_)
  {
    std::string error = "Could not find joint model group '" + group_name + "'.";
    ROS_FATAL_STREAM_NAMED(LOGNAME, error);
    throw std::runtime_error(error);
  }
  planning_pipeline_names_ = moveit_cpp_->getPlanningPipelineNames(group_name);
}

PlanningComponent::PlanningComponent(const std::string& group_name, const ros::NodeHandle& nh)
  : group_name_(group_name), nh_(nh), moveit_cpp_(new MoveitCpp(nh))
{
}

PlanningComponent::~PlanningComponent()
{
  ROS_INFO_NAMED(LOGNAME, "Deleting PlanningComponent '%s'", group_name_.c_str());
  clearContents();
}

PlanningComponent& PlanningComponent::operator=(PlanningComponent&& other)
{
  if (this != &other)
  {
    this->considered_start_state_ = other.considered_start_state_;
    this->workspace_parameters_ = other.workspace_parameters_;
    this->last_plan_solution_ = other.last_plan_solution_;
    other.clearContents();
  }
  return *this;
}

const std::vector<std::string> PlanningComponent::getNamedTargets()
{
  if (joint_model_group_)
  {
    return joint_model_group_->getDefaultStateNames();
  }
  else
  {
    ROS_WARN_NAMED(LOGNAME, "Unable to find joint group with name '%s'.", group_name_.c_str());
  }

  std::vector<std::string> empty;
  return empty;
}

const std::string& PlanningComponent::getName() const
{
  return group_name_;
}

bool PlanningComponent::plan(const PlanRequestParameters& parameters)
{
  if (!joint_model_group_)
  {
    ROS_ERROR_NAMED(LOGNAME, "Failed to retrieve joint model group for name '%s'.", group_name_.c_str());
    return false;
  }

  // Clone current planning scene
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor =
      moveit_cpp_->getPlanningSceneMonitorNonConst();
  planning_scene_monitor->updateFrameTransforms();
  planning_scene_monitor->lockSceneRead();  // LOCK planning scene
  planning_scene::PlanningScenePtr planning_scene =
      planning_scene::PlanningScene::clone(planning_scene_monitor->getPlanningScene());
  planning_scene_monitor->unlockSceneRead();  // UNLOCK planning scene
  planning_scene_monitor.reset();             // release this pointer

  // Init MotionPlanRequest
  ::planning_interface::MotionPlanRequest req;
  req.group_name = group_name_;
  req.planner_id = parameters.planner_id;
  req.allowed_planning_time = parameters.planning_time;
  req.max_velocity_scaling_factor = parameters.max_velocity_scaling_factor;
  req.max_acceleration_scaling_factor = parameters.max_acceleration_scaling_factor;
  if (workspace_parameters_set_)
    req.workspace_parameters = workspace_parameters_;

  // Set start state
  moveit::core::RobotStatePtr start_state = considered_start_state_;
  if (!start_state)
    start_state = moveit_cpp_->getCurrentState();
  moveit::core::robotStateToRobotStateMsg(*start_state, req.start_state);
  planning_scene->setCurrentState(*start_state);

  // Set goal constraints
  if (current_goal_constraints_.empty())
  {
    ROS_ERROR_NAMED(LOGNAME, "No goal constraints set for planning request");
    return false;
  }
  req.goal_constraints = current_goal_constraints_;

  // Run planning attempt
  ::planning_interface::MotionPlanResponse res;
  if (planning_pipeline_names_.find(parameters.planning_pipeline) == planning_pipeline_names_.end())
  {
    ROS_ERROR_NAMED(LOGNAME, "No planning pipeline available for name '%s'", parameters.planning_pipeline.c_str());
    return false;
  }
  const planning_pipeline::PlanningPipelinePtr pipeline =
      moveit_cpp_->getPlanningPipelines().at(parameters.planning_pipeline);
  pipeline->generatePlan(planning_scene, req, res);
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return false;
  }
  last_plan_solution_.reset(new PlanSolution());
  last_plan_solution_->start_state = req.start_state;
  last_plan_solution_->trajectory = res.trajectory_;
  // TODO(henningkayser): Visualize trajectory
  // std::vector<const moveit::core::LinkModel*> eef_links;
  // if (joint_model_group->getEndEffectorTips(eef_links))
  //{
  //  for (const auto& eef_link : eef_links)
  //  {
  //    ROS_INFO_STREAM("Publishing trajectory for end effector " << eef_link->getName());
  //    visual_tools_->publishTrajectoryLine(last_solution_trajectory_, eef_link);
  //    visual_tools_->publishTrajectoryPath(last_solution_trajectory_, false);
  //    visual_tools_->publishRobotState(last_solution_trajectory_->getLastWayPoint(), rviz_visual_tools::TRANSLUCENT);
  //  }
  //}
  return true;
}

bool PlanningComponent::plan()
{
  PlanRequestParameters default_parameters;
  default_parameters.planning_attempts = 1;
  default_parameters.planning_time = 5.0;
  default_parameters.max_velocity_scaling_factor = 1.0;
  default_parameters.max_acceleration_scaling_factor = 1.0;
  if (!planning_pipeline_names_.empty())
    default_parameters.planning_pipeline = *planning_pipeline_names_.begin();
  return plan(default_parameters);
}

void PlanningComponent::setStartState(const robot_state::RobotState& start_state)
{
  considered_start_state_.reset(new robot_state::RobotState(start_state));
}

robot_state::RobotStatePtr PlanningComponent::getStartState()
{
  if (considered_start_state_)
    return considered_start_state_;
  else
  {
    robot_state::RobotStatePtr s;
    moveit_cpp_->getCurrentState(s, 1.0);
    return s;
  }
}

void PlanningComponent::setStartStateToCurrentState()
{
  considered_start_state_.reset();
}

std::map<std::string, double> PlanningComponent::getNamedTargetValues(const std::string& name)
{
  // TODO(henningkayser): verify result
  std::map<std::string, double> positions;
  joint_model_group_->getVariableDefaultPositions(name, positions);
  return positions;
}

void PlanningComponent::setWorkspace(double minx, double miny, double minz, double maxx, double maxy, double maxz)
{
  workspace_parameters_.header.frame_id = moveit_cpp_->getRobotModel()->getModelFrame();
  workspace_parameters_.header.stamp = ros::Time::now();
  workspace_parameters_.min_corner.x = minx;
  workspace_parameters_.min_corner.y = miny;
  workspace_parameters_.min_corner.z = minz;
  workspace_parameters_.max_corner.x = maxx;
  workspace_parameters_.max_corner.y = maxy;
  workspace_parameters_.max_corner.z = maxz;
  workspace_parameters_set_ = true;
}

void PlanningComponent::unsetWorkspace()
{
  workspace_parameters_set_ = false;
}

bool PlanningComponent::setGoal(const std::vector<moveit_msgs::Constraints>& goal_constraints)
{
  current_goal_constraints_ = goal_constraints;
  return true;
}

bool PlanningComponent::setGoal(const robot_state::RobotState& goal_state)
{
  current_goal_constraints_ = { kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group_) };
  return true;
}

bool PlanningComponent::setGoal(const geometry_msgs::PoseStamped& goal_pose, const std::string& link_name)
{
  const auto& joint_names = joint_model_group_->getLinkModelNames();
  // if (std::find(joint_names.begin(), joint_names.end(), link_name) == joint_names.end())
  //{
  //  ROS_ERROR_NAMED(LOGNAME, "Link '%s' is not part of joint model group '%s'.", link_name.c_str(),
  //                  group_name_.c_str());
  //  return false;
  //}
  current_goal_constraints_ = { kinematic_constraints::constructGoalConstraints(link_name, goal_pose) };
  return true;
}

bool PlanningComponent::setGoal(const std::string& goal_state_name)
{
  const auto& named_targets = getNamedTargets();
  if (std::find(named_targets.begin(), named_targets.end(), goal_state_name) == named_targets.end())
  {
    ROS_ERROR_NAMED(LOGNAME, "No predefined joint state found for target name '%s'", goal_state_name.c_str());
    return false;
  }
  robot_state::RobotState goal_state(moveit_cpp_->getRobotModel());
  goal_state.setToDefaultValues(joint_model_group_, goal_state_name);
  return setGoal(goal_state);
}

bool PlanningComponent::execute(bool blocking)
{
  if (!last_plan_solution_)
  {
    ROS_ERROR_NAMED(LOGNAME, "There is no successfull plan to execute");
    return false;
  }

  // TODO(henningkayser): parameterize timestamps if required
  // trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  // if (!totg.computeTimeStamps(*last_solution_trajectory_, max_velocity_scaling_factor_,
  // max_acceleration_scaling_factor_))
  //{
  //  ROS_ERROR("Failed to parameterize trajectory");
  //  return false;
  //}
  moveit_cpp_->execute(group_name_, last_plan_solution_->trajectory, blocking);
}

void PlanningComponent::clearContents()
{
  considered_start_state_.reset();
  last_plan_solution_.reset();
  current_goal_constraints_.clear();
  moveit_cpp_.reset();
  planning_pipeline_names_.clear();
}
}  //  planning_interface
}  //  moveit
