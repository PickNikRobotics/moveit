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


/* Author: Simon Goldstien, Henning Kayser */

#include <stdexcept>
#include <moveit/moveit_cpp/moveit_core.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/common_planning_interface_objects/common_objects.h>

#include <std_msgs/String.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <ros/console.h>
#include <ros/ros.h>

namespace moveit
{
namespace planning_interface
{
constexpr char LOGNAME[] = "moveit_cpp";
MoveItCpp::MoveItCpp(const std::string& group_name,
                                                 const std::shared_ptr<tf2_ros::Buffer>& tf_buffer)
  : MoveItCpp(Options(group_name), tf_buffer)
{
}

MoveItCpp::MoveItCpp(const Options& opt, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer)
  : node_handle_(opt.node_handle_), tf_buffer_(tf_buffer)
{
  // Initialize robot model
  robot_description_ = opt.robot_description_;
  robot_model_ = opt.robot_model_ ? opt.robot_model_ : getSharedRobotModel(opt.robot_description_);
  if (!robot_model_)
  {
    std::string error = "Unable to construct robot model. Please make sure all needed information is on the "
                        "parameter server.";
    ROS_FATAL_STREAM_NAMED(LOGNAME, error);
    throw std::runtime_error(error);
  }

  // Check if default planning group is specified
  if (!opt.default_group_name_.empty())
  {
    setDefaultGroup(opt.default_group_name_);
  }
  else
  {
    ROS_INFO_NAMED(LOGNAME, "No default planning group specified");
  }

  // Init current state monitor
  current_state_monitor_ = getSharedStateMonitor(robot_model_, tf_buffer_, node_handle_);
}

MoveItCpp::~MoveItCpp()
{
  clearContents();
}

MoveItCpp::MoveItCpp(MoveItCpp&& other)
{
  other.clearContents();
}

MoveItCpp& MoveItCpp::operator=(MoveItCpp&& other)
{
  if (this != &other)
  {
    this->default_group_name_ = other.default_group_name_;
    this->robot_description_ = other.robot_description_;
    this->node_handle_ = other.node_handle_;
    this->tf_buffer_ = other.tf_buffer_;
    this->robot_model_ = other.robot_model_;
    this->current_state_monitor_ = other.current_state_monitor_;
    other.clearContents();
  }

  return *this;
}

const std::string& MoveItCpp::getDefaultGroup() const
{
  return default_group_name_;
}

bool MoveItCpp::setDefaultGroup(const std::string& group_name)
{
  if(!robot_model_->hasJointModelGroup(group_name))
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Group '" << group_name << "' was not found. Nothing changed.");
    return false;
  }

  default_group_name_ = group_name;
  ROS_INFO_STREAM_NAMED(LOGNAME, "Ready to take commands for planning group " << group_name << ".");
  return true;
}

const std::vector<std::string> MoveItCpp::getNamedTargets()
{
  const robot_model::RobotModelConstPtr& robot = robot_model_;
  const std::string& group = default_group_name_;
  const robot_model::JointModelGroup* joint_group = robot->getJointModelGroup(group);

  if (joint_group)
  {
    return joint_group->getDefaultStateNames();
  }

  std::vector<std::string> empty;
  return empty;
}

robot_model::RobotModelConstPtr MoveItCpp::getRobotModel() const
{
  return robot_model_;
}

const ros::NodeHandle& MoveItCpp::getNodeHandle() const
{
  return node_handle_;
}


void MoveItCpp::clearContents()
{
  default_group_name_.clear();
  robot_description_.clear();
  robot_model_.reset();
  tf_buffer_.reset();
  current_state_monitor_.reset();
}
}  //  planning_interface
}  //  moveit
