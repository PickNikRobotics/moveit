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

/* Author: Simon Goldstein, Henning Kayser */

#ifndef MOVEIT_MOVEIT_CPP_MOVEIT_CPP_
#define MOVEIT_MOVEIT_CPP_MOVEIT_CPP_

#include <moveit/macros/class_forward.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <memory>
#include <tf2_ros/buffer.h>

namespace planning_scene_monitor
{
MOVEIT_CLASS_FORWARD(PlanningSceneMonitor);
}

namespace planning_pipeline
{
MOVEIT_CLASS_FORWARD(PlanningPipeline);
}

namespace plan_execution
{
MOVEIT_CLASS_FORWARD(PlanExecution);
MOVEIT_CLASS_FORWARD(PlanWithSensing);
}

namespace trajectory_execution_manager
{
MOVEIT_CLASS_FORWARD(TrajectoryExecutionManager);
}

namespace
{
enum ActiveTargetType
{
  JOINT,
  POSE,
  POSITION,
  ORIENTATION
};
}

namespace moveit
{
/** \brief Simple interface to MoveIt components */
namespace planning_interface
{
//TODO(henningkayser): Move to own header and share with MoveGroup
class MoveItErrorCode : public moveit_msgs::MoveItErrorCodes
{
public:
  MoveItErrorCode()
  {
    val = 0;
  }
  MoveItErrorCode(int code)
  {
    val = code;
  }
  MoveItErrorCode(const moveit_msgs::MoveItErrorCodes& code)
  {
    val = code.val;
  }
  explicit operator bool() const
  {
    return val == moveit_msgs::MoveItErrorCodes::SUCCESS;
  }
  bool operator==(const int c) const
  {
    return val == c;
  }
  bool operator!=(const int c) const
  {
    return val != c;
  }
};

MOVEIT_CLASS_FORWARD(MoveItCpp);

/** \class MoveItCpp move_group_interface.h moveit/planning_interface/move_group_interface.h

    \brief Client class to conveniently use the ROS interfaces provided by the move_group node.

    This class includes many default settings to make things easy to use. */
class MoveItCpp
{
public:
  /** \brief Specification of options to use when constructing the MoveItCpp class */
  struct Options
  {
    Options(const std::string& group_name = "FAKE", const std::string& desc = "robot_description",
            const ros::NodeHandle& node_handle = ros::NodeHandle())
      : default_group_name_(group_name), robot_description_(desc), node_handle_(node_handle)
    {
    }

    /// The group to construct the class instance for
    std::string default_group_name_;

    /// The robot description parameter name (if different from default)
    std::string robot_description_;

    /// Optionally, an instance of the RobotModel to use can be also specified
    robot_model::RobotModelConstPtr robot_model_;

    ros::NodeHandle node_handle_;
  };

  MOVEIT_STRUCT_FORWARD(Plan);

  /// The representation of a motion plan (as ROS messages)
  struct Plan
  {
    /// The full starting state used for planning
    moveit_msgs::RobotState start_state_;

    /// The trajectory of the robot (may not contain joints that are the same as for the start_state_)
    moveit_msgs::RobotTrajectory trajectory_;

    /// The amount of time it took to generate the plan
    double planning_time_;
  };

  /**
      \brief Construct a MoveItCpp instance call using a specified set of options \e opt.

      \param opt. A MoveItCpp::Options structure, if you pass a ros::NodeHandle with a specific callback queue,
     it has to be of type ros::CallbackQueue
        (which is the default type of callback queues used in ROS)
      \param tf_buffer. Specify a TF2_ROS Buffer instance to use. If not specified,
                        one will be constructed internally along with an internal TF2_ROS TransformListener
      \param wait_for_servers. Timeout for connecting to action servers. Zero time means unlimited waiting.
    */
  MoveItCpp(const Options& opt, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer = std::shared_ptr<tf2_ros::Buffer>());
  /**
      \brief Construct a client for the MoveGroup action for a particular \e group.

      \param tf_buffer. Specify a TF2_ROS Buffer instance to use. If not specified,
                        one will be constructed internally along with an internal TF2_ROS TransformListener
      \param wait_for_servers. Timeout for connecting to action servers. Zero time means unlimited waiting.
    */
  MoveItCpp(const std::string& group,
            const std::shared_ptr<tf2_ros::Buffer>& tf_buffer = std::shared_ptr<tf2_ros::Buffer>());

  ~MoveItCpp();

  /**
   * @brief This class owns unique resources (e.g. action clients, threads) and its not very
   * meaningful to copy. Pass by references, move it, or simply create multiple instances where
   * required.
   */
  MoveItCpp(const MoveItCpp&) = delete;
  MoveItCpp& operator=(const MoveItCpp&) = delete;

  MoveItCpp(MoveItCpp&& other);
  MoveItCpp& operator=(MoveItCpp&& other);

  /** \brief Get the names of the named robot states available as targets, both either remembered states or default
   * states from srdf */
  const std::vector<std::string> getNamedTargets();

  /** \brief Get the RobotModel object. */
  robot_model::RobotModelConstPtr getRobotModel() const;

  /** \brief Get the name of the default planning group */
  const std::string& getDefaultGroup() const;
  
  /** \brief Set the name of the default planning group */
  bool setDefaultGroup(const std::string& group_name);

  /** \brief Get the ROS node handle of this instance operates on */
  const ros::NodeHandle& getNodeHandle() const;

private:
  //  Core properties
  ros::NodeHandle node_handle_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string robot_description_;
  std::string default_group_name_;
  robot_model::RobotModelConstPtr robot_model_;
  planning_scene_monitor::CurrentStateMonitorPtr current_state_monitor_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  void clearContents();
};
}  // namespace planning_interface
}  // namespace moveit

#endif
