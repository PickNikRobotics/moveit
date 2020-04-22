/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Unbounded Robotics Inc.
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Michael Ferguson, Ioan Sucan, E. Gil Jones */

#include <moveit_simple_controller_manager/follow_joint_trajectory_controller_handle.h>
#include <moveit/utils/xmlrpc_casts.h>

// Trackjoint
#include <std_msgs/Float64MultiArray.h>
#include <trackjoint/trajectory_generator.h>

using namespace moveit::core;
static const std::string LOGNAME("SimpleControllerManager");

namespace moveit_simple_controller_manager
{
bool FollowJointTrajectoryControllerHandle::sendTrajectory(const moveit_msgs::RobotTrajectory& trajectory)
{
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "new trajectory to " << name_);

  if (!controller_action_client_)
    return false;

  if (!trajectory.multi_dof_joint_trajectory.points.empty())
  {
    ROS_WARN_NAMED(LOGNAME, "%s cannot execute multi-dof trajectories.", name_.c_str());
  }

  if (done_)
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "sending trajectory to " << name_);
  else
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "sending continuation for the currently executed trajectory to " << name_);

  control_msgs::FollowJointTrajectoryGoal goal = goal_template_;
  goal.trajectory = trajectory.joint_trajectory;

  ////////////////////////////////////////
  // Smooth the trajectory with TrackJoint
  ////////////////////////////////////////
  constexpr int kNumDof = 6;
  constexpr double kMaxDuration = 30;
  constexpr double kTimestep = 0.0039;  // Slightly faster than 250 Hz
  constexpr double kPositionTolerance = 1e-6;
  constexpr bool kUseHighSpeedMode = false;

  std::vector<trackjoint::Limits> limits(kNumDof);
  trackjoint::Limits single_joint_limits;
  single_joint_limits.velocity_limit = 3.15;  // To match value in joint_limits.yaml
  single_joint_limits.acceleration_limit = 5;
  single_joint_limits.jerk_limit = 10000;
  limits[0] = single_joint_limits;
  limits[1] = single_joint_limits;
  limits[2] = single_joint_limits;
  limits[3] = single_joint_limits;
  limits[4] = single_joint_limits;
  limits[5] = single_joint_limits;

  ////////////////////////////////////////////////
  // Get TrackJoint initial states and goal states
  ////////////////////////////////////////////////

  // Vector of start states - for each waypoint, for each joint
  std::vector<std::vector<trackjoint::KinematicState>> trackjt_current_joint_states;
  // Vector of goal states - for each waypoint, for each joint
  std::vector<std::vector<trackjoint::KinematicState>> trackjt_goal_joint_states;

  std::vector<double> trackjt_desired_durations;

  trackjoint::KinematicState joint_state;

  // For each MoveIt waypoint
  for (std::size_t point = 0; point<goal.trajectory.points.size()-1; ++point)
  {
    std::vector<trackjoint::KinematicState> current_joint_states;
    std::vector<trackjoint::KinematicState> goal_joint_states;

    // for each joint
    for (std::size_t joint = 0; joint<kNumDof; ++joint)
    {
      // Save the start state of the robot
      joint_state.position = goal.trajectory.points[point].positions[joint];
      joint_state.velocity = goal.trajectory.points[point].velocities[joint];
      joint_state.acceleration = goal.trajectory.points[point].accelerations[joint];

      current_joint_states.push_back(joint_state);

      // Save the goal state of the robot
      joint_state.position = goal.trajectory.points[point+1].positions[joint];
      joint_state.velocity = goal.trajectory.points[point+1].velocities[joint];
      joint_state.acceleration = goal.trajectory.points[point+1].accelerations[joint];

      goal_joint_states.push_back(joint_state);
    }
    trackjt_current_joint_states.push_back(current_joint_states);
    trackjt_goal_joint_states.push_back(goal_joint_states);

    trackjt_desired_durations.push_back( goal.trajectory.points[point+1].time_from_start.toSec() - goal.trajectory.points[point].time_from_start.toSec() );
  }

  ROS_WARN_STREAM("MoveIt's original num. waypoints: " << trackjt_desired_durations.size());

  /////////////////
  // Run TrackJoint
  /////////////////

  // Save the new trajectory in this message
  control_msgs::FollowJointTrajectoryGoal smoothed_goal;
  smoothed_goal.path_tolerance = goal.path_tolerance;
  smoothed_goal.goal_tolerance = goal.goal_tolerance;
  smoothed_goal.goal_tolerance = goal.goal_tolerance;
  smoothed_goal.goal_time_tolerance = goal.goal_time_tolerance;
  smoothed_goal.trajectory.header = goal.trajectory.header;
  smoothed_goal.trajectory.joint_names = goal.trajectory.joint_names;

  trackjoint::ErrorCodeEnum error_code = trackjoint::ErrorCodeEnum::kNoError;

  double waypoint_start_time = 0;
  std::vector<trackjoint::JointTrajectory> output_trajectories(kNumDof);

  // Step through the saved waypoints and smooth them with TrackJoint
  for (std::size_t point=0; point<trackjt_desired_durations.size(); ++point)
  {
    trackjoint::TrajectoryGenerator traj_gen(kNumDof, kTimestep, trackjt_desired_durations[point],
                                        kMaxDuration, trackjt_current_joint_states[point],
                                        trackjt_goal_joint_states[point], limits, kPositionTolerance,
                                        kUseHighSpeedMode);

    error_code = traj_gen.generateTrajectories(&output_trajectories);

    std::cout << "Error code: " << trackjoint::kErrorCodeMap.at(error_code)
          << std::endl;
    // Debug output, if failure
    if (error_code != trackjoint::ErrorCodeEnum::kNoError)
    {
      std::cout << "===" << std::endl;
      std::cout << "Failing conditions: " << std::endl;
      std::cout << "Timestep: " << kTimestep << std::endl;
      std::cout << "Desired duration: " << trackjt_desired_durations[point] << std::endl;
      std::cout << "Max duration: " << kMaxDuration << std::endl;
      std::cout << "Current joint positions / velocities / accelerations:" << std::endl;
      for (std::size_t joint = 0; joint<kNumDof; ++joint)
      {
        std::cout << trackjt_current_joint_states[point][joint].position
        << "  " << trackjt_current_joint_states[point][joint].velocity
        << "  " << trackjt_current_joint_states[point][joint].acceleration
        << std::endl;
      }
      std::cout << "Goal joint positions / velocities / accelerations:" << std::endl;
      for (std::size_t joint = 0; joint<kNumDof; ++joint)
      {
        std::cout << trackjt_goal_joint_states[point][joint].position
        << "  " << trackjt_goal_joint_states[point][joint].velocity
        << "  " << trackjt_goal_joint_states[point][joint].acceleration
        << std::endl;
      }
      std::cout << "Velocity limit: " << limits[0].velocity_limit << std::endl;
      std::cout << "Acceleration limit: " << limits[0].acceleration_limit << std::endl;
      std::cout << "Jerk limit: " << limits[0].jerk_limit << std::endl;
      std::cout << "===" << std::endl;
    }

    // Save the smoothed trajectory
    trajectory_msgs::JointTrajectoryPoint new_point;

    std_msgs::Float64MultiArray new_positions;
    new_positions.data.resize(kNumDof);

    std_msgs::Float64MultiArray new_velocities;
    new_velocities.data.resize(kNumDof);

    std_msgs::Float64MultiArray new_accelerations;
    new_accelerations.data.resize(kNumDof);

    for (std::size_t smoothed_pt = 0; smoothed_pt<output_trajectories.at(0).elapsed_times.size(); ++smoothed_pt)
    {
      for (std::size_t joint = 0; joint<kNumDof; ++joint)
      {
        new_positions.data[joint] = output_trajectories.at(joint).positions(smoothed_pt);
        new_velocities.data[joint] = output_trajectories.at(joint).velocities(smoothed_pt);
        new_accelerations.data[joint] = output_trajectories.at(joint).accelerations(smoothed_pt);
      }

      new_point.positions = new_positions.data;
      new_point.velocities = new_velocities.data;
      new_point.accelerations = new_accelerations.data;
      new_point.time_from_start = ros::Duration(waypoint_start_time + output_trajectories.at(0).elapsed_times(smoothed_pt));
      smoothed_goal.trajectory.points.push_back(new_point);
    }
    waypoint_start_time = new_point.time_from_start.toSec() + kTimestep;

    // Save to file, for debugging
    traj_gen.saveTrajectoriesToFile(output_trajectories, "/home/andyz/Documents/" + std::to_string(point) + "_");
  }

  ////////////////
  // Send the goal
  ////////////////
  controller_action_client_->sendGoal(
      smoothed_goal, boost::bind(&FollowJointTrajectoryControllerHandle::controllerDoneCallback, this, _1, _2),
      boost::bind(&FollowJointTrajectoryControllerHandle::controllerActiveCallback, this),
      boost::bind(&FollowJointTrajectoryControllerHandle::controllerFeedbackCallback, this, _1));
  done_ = false;
  last_exec_ = moveit_controller_manager::ExecutionStatus::RUNNING;
  return true;
}

void FollowJointTrajectoryControllerHandle::configure(XmlRpc::XmlRpcValue& config)
{
  if (config.hasMember("path_tolerance"))
    configure(config["path_tolerance"], "path_tolerance", goal_template_.path_tolerance);
  if (config.hasMember("goal_tolerance"))
    configure(config["goal_tolerance"], "goal_tolerance", goal_template_.goal_tolerance);
  if (config.hasMember("goal_time_tolerance"))
    goal_template_.goal_time_tolerance = ros::Duration(parseDouble(config["goal_time_tolerance"]));
}

namespace
{
enum ToleranceVariables
{
  POSITION,
  VELOCITY,
  ACCELERATION
};
template <ToleranceVariables>
double& variable(control_msgs::JointTolerance& msg);

template <>
inline double& variable<POSITION>(control_msgs::JointTolerance& msg)
{
  return msg.position;
}
template <>
inline double& variable<VELOCITY>(control_msgs::JointTolerance& msg)
{
  return msg.velocity;
}
template <>
inline double& variable<ACCELERATION>(control_msgs::JointTolerance& msg)
{
  return msg.acceleration;
}

static std::map<ToleranceVariables, std::string> VAR_NAME = { { POSITION, "position" },
                                                              { VELOCITY, "velocity" },
                                                              { ACCELERATION, "acceleration" } };
static std::map<ToleranceVariables, decltype(&variable<POSITION>)> VAR_ACCESS = { { POSITION, &variable<POSITION> },
                                                                                  { VELOCITY, &variable<VELOCITY> },
                                                                                  { ACCELERATION,
                                                                                    &variable<ACCELERATION> } };

const char* errorCodeToMessage(int error_code)
{
  switch (error_code)
  {
    case control_msgs::FollowJointTrajectoryResult::SUCCESSFUL:
      return "SUCCESSFUL";
    case control_msgs::FollowJointTrajectoryResult::INVALID_GOAL:
      return "INVALID_GOAL";
    case control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS:
      return "INVALID_JOINTS";
    case control_msgs::FollowJointTrajectoryResult::OLD_HEADER_TIMESTAMP:
      return "OLD_HEADER_TIMESTAMP";
    case control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED:
      return "PATH_TOLERANCE_VIOLATED";
    case control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED:
      return "GOAL_TOLERANCE_VIOLATED";
    default:
      return "unknown error";
  }
}
}  // namespace

void FollowJointTrajectoryControllerHandle::configure(XmlRpc::XmlRpcValue& config, const std::string& config_name,
                                                      std::vector<control_msgs::JointTolerance>& tolerances)
{
  if (isStruct(config))  // config should be either a struct of position, velocity, acceleration
  {
    for (ToleranceVariables var : { POSITION, VELOCITY, ACCELERATION })
    {
      if (!config.hasMember(VAR_NAME[var]))
        continue;
      XmlRpc::XmlRpcValue values = config[VAR_NAME[var]];
      if (isArray(values, joints_.size()))
      {
        size_t i = 0;
        for (const auto& joint_name : joints_)
          VAR_ACCESS[var](getTolerance(tolerances, joint_name)) = parseDouble(values[i++]);
      }
      else
      {  // use common value for all joints
        double value = parseDouble(values);
        for (const auto& joint_name : joints_)
          VAR_ACCESS[var](getTolerance(tolerances, joint_name)) = value;
      }
    }
  }
  else if (isArray(config))  // or an array of JointTolerance msgs
  {
    for (int i = 0; i < config.size(); ++i)  // NOLINT(modernize-loop-convert)
    {
      control_msgs::JointTolerance& tol = getTolerance(tolerances, config[i]["name"]);
      for (ToleranceVariables var : { POSITION, VELOCITY, ACCELERATION })
      {
        if (!config[i].hasMember(VAR_NAME[var]))
          continue;
        VAR_ACCESS[var](tol) = parseDouble(config[i][VAR_NAME[var]]);
      }
    }
  }
  else
    ROS_WARN_STREAM_NAMED(LOGNAME, "Invalid " << config_name);
}

control_msgs::JointTolerance& FollowJointTrajectoryControllerHandle::getTolerance(
    std::vector<control_msgs::JointTolerance>& tolerances, const std::string& name)
{
  auto it =
      std::lower_bound(tolerances.begin(), tolerances.end(), name,
                       [](const control_msgs::JointTolerance& lhs, const std::string& rhs) { return lhs.name < rhs; });
  if (it == tolerances.cend() || it->name != name)
  {  // insert new entry if not yet available
    it = tolerances.insert(it, control_msgs::JointTolerance());
    it->name = name;
  }
  return *it;
}

void FollowJointTrajectoryControllerHandle::controllerDoneCallback(
    const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  // Output custom error message for FollowJointTrajectoryResult if necessary
  if (!result)
    ROS_WARN_STREAM_NAMED(LOGNAME, "Controller " << name_ << " done, no result returned");
  else if (result->error_code == control_msgs::FollowJointTrajectoryResult::SUCCESSFUL)
    ROS_INFO_STREAM_NAMED(LOGNAME, "Controller " << name_ << " successfully finished");
  else
    ROS_WARN_STREAM_NAMED(LOGNAME, "Controller " << name_ << " failed with error "
                                                 << errorCodeToMessage(result->error_code) << ": "
                                                 << result->error_string);
  finishControllerExecution(state);
}

void FollowJointTrajectoryControllerHandle::controllerActiveCallback()
{
  ROS_DEBUG_STREAM_NAMED(LOGNAME, name_ << " started execution");
}

void FollowJointTrajectoryControllerHandle::controllerFeedbackCallback(
    const control_msgs::FollowJointTrajectoryFeedbackConstPtr& /* feedback */)
{
}

}  // end namespace moveit_simple_controller_manager
