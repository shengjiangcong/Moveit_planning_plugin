/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik, Inc.
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
 *   * Neither the name of PickNik nor the names of its
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

/* Author: Omid Heidari */

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/MotionPlanRequest.h>

#include <ros/ros.h>

#include <limits>
#include <vector>
#include <Eigen/Geometry>
#include <unordered_map>

#include "lerp_interface/lerp_interface.h"
#include "lerp_interface/Quintic_Spline.h"

namespace lerp_interface
{
LERPInterface::LERPInterface(const ros::NodeHandle& nh) : nh_(nh), name_("LERPInterface")
{
}

bool LERPInterface::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                          const planning_interface::MotionPlanRequest& req,
                          moveit_msgs::MotionPlanDetailedResponse& res)
{
  // Load the planner-specific parameters
  nh_.getParam("num_steps", num_steps_);//配置文件中的步长40

  ros::WallTime start_time = ros::WallTime::now();
  robot_model::RobotModelConstPtr robot_model = planning_scene->getRobotModel();
  robot_state::RobotStatePtr start_state(new robot_state::RobotState(robot_model));
  *start_state = planning_scene->getCurrentState();
  const robot_state::JointModelGroup* joint_model_group = start_state->getJointModelGroup(req.group_name);
  std::vector<std::string> joint_names = joint_model_group->getVariableNames();
  dof_ = joint_names.size();
  std::vector<double> start_joint_values;
  start_state->copyJointGroupPositions(joint_model_group, start_joint_values);
  //ROS_INFO("start_joint_degrees:%f,%f,%f,%f,%f,%f",start_joint_values[0],start_joint_values[1],start_joint_values[2],start_joint_values[3],start_joint_values[4],start_joint_values[5]);//规划初始六轴角度

  // This planner only supports one goal constriant in the request
  std::vector<moveit_msgs::Constraints> goal_constraints = req.goal_constraints;
  std::vector<moveit_msgs::JointConstraint> goal_joint_constraint = goal_constraints[0].joint_constraints;

  std::vector<double> goal_joint_values;
  for (auto constraint : goal_joint_constraint)
  {
    goal_joint_values.push_back(constraint.position);
  }
//ROS_INFO("goal_joint_degrees:%f,%f,%f,%f,%f,%f",goal_joint_values[0],goal_joint_values[1],goal_joint_values[2],goal_joint_values[3],goal_joint_values[4],goal_joint_values[5]);//规划终点六轴角度
  // ==================== Interpolation
  trajectory_msgs::JointTrajectory joint_trajectory;
  //interpolate(joint_names, start_state, joint_model_group, start_joint_values, goal_joint_values, joint_trajectory);
  Quintic(joint_names, start_state, joint_model_group, start_joint_values, goal_joint_values, joint_trajectory);
  // ==================== feed the response
  res.trajectory.resize(1);
  res.trajectory[0].joint_trajectory.joint_names = joint_names;
  res.trajectory[0].joint_trajectory.header = req.start_state.joint_state.header;
  res.trajectory[0].joint_trajectory = joint_trajectory;

  res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  res.processing_time.push_back((ros::WallTime::now() - start_time).toSec());

  res.group_name = req.group_name;
  res.trajectory_start.joint_state.name = joint_names;
  res.trajectory_start.joint_state.position = start_joint_values;

  return true;
}

void LERPInterface::interpolate(const std::vector<std::string> joint_names, robot_state::RobotStatePtr& rob_state,
                                const robot_state::JointModelGroup* joint_model_group,
                                const std::vector<double>& start_joint_vals, const std::vector<double>& goal_joint_vals,
                                trajectory_msgs::JointTrajectory& joint_trajectory)
{
  joint_trajectory.points.resize(num_steps_ + 1);

  std::vector<double> dt_vector;
  for (int joint_index = 0; joint_index < dof_; ++joint_index)
  {
    double dt = (goal_joint_vals[joint_index] - start_joint_vals[joint_index]) / num_steps_;
    dt_vector.push_back(dt);
  }

  for (int step = 0; step <= num_steps_; ++step)
  {
    std::vector<double> joint_values;
    for (int k = 0; k < dof_; ++k)
    {
      double joint_value = start_joint_vals[k] + step * dt_vector[k];
      joint_values.push_back(joint_value);
    }
    rob_state->setJointGroupPositions(joint_model_group, joint_values);
    rob_state->update();

    joint_trajectory.joint_names = joint_names;
    joint_trajectory.points[step].positions = joint_values;
//ROS_INFO("trajectory[%d]:%f",step,joint_trajectory.points[step].positions[1]);//轴1的轨迹
  }
}
void LERPInterface::Quintic(const std::vector<std::string> joint_names, robot_state::RobotStatePtr& rob_state,
                                const robot_state::JointModelGroup* joint_model_group,
                                const std::vector<double>& start_joint_vals, const std::vector<double>& goal_joint_vals,
                                trajectory_msgs::JointTrajectory& joint_trajectory)
{
  joint_trajectory.points.resize(num_steps_);
        int time = 4;//单位s
	float trajpoints_j0[500][4] = { 0 };//最多支持500个点
	float trajpoints_j1[500][4] = { 0 };//最多支持500个点
	float trajpoints_j2[500][4] = { 0 };//最多支持500个点
	float trajpoints_j3[500][4] = { 0 };//最多支持500个点
	float trajpoints_j4[500][4] = { 0 };//最多支持500个点
	float trajpoints_j5[500][4] = { 0 };//最多支持500个点
	bool Traj_Quintic_manypoints;
	float degree_array[4] = { 0,0 ,0,0};//单位°
	float velocity_array[4] = { 0,0,0,0};//单位°/s
	float acceleration_array[4] = { 0,0 ,0,0};
	unsigned long long array_num = sizeof(degree_array) / sizeof(float);//轨迹点的个数

  for (int joint_index = 0; joint_index < dof_; ++joint_index)
  {
        degree_array[0] = start_joint_vals[joint_index];
	degree_array[1] = 0;
	degree_array[2] = 0;
	degree_array[3] = goal_joint_vals[joint_index];
	QuinticSpline traj;
        if(joint_index == 0)
	Traj_Quintic_manypoints = traj.getTraj_Quintic_manypoints(time, degree_array, velocity_array, acceleration_array, *trajpoints_j0, array_num);
	else if(joint_index == 1)
	Traj_Quintic_manypoints = traj.getTraj_Quintic_manypoints(time, degree_array, velocity_array, acceleration_array, *trajpoints_j1, array_num);
	else if(joint_index == 2)
	Traj_Quintic_manypoints = traj.getTraj_Quintic_manypoints(time, degree_array, velocity_array, acceleration_array, *trajpoints_j2, array_num);
	else if(joint_index == 3)
	Traj_Quintic_manypoints = traj.getTraj_Quintic_manypoints(time, degree_array, velocity_array, acceleration_array, *trajpoints_j3, array_num);
	else if(joint_index == 4)
	Traj_Quintic_manypoints = traj.getTraj_Quintic_manypoints(time, degree_array, velocity_array, acceleration_array, *trajpoints_j4, array_num);
	else if(joint_index == 5)
	Traj_Quintic_manypoints = traj.getTraj_Quintic_manypoints(time, degree_array, velocity_array, acceleration_array, *trajpoints_j5, array_num);
	/*printf("joint%d\n",joint_index);
		for (int t = 0; t < time * 10; t++)
			{
				printf("degree:%f\n",trajpoints[t][1]);
			}*/
  }

  for (int step = 0; step < num_steps_; ++step)
  {
    std::vector<double> joint_values;
    for (int k = 0; k < dof_; ++k)
    {
      double joint_value = 0;
      if (k == 0)
      joint_value = trajpoints_j0[step][1];
      else if (k == 1)
      joint_value = trajpoints_j1[step][1];      
      else if (k == 2)
      joint_value = trajpoints_j2[step][1]; 
      else if (k == 3)
      joint_value = trajpoints_j3[step][1]; 
      else if (k == 4)
      joint_value = trajpoints_j4[step][1]; 
      else if (k == 5)
      joint_value = trajpoints_j5[step][1]; 
      joint_values.push_back(joint_value);
    }
    rob_state->setJointGroupPositions(joint_model_group, joint_values);
    rob_state->update();

    joint_trajectory.joint_names = joint_names;
    joint_trajectory.points[step].positions = joint_values;
//ROS_INFO("trajectory[%d]:%f",step,joint_trajectory.points[step].positions[1]);//轴1的轨迹
  }
}
}  // namespace lerp_interface
