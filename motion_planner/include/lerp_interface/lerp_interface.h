/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik, LLC.
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

/* Author: Omid Heidari
   Desc: LERP planner which is a linear interpolation algorithm in joint space.
 */
#pragma once

#include <ros/ros.h>
#include <moveit/planning_interface/planning_interface.h>

namespace lerp_interface
{
MOVEIT_CLASS_FORWARD(LERPInterface);

typedef struct
{
 float pos_x;
 float pos_y;
 float pos_z;
 float pos_row;
 float pos_pitch;
 float pos_yaw;
}Pos;

typedef struct
{
 float joint0;
 float joint1;
 float joint2;
 float joint3;
 float joint4;
 float joint5;
}Joint;

class LERPInterface
{
public:
  LERPInterface(const ros::NodeHandle& nh = ros::NodeHandle("~"));

  bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const planning_interface::MotionPlanRequest& req, moveit_msgs::MotionPlanDetailedResponse& res);

  void initialize();
  Pos getPositionFK(float joint[6]);
  Joint getPositionIK(float pos[6], float seed[6]);

protected:
  ros::NodeHandle nh_;
  std::string name_;
  int num_steps_;
  int dof_;

private:
  void interpolate(const std::vector<std::string> joint_names, robot_state::RobotStatePtr& robot_state,
                   const robot_state::JointModelGroup* joint_model_group, const std::vector<double>& start_joint_vals,
                   const std::vector<double>& goal_joint_vals, trajectory_msgs::JointTrajectory& joint_trajectory);
  void Quintic(const std::vector<std::string> joint_names, robot_state::RobotStatePtr& robot_state,
                   const robot_state::JointModelGroup* joint_model_group, const std::vector<double>& start_joint_vals,
                   const std::vector<double>& goal_joint_vals, trajectory_msgs::JointTrajectory& joint_trajectory);
  

};
}
