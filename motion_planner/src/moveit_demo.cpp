/***********************************************************************
Copyright 2019 Wuhan PS-Micro Technology Co., Itd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "kinematics/robcprobot_arm6DOFik.h"

bool movetopoint(float seed[6], float posdest[6])
{

    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    arm.setGoalJointTolerance(0.001);

    arm.setMaxAccelerationScalingFactor(0.5);
    arm.setMaxVelocityScalingFactor(0.5);


    Joint joint = getPositionIK(posdest,seed);
    printf("j0:%f\n",joint.joint0);
    printf("j0:%f\n",joint.joint1);
    printf("j0:%f\n",joint.joint2);
    printf("j0:%f\n",joint.joint3);
    printf("j0:%f\n",joint.joint4);
    printf("j0:%f\n",joint.joint5);

    std::vector<double> joint_group_positions(6);
    joint_group_positions[0] = joint.joint0;
    joint_group_positions[1] = joint.joint1;
    joint_group_positions[2] = joint.joint2;
    joint_group_positions[3] = joint.joint3;
    joint_group_positions[4] = joint.joint4;
    joint_group_positions[5] = joint.joint5;

    seed[0] = joint.joint0;
    seed[1] = joint.joint1;
    seed[2] = joint.joint2;
    seed[3] = joint.joint3;
    seed[4] = joint.joint4;
    seed[5] = joint.joint5;

    arm.setJointValueTarget(joint_group_positions);
    arm.move();
    sleep(1);

return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_fk_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    arm.setGoalJointTolerance(0.001);

    arm.setMaxAccelerationScalingFactor(0.5);
    arm.setMaxVelocityScalingFactor(0.5);

    initialize();

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    /*float gg[6] = {1.42314,0.91926,1.01491,1.61268,2.65534,-0.125838};
    Pos position = getPositionFK(gg);
    printf("x:%f\n",position.pos_x);
    printf("x:%f\n",position.pos_y);
    printf("x:%f\n",position.pos_z);
    printf("x:%f\n",position.pos_row);
    printf("x:%f\n",position.pos_pitch);
    printf("x:%f\n",position.pos_yaw);*/

    float seed[6] = { 0 };
    float posdest[6] = { 250,250,0,0,0,-3.141492 };
    movetopoint(seed, posdest);
    posdest[0] =  200; 
    posdest[1] =  300; 
    movetopoint(seed, posdest);


    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    ros::shutdown(); 

    return 0;
}

