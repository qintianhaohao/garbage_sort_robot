#include <ros/ros.h>
#include <iostream>
#include <string.h>
#include <string>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdbool.h>

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <wheeltec_arm_pick/pick_and_put.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "arm_pick_and_put.h"


std::string arm_state="none";

void pick_place_callback(const std_msgs::String &pick_place)
{
    arm_state = pick_place.data;
}


int main(int argc, char **argv)
{ 
    //std_msgs::Float32 msg;
    wheeltec_arm_pick::pick_and_put msg;
    ros::init(argc, argv, "arm_pick_and_place");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("arm");   
    moveit::planning_interface::MoveGroupInterface hand("hand"); 

    arm.setGoalJointTolerance(0.01);
    //arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.5);

    arm.setNamedTarget("arm_uplift"); arm.move(); sleep(1); //机械臂回到收起的状态
    hand.setNamedTarget("hand_close"); hand.move(); sleep(1); //机械爪关闭

    ros::Subscriber pick_place_sub=n.subscribe("pick_place",10,pick_place_callback); // subscribe wheel area
   
    while(ros::ok())
   {
         if (arm_state=="pick")  arm_pick();//机械臂抓取色块
    else if (arm_state=="place")   arm_put(); //机械臂放置色块
    ros::spinOnce();
   }
    ros::shutdown(); 
    return 0;
}

//一个完整的夹取动作
void arm_pick()
{
    moveit::planning_interface::MoveGroupInterface arm("arm");
    moveit::planning_interface::MoveGroupInterface hand("hand");
    arm.setGoalJointTolerance(0.01);
    //arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.6);
    hand.setNamedTarget("hand_open");  hand.move(); sleep(1);
    arm.setNamedTarget("arm_clamp");   arm.move();  sleep(1);
    hand.setNamedTarget("hand_close"); hand.move(); sleep(1);
    arm.setNamedTarget("arm_uplift");  arm.move();  

}

//一个完整的放置动作
void arm_put()
{
    moveit::planning_interface::MoveGroupInterface arm("arm");
    moveit::planning_interface::MoveGroupInterface hand("hand");
    arm.setGoalJointTolerance(0.01);
    //arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.6);
    arm.setNamedTarget("arm_clamp");   arm.move();  sleep(1);
    hand.setNamedTarget("hand_open");  hand.move(); sleep(1);
    arm.setNamedTarget("arm_uplift");  arm.move();  sleep(1);
    hand.setNamedTarget("hand_close"); hand.move(); 
}

