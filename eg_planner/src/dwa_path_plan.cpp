/**
* @file dwa_path_plan.cpp
* @brief path planning by dynamic window approach
* @author Michikuni Eguchi
* @date 2021.8.19
* @details local path planning by dynamic window approach using local cost map
*/

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include "eg_planner/dwa.h"
#include "eg_planner/tf_position.h"

bool is_path_topic = false;
bool is_costmap_topic = false;

int targetWp = 0;
void targetWp_callback(const std_msgs::Int32& targetWp_num)
{
    targetWp = targetWp_num.data;
}

nav_msgs::Path path;
void path_callback(const nav_msgs::Path& path_message)
{
    path = path_message;

    is_path_topic = true;
}

nav_msgs::OccupancyGrid costmap;
void cost_callback(const nav_msgs::OccupancyGrid& costmap_message)
{
    costmap = costmap_message;

    is_costmap_topic = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dwa_path_plan");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    double maxSpeed, maxYaw_rate;
    pnh.param<double>("maxSpeed", maxSpeed, 2.0);
    pnh.param<double>("maxYaw_rate", maxYaw_rate, 40);
    maxYaw_rate *= M_PI/180;
    std::string map_id, base_link_id;
    pnh.param<std::string>("map_frame_id", map_id, "map");
    pnh.param<std::string>("base_link_frame_id", base_link_id, "base_link");
    double rate;
    pnh.param<double>("loop_rate", rate, 100);

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("dwa_path_plan/trajectory", 10);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("dwa_path_plan/cmd_vel", 10);
    ros::Subscriber targetWp_sub = nh.subscribe("dwa_path_plan/goal", 50, targetWp_callback);
    ros::Subscriber path_sub = nh.subscribe("path", 50, path_callback);
    ros::Subscriber cost_sub = nh.subscribe("dwa_path_plan/costmap", 10, cost_callback);

    ros::Rate loop_rate(rate);

    geometry_msgs::Twist cmd_vel;
    nav_msgs::Path trajectory;
    tf_position nowPosition(map_id, base_link_id, rate);

    double preX = 0;
    double preY = 0;
    double preAngle = 0;

    ctr::DWA dwa;
    ctr::DWA::motionState nowState;

    while(ros::ok())
    {
        
        //updata now position and velocity information
        nowState.x = nowPosition.getPose().position.x;
        nowState.y = nowPosition.getPose().position.y;
        nowState.yawAngle = nowPosition.getYaw();
        nowState.vel = (sqrt(pow(nowState.x - preX, 2) + pow(nowState.y - preY, 2)))/(1/rate);
        nowState.yawVel = (nowState.yawAngle - preAngle)/(1/rate);

        preX = nowState.x;
        preY = nowState.y;
        preAngle = nowState.yawAngle;


        if(is_path_topic && is_costmap_topic){
            //local path planning by dynamic window approach
            dwa.dwa_controll(nowState, path.poses[targetWp].pose, costmap);
            dwa.get_result(cmd_vel, trajectory);

            if(cmd_vel.linear.x > 0 && cmd_vel.linear.x < 0.05){
                cmd_vel.linear.x = 0.05;
            }
            if(cmd_vel.angular.z > 0 && cmd_vel.angular.z < 0.05){
                cmd_vel.angular.z = 0.05;
            }
            if(cmd_vel.angular.z > 0 && cmd_vel.angular.z < -0.05){
                cmd_vel.angular.z = -0.05;
            }

            //publish cmd_vel and trajectory
            cmd_pub.publish(cmd_vel);
            path_pub.publish(trajectory);

        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}