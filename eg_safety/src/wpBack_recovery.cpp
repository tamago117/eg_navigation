/**
* @file wpBack_recovery.cpp
* @brief recovery behaviour toward the last way point
* @author Michikuni Eguchi
* @date 2021.9.28
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>
#include <string>
#include <iostream>
#include "eg_navigation/robot_status.h"
#include "eg_track/PurePursuit.h"
#include "eg_navigation/tf_position.h"

std_msgs::String mode;
void mode_callback(const std_msgs::String& mode_message)
{
    mode = mode_message;
}

std_msgs::Int32 now_wp;
void now_wp_callback(const std_msgs::Int32& now_wp_){
    now_wp=now_wp_;
}

nav_msgs::Path path;
void path_callback(const nav_msgs::Path path_message)
{
    path = path_message;
}

double poseStampDistance(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2)
{
    double diffX = pose1.pose.position.x - pose2.pose.position.x;
    double diffY = pose1.pose.position.y - pose2.pose.position.y;
    double diffZ = pose1.pose.position.z - pose2.pose.position.z;

    return sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpBack_recovery");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string base_link_id, map_id;
    pnh.param<std::string>("base_link_frame_id", base_link_id, "base_link");
    pnh.param<std::string>("map_frame_id", map_id, "map");
    double rate;
    pnh.param<double>("loop_rate", rate, 30);
    double linear_vel, max_angular_vel;
    double duration;
    double recovery_leastDistance;
    double fin_recovery_deviation;
    pnh.param<double>("linear_vel", linear_vel, -0.3);
    pnh.param<double>("max_angular_vel", max_angular_vel, 0.5);
    pnh.param<double>("duration", duration, 0.5);
    pnh.param<double>("recovery_leastDistance", recovery_leastDistance, 2);
    pnh.param<double>("fin_recovery_deviation", fin_recovery_deviation, 0.3);

    ros::Subscriber mode_sub = nh.subscribe("safety_limit/mode", 10, mode_callback);
    ros::Subscriber now_wp_sub = nh.subscribe("waypoint/now", 50, now_wp_callback);
    ros::Subscriber path_sub = nh.subscribe("path", 50, path_callback);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("recovery/cmd_vel", 10);
    ros::Publisher mode_pub = nh.advertise<std_msgs::String>("recovery/mode", 10);

    ros::Rate loop_rate(10);

    ctr::PurePursuit pure_pursuit(max_angular_vel);
    tf_position nowPosition(map_id, base_link_id, rate);

    geometry_msgs::Pose iniPose = nowPosition.getPose();
    geometry_msgs::PoseStamped recovery_pose;
    recovery_pose.pose= iniPose;

    std_msgs::String mode_out;
    geometry_msgs::Twist cmd_vel;
    ros::Time preT = ros::Time::now();

    int recoveryWpNum = 0;
    while(ros::ok())
    {
        if(path.poses.size()>0){
            if(mode_out.data == robot_status_str(robot_status::recovery)){
                cmd_vel.linear.x = linear_vel;
                cmd_vel.angular.z = pure_pursuit.getYawVel(nowPosition.getPoseStamped(), recovery_pose, cmd_vel.linear.x);

                if((ros::Time::now() - preT) >= ros::Duration(duration)){
                    mode_out.data = robot_status_str(robot_status::run);
                    std::cout<<"recovery end! (timer)"<<std::endl;
                }else if(poseStampDistance(path.poses[recoveryWpNum], nowPosition.getPoseStamped()) <= fin_recovery_deviation){
                    mode_out.data = robot_status_str(robot_status::run);
                    std::cout<<"recovery end! (distance)"<<std::endl;
                }

            }else{
                //select recovery way point
                if(recoveryWpNum > 0){
                    recoveryWpNum = now_wp.data - 1;
                }
                //target_deviationになるよう target way pointの更新
                while(!(poseStampDistance(path.poses[recoveryWpNum], nowPosition.getPoseStamped()) >= recovery_leastDistance))
                {
                    //first point
                    if(recoveryWpNum <= 0){
                        recoveryWpNum = 0;
                        break;
                    }
                    recoveryWpNum--;
                }
                
                recovery_pose = path.poses[recoveryWpNum];

                mode_out = mode;

                preT = ros::Time::now();
            }

            cmd_pub.publish(cmd_vel);
            mode_pub.publish(mode_out);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}