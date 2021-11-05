/**
* @file wpControll.cpp
* @brief select target way point
* @author Michikuni Eguchi
* @date 2021.7.29
* @details 位置情報やインタフェースからpublishするtarget way point を選ぶ
*/
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <iostream>
#include <string>
#include "eg_wptool/tf_position.h"
#include "eg_wptool/robot_status.h"

//poseStamp間の距離
double poseStampDistance(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2)
{
    double diffX = pose1.pose.position.x - pose2.pose.position.x;
    double diffY = pose1.pose.position.y - pose2.pose.position.y;
    double diffZ = pose1.pose.position.z - pose2.pose.position.z;

    return sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ);
}

template<class T> T constrain(T num, double minVal, double maxVal)
{
    if(num > maxVal){
        num = maxVal;
    }
    if(num < minVal){
        num = minVal;
    }

    return num;
}

int targetWp = 0;
void targetWp_callback(const std_msgs::Int32& targetWp_num)
{
    targetWp = targetWp_num.data;
}

nav_msgs::Path path;
void path_callback(const nav_msgs::Path path_message)
{
    path = path_message;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpControll");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string map_id, base_link_id;
    pnh.param<std::string>("map_frame_id", map_id, "map");
    pnh.param<std::string>("base_link_frame_id", base_link_id, "base_link");
    double target_pitch, wp_pitch, tar_deviation;
    pnh.param<double>("target_pitch", target_pitch, 1.0);
    pnh.param<double>("waypoint_pitch", wp_pitch, 0.1);
    pnh.param<double>("target_deviation", tar_deviation, 0.05);
    double rate;
    pnh.param<double>("loop_rate", rate, 100);
    double maxSpeed;
    pnh.param<double>("max_speed", maxSpeed, 1.0);

    tf_position nowPosition(map_id, base_link_id, rate);

    ros::Subscriber path_sub = nh.subscribe("path", 50, path_callback);
    ros::Publisher tarWp_pub = nh.advertise<std_msgs::Int32>("targetWp", 10);
    ros::Publisher nowWp_pub = nh.advertise<std_msgs::Int32>("nowWp", 10);
    ros::Publisher mode_pub = nh.advertise<std_msgs::String>("mode_select/mode", 10);

    ros::Rate loop_rate(rate);

    bool trace_wp_mode = true;
    std_msgs::Int32 targetWp;
    std_msgs::Int32 nowWp;

    std_msgs::String mode;
    mode.data = robot_status_str(robot_status::angleAdjust);
    bool isReach = false;
    while(ros::ok())
    {
        if(path.poses.size()>0){
            if(trace_wp_mode){
                //target_pitchになるよう target way pointの更新
                while(!(poseStampDistance(path.poses[targetWp.data], nowPosition.getPoseStamped()) >= target_pitch))
                {
                    //end point
                    if(targetWp.data >= (path.poses.size()-1)){
                        break;
                    }
                    targetWp.data++;
                }
            }
        }

        if(targetWp.data >= (path.poses.size()-1)){
            //distance
            if(!isReach){
                if(poseStampDistance(path.poses[targetWp.data], nowPosition.getPoseStamped()) <= tar_deviation){
                    //isReach = true;
                    mode_pub.publish(mode);
                }
            }
        }

        double dt = 0.2;
        nowWp.data = targetWp.data - (target_pitch + maxSpeed*dt)/wp_pitch;
        nowWp.data = constrain(nowWp.data, 0.0, path.poses.size());

        tarWp_pub.publish(targetWp);
        nowWp_pub.publish(nowWp);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}