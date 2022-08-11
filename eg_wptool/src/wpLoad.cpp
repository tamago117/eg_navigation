/**
* @file wp_load.cpp
* @brief csv to path node
* @author Michikuni Eguchi
* @date 2021.7.28
* @details wpをcsvから読み込んでpathとmarkerarrayで配信する
*          waypoint/nowにあわせてmarkerの色を変える
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/MarkerArray.h>
#include "eg_wptool/csv_input.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpLoad");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    //csv file nameの読み込み
    std::string filePath, map_id;
    pnh.getParam("filePath", filePath);
    pnh.param<std::string>("map_frame_id", map_id, "map");


    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("wayPoint/path", 10);
    ros::Publisher wayMode_pub = nh.advertise<std_msgs::UInt8MultiArray>("wayPoint/mode", 10);
    ros::Publisher path_width_pub = nh.advertise<std_msgs::Float32MultiArray>("wayPoint/pathWidth", 10);


    geometry_msgs::PoseStamped pose;
    

    ros::Rate loop_rate(5);
    while(ros::ok())
    {
        std_msgs::UInt8MultiArray mode_array;
        nav_msgs::Path path;
        std_msgs::Float32MultiArray path_width;

        //csv読み込み
        csv::csv_input csv(filePath);
        for(int i = 0; i < csv.lineNum(); ++i)
        {
            pose.header.frame_id = map_id;
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = csv.readCSV(i, 0);
            pose.pose.position.y = csv.readCSV(i, 1);
            pose.pose.position.z = csv.readCSV(i, 2);
            pose.pose.orientation.x = csv.readCSV(i, 3);
            pose.pose.orientation.y = csv.readCSV(i, 4);
            pose.pose.orientation.z = csv.readCSV(i, 5);
            pose.pose.orientation.w = csv.readCSV(i, 6);

            mode_array.data.push_back((uint8_t)csv.readCSV(i, 7));

            path_width.data.push_back(csv.readCSV(i, 8));

            path.poses.push_back(pose);
        }
        path.header.frame_id = map_id;
        path.header.stamp = ros::Time::now();

        path_pub.publish(path);
        wayMode_pub.publish(mode_array);
        path_width_pub.publish(path_width);
        ros::spinOnce();
        loop_rate.sleep();
    }
}