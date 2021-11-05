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

    geometry_msgs::PoseStamped pose;

    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        nav_msgs::Path path;
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

            path.poses.push_back(pose);
        }
        path.header.frame_id = map_id;
        path.header.stamp = ros::Time::now();

        path_pub.publish(path);
        ros::spinOnce();
        loop_rate.sleep();
    }
}