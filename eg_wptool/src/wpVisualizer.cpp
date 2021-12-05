/**
* @file wpVisualizer.cpp
* @brief way point visualizer node for Rviz
* @author Michikuni Eguchi
* @date 2021.8.19
* @details pathを読んでwaypointの番号をmarkerarrayで配信する可視化用ノード This node visualize waypoint number reading path.
*          And change marker color in accordance with now waypoint number
*/
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/UInt8MultiArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/ColorRGBA.h>
#include <iostream>
#include <string>
#include "eg_wptool/robot_status.h"

const double marker_diameter = 0.1;
const double marker_height = 0.03;
const double text_size = 0.1;

std_msgs::ColorRGBA set_color(double r, double g, double b, double a)
{
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;

    return color;
}
std_msgs::ColorRGBA green = set_color(0.0, 1.0, 0.0, 1.0);
std_msgs::ColorRGBA red = set_color(1.0, 0.0, 0.0, 1.0);
std_msgs::ColorRGBA blue = set_color(0.0, 0.0, 1.0, 1.0);
std_msgs::ColorRGBA gray = set_color(0.3, 0.3, 0.3, 1.0);
std_msgs::ColorRGBA black = set_color(0.0, 0.0, 0.0, 1.0);


int targetWp = 0;
void targetWp_callback(const std_msgs::Int32& targetWp_num)
{
    targetWp = targetWp_num.data;
}

nav_msgs::Path path;
void path_callback(const nav_msgs::Path& path_message)
{
    path = path_message;
}

std_msgs::UInt8MultiArray mode_array;
void wpMode_callback(const std_msgs::UInt8MultiArray& modeArray_message)
{
    mode_array = modeArray_message;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpVisualizer");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("wayPoint/marker", 10);
    ros::Publisher markerText_pub = nh.advertise<visualization_msgs::MarkerArray>("wayPoint/markerText", 10);
    ros::Subscriber targetWp_sub = nh.subscribe("targetWp", 50, targetWp_callback);
    ros::Subscriber path_sub = nh.subscribe("path", 50, path_callback);
    ros::Subscriber wpMode_sub = nh.subscribe("wayPoint/mode", 10, wpMode_callback);

    double markerSize;
    pnh.param<double>("markerSize", markerSize, 1.0);
    
    ros::Rate loop_rate(10);

    const double marker_diameter = 0.1*markerSize;
    const double marker_height = 0.03*markerSize;
    const double text_size = 0.1*markerSize;
    while(ros::ok())
    {
        visualization_msgs::MarkerArray marker_array, markerText_array;
        //marker
        for(int i=0; i<path.poses.size();++i){
            visualization_msgs::Marker marker, markerText;
            marker.header.frame_id = markerText.header.frame_id =path.header.frame_id;
            marker.header.stamp = markerText.header.stamp = ros::Time::now();
            marker.ns = markerText.ns = "waypoint_marker";
            marker.id = markerText.id = i;
            //marker.id = 0;
            //markerText.id = 1;
            marker.lifetime = markerText.lifetime = ros::Duration();
            marker.type = visualization_msgs::Marker::CYLINDER;
            markerText.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action = markerText.action = visualization_msgs::Marker::ADD;
            marker.scale.x = marker_diameter;
            marker.scale.y = marker_diameter;
            marker.scale.z =marker_height;
            markerText.scale.x = text_size;
            markerText.scale.y = text_size;
            markerText.scale.z =text_size;
            marker.pose = markerText.pose = path.poses.at(i).pose;
            markerText.pose.position.z+=marker_height;
            markerText.text= std::to_string(i).c_str();
            //color select
            if(targetWp > i){
                marker.color = gray;
            }
            //target
            if(targetWp == i){
                marker.color = red;
            }
            if(targetWp < i){
                marker.color = green;
            }
            //stop marker
            if(mode_array.data[i] == (uint8_t)robot_status::stop){
                marker.color = blue;
            }
            
            markerText.color = black;

            marker_array.markers.push_back(marker);
            markerText_array.markers.push_back(markerText);
        }

        marker_pub.publish(marker_array);
        markerText_pub.publish(markerText_array);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
