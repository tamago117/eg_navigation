/**
* @file wpnitialPose.cpp
* @brief 2D Pose Estimate
* @author Shunya Hara
* @date 2021.12.10
*/

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<geometry_msgs/PoseWithCovariance.h>


class wpInitialPose
{
public:
    wpInitialPose();

private:
    void wpset_callback(const std_msgs::Int32 wpset_message);
    void path_callback(const nav_msgs::Path& path_message);

    ros::Publisher initial_pub;
    ros::Subscriber buttons_sub;
    ros::Subscriber path_sub;

    geometry_msgs::PoseWithCovarianceStamped pubpose;
    nav_msgs::Path path;
};

wpInitialPose::wpInitialPose()
{
    ros::NodeHandle nh;
    initial_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    buttons_sub = nh.subscribe("wayPoint/set", 50, &wpInitialPose::wpset_callback, this);
    path_sub = nh.subscribe("wayPoint/path", 1, &wpInitialPose::path_callback, this);
    //3D pose の共分散
    pubpose.pose.covariance[0]=0.25;
    pubpose.pose.covariance[7]=0.25;
    pubpose.pose.covariance[35]=0.06853891945200942;
}

void wpInitialPose::wpset_callback(const std_msgs::Int32 wpset_message)
{
    int wpNum = wpset_message.data;

    if(path.poses.size()>0){
        pubpose.header.stamp=ros::Time::now();
        pubpose.header=path.header;
        pubpose.pose.pose = path.poses.at(wpNum).pose;

        initial_pub.publish(pubpose);
    }
}

void wpInitialPose::path_callback(const nav_msgs::Path& path_message)
{
    path = path_message;
}

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "wpInitialPose");
    wpInitialPose wpinitial;

    ros::spin();
    
    return 0;
}