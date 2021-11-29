/**
* @file tf_position.h
* @brief  read frame position from tf
* @author Michikuni Eguchi
* @date 2021.7.29
* @details Read position from transformations in tf
*/

#pragma once
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <iostream>

class tf_position
{
public:
    tf_position(std::string base_id, std::string child_id, double rate);
    geometry_msgs::PoseStamped getPoseStamped();
    geometry_msgs::Pose getPose();
    double getRoll();
    double getPitch();
    double getYaw();
    double norm();

private:
    ros::NodeHandle nh;
    ros::Timer timer;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    std::string base_id, child_id;
    geometry_msgs::TransformStamped tfStamp;
    geometry_msgs::PoseStamped poseStamp;
    void quat2rpy(double& roll, double& pitch, double& yaw);

};

tf_position::tf_position(std::string base_id_, std::string child_id_, double rate) : base_id(base_id_), child_id(child_id_), nh(), tfBuffer(), tfListener(tfBuffer)
{
    timer = nh.createTimer(ros::Duration(1.0/rate), [&](const ros::TimerEvent& e)
    {
        try
        {
            tfStamp = tfBuffer.lookupTransform(base_id, child_id, ros::Time(0), ros::Duration(0.1));
            poseStamp.header = tfStamp.header;
            poseStamp.pose.position.x = tfStamp.transform.translation.x;
            poseStamp.pose.position.y = tfStamp.transform.translation.y;
            poseStamp.pose.position.z = tfStamp.transform.translation.z;
            poseStamp.pose.orientation = tfStamp.transform.rotation;
        }
        catch(tf2::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }
    });

}

geometry_msgs::PoseStamped tf_position::getPoseStamped()
{
    return poseStamp;
}

geometry_msgs::Pose tf_position::getPose()
{
    return poseStamp.pose;
}

double tf_position::getRoll(){
    double roll,pitch,yaw;
    quat2rpy(roll, pitch, yaw);
    return roll;
}

double tf_position::getYaw(){
    double roll,pitch,yaw;
    quat2rpy(roll, pitch, yaw);
    return yaw;
}

double tf_position::getPitch(){
    double roll,pitch,yaw;
    quat2rpy(roll, pitch, yaw);
    return pitch;
}

void tf_position::quat2rpy(double& roll, double& pitch, double& yaw)
{
    tf::Quaternion quat;
    quaternionMsgToTF(poseStamp.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}

double tf_position::norm()
{
    double x = poseStamp.pose.position.x;
    double y = poseStamp.pose.position.y;
    double z = poseStamp.pose.position.z;
    return sqrt(x*x + y*y + z*z);
}