/**
* @file pureWpTrace.cpp
* @brief tracking target way point
* @author Michikuni Eguchi
* @date 2021.7.29
* @details 受け取ったtarget way point に追従するようにpure pursuit
*/
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include "eg_track/PurePursuit.h"
#include "eg_track/tf_position.h"
#include <vector>
#include <string>
#include <iostream>

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

double quat2yaw(geometry_msgs::Quaternion orientation)
{
    double roll, pitch, yaw;
    tf::Quaternion quat;
    quaternionMsgToTF(orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference

    return yaw;
}

double arrangeAngle(double angle)
{
    while(angle>M_PI)
    {
        angle -= 2*M_PI;
    }
    while(angle<-M_PI)
    {
        angle += 2*M_PI;
    }

    return angle;
}

//poseStamp間の距離
double poseStampDistance(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2)
{
    double diffX = pose1.pose.position.x - pose2.pose.position.x;
    double diffY = pose1.pose.position.y - pose2.pose.position.y;
    double diffZ = pose1.pose.position.z - pose2.pose.position.z;

    return sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ);
}

std::vector<double> calc_curvature(const nav_msgs::Path& path)
{
    std::vector<double> curvatures;

    curvatures.push_back(0.0);
    for(int i=0; i<path.poses.size()-1; i++){
        double diff_angle = arrangeAngle(quat2yaw(path.poses[i+1].pose.orientation) - quat2yaw(path.poses[i].pose.orientation));
        double distance = poseStampDistance(path.poses[i+1], path.poses[i]);
        double curvature = abs(diff_angle / distance);

        curvatures.push_back(curvature);
    }

    return curvatures;
}

template <class T> std::vector<T> normalize(std::vector<T>& num)
{
    if(num.empty()){
        return num;
    }

    T xmin = *std::min_element(std::begin(num), std::end(num));
    T xmax = *std::max_element(std::begin(num), std::end(num));

    //0 division
    if(xmin == xmax){
        if(xmin == 0){
            return num;
        }else{
            for(auto& n : num){
                n = n/n; //1
            }
            return num;
        }
    }

    for(auto& n : num){
        n = (n - xmin) / (xmax - xmin);
    }
    return num;
}

template <class T> T clip(const T& n, double lower, double upper)
{
    T numbers = n;
    for(auto& number : numbers)
    {
        number = std::max(lower, std::min(number, upper));
    }
  return numbers;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pureWpTrace");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string base_link_id, map_id;
    pnh.param<std::string>("base_link_frame_id", base_link_id, "base_link");
    pnh.param<std::string>("map_frame_id", map_id, "map");
    double rate;
    pnh.param<double>("loop_rate", rate, 30);
    double max_angular_vel;
    pnh.param<double>("max_angular_vel", max_angular_vel, 1);
    double minSpeed, maxSpeed;
    pnh.param<double>("minSpeed", minSpeed, 0.1);
    pnh.param<double>("maxSpeed", maxSpeed, 0.5);
    double maxCurvature;
    pnh.param<double>("maxCurvature", maxCurvature, 3);

    ros::Subscriber targetWp_sub = nh.subscribe("targetWp", 50, targetWp_callback);
    ros::Subscriber path_sub = nh.subscribe("path", 50, path_callback);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("roomba/cmd_vel", 10);

    ctr::PurePursuit pure_pursuit(max_angular_vel);
    tf_position nowPosition(map_id, base_link_id, rate);

    ros::Rate loop_rate(rate);

    std::vector<double> curvatures;
    geometry_msgs::Twist cmd_vel;
    while(ros::ok())
    {
        if(path.poses.size()>0){


            curvatures = calc_curvature(path);
            curvatures = clip(curvatures, 0, maxCurvature);
            // ->0 ~ 1
            curvatures = normalize(curvatures);

            //change velocity according to curvature (asteroid)
            cmd_vel.linear.x = maxSpeed * pow(sin(acos(std::cbrt(curvatures[targetWp]))), 3);
            //change velocity according to curvature (linear)
            //cmd_vel.linear.x = maxSpeed - (maxSpeed - minSpeed)*curvatures[targetWp];

            cmd_vel.angular.z = pure_pursuit.getYawVel(nowPosition.getPoseStamped(), path.poses[targetWp] , cmd_vel.linear.x);
            cmd_pub.publish(cmd_vel);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}