/**
* @file pure_posePursuit.cpp
* @brief pursuit target pose
* @author Michikuni Eguchi
* @date 2021.9.22
* @details 受け取ったtarget way point に追従するようにpure pursuit
*/
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <string>
#include <iostream>
#include "eg_track/PurePursuit.h"
#include "eg_track/tf_position.h"

geometry_msgs::Pose targetPose;
void pose_callback(const geometry_msgs::Pose& pose_message)
{
    targetPose = pose_message;
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
    
    T number = std::max(lower, std::min(n, upper));
    
    return number;
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
    double minVelocity, maxVelocity;
    pnh.param<double>("minVelocity", minVelocity, 0.1);
    pnh.param<double>("maxVelocity", maxVelocity, 0.5);
    double maxCurvature;
    pnh.param<double>("maxCurvature", maxCurvature, 3);

    ros::Subscriber pose_sub = nh.subscribe("pure_posePursuit/pose_in", 10, pose_callback);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("pure_posePursuit/cmd_vel", 10);

    ctr::PurePursuit pure_pursuit(max_angular_vel);
    tf_position nowPosition(map_id, base_link_id, rate);

    ros::Rate loop_rate(rate);

    geometry_msgs::Twist cmd_vel;
    while(ros::ok())
    {
        //calc curvature
        double dx = targetPose.position.x - nowPosition.getPose().position.x;
        double dy = targetPose.position.y - nowPosition.getPose().position.y;
        double distance = sqrt(pow(dx, 2) + pow(dy, 2));
        double diffTheta = arrangeAngle(atan2(dy, dx) - nowPosition.getYaw());
        double curvature = abs(diffTheta/distance);
        //->0~maxCurvature
        curvature = clip(curvature, 0, maxCurvature);
        //->0~1
        curvature = curvature/maxCurvature;

        //change velocity according to curvature (asteroid)
        cmd_vel.linear.x = (maxVelocity-minVelocity) * pow(sin(acos(std::cbrt(curvature))), 3) + minVelocity;

        //change velocity according to curvature (linear)
        //cmd_vel.linear.x = maxVelocity - (maxVelocity - minVelocity)*curvatures[targetWp];

        cmd_vel.angular.z = pure_pursuit.getYawVel(nowPosition.getPose(), targetPose, cmd_vel.linear.x);
        cmd_pub.publish(cmd_vel);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}