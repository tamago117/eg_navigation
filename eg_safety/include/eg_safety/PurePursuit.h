#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <iostream>
#include <math.h>

namespace ctr{

class PurePursuit
{
    enum class car_model
    {
        steer,
        wheel_2
    };

private:
    car_model model;
    double max_angular_vel;
    double wheel_tred;
    double quat2yaw(const geometry_msgs::Quaternion& geometry_quat);
    double double_constrain(double val,double down_limit,double up_limit);
public:
    PurePursuit(double max_angular_vel_);
    PurePursuit(double wheel_tred_, double max_angular_vel_);
    double getYawVel(geometry_msgs::PoseStamped nowPos, const geometry_msgs::PoseStamped& tarPos, double forwardV);
    double getYawVel(const geometry_msgs::Pose& nowPos, const  geometry_msgs::Pose& tarPos, double forwardV);
};

PurePursuit::PurePursuit(double max_angular_vel_) : max_angular_vel(max_angular_vel_)
{
    model = car_model::wheel_2;
}

PurePursuit::PurePursuit(double wheel_tred_, double max_angular_vel_):wheel_tred(wheel_tred_) ,max_angular_vel(max_angular_vel_)
{
    model = car_model::steer;
}

double PurePursuit::getYawVel(geometry_msgs::PoseStamped nowPos, const geometry_msgs::PoseStamped& tarPos, double forwardV)
{
    double nowX = nowPos.pose.position.x;//m
    double nowY = nowPos.pose.position.y;//m
    double nowYaw = quat2yaw(nowPos.pose.orientation);//rad
    double tarX = tarPos.pose.position.x;
    double tarY = tarPos.pose.position.y;
    
    double L = sqrt(pow(tarX - nowX, 2) + pow(tarY - nowY, 2));

    //角度が180度を超えないようにする
    double angle1 = atan2(tarY-nowY, tarX-nowX) - nowYaw + 2*M_PI;
    double angle2 = atan2(tarY-nowY, tarX-nowX) - nowYaw;
    double angle3 = atan2(tarY-nowY, tarX-nowX) - nowYaw - 2*M_PI;

    double alfa;
    if(abs(angle1)<abs(angle2)&&abs(angle1)<abs(angle3)){
        alfa=angle1;
    }
    else if(abs(angle2)<abs(angle1)&&abs(angle2)<abs(angle3)){
        alfa=angle2;
    }
    else if(abs(angle3)<abs(angle1)&&abs(angle3)<abs(angle2)){
        alfa=angle3;
    }

    alfa=double_constrain(alfa,-M_PI/2.0,M_PI/2.0);


    if(model == car_model::wheel_2){
        double angular_vel = 2.0*forwardV*sin(alfa)/L;
        angular_vel = double_constrain(angular_vel, -max_angular_vel, max_angular_vel);

        return angular_vel;

    }else if(model == car_model::steer){
        double angular_vel = atan2(2*wheel_tred*sin(alfa), L);
        angular_vel = double_constrain(angular_vel, -max_angular_vel, max_angular_vel);

        return angular_vel;
    }

}

double PurePursuit::getYawVel(const geometry_msgs::Pose& nowPos, const geometry_msgs::Pose& tarPos, double forwardV)
{
    double nowX = nowPos.position.x;//m
    double nowY = nowPos.position.y;//m
    double nowYaw = quat2yaw(nowPos.orientation);//rad
    double tarX = tarPos.position.x;
    double tarY = tarPos.position.y;
    double tarYaw = quat2yaw(tarPos.orientation);

    double L = sqrt(pow(tarX - nowX, 2) + pow(tarY - nowY, 2));

    //角度が180度を超えないようにする
    double angle1 = atan2(tarY-nowY, tarX-nowX) - nowYaw + 2*M_PI;
    double angle2 = atan2(tarY-nowY, tarX-nowX) - nowYaw;
    double angle3 = atan2(tarY-nowY, tarX-nowX) - nowYaw - 2*M_PI;

    double alfa;
    if(abs(angle1)<abs(angle2)&&abs(angle1)<abs(angle3)){
        alfa=angle1;
    }
    else if(abs(angle2)<abs(angle1)&&abs(angle2)<abs(angle3)){
        alfa=angle2;
    }
    else if(abs(angle3)<abs(angle1)&&abs(angle3)<abs(angle2)){
        alfa=angle3;
    }

    alfa=double_constrain(alfa,-M_PI/2.0,M_PI/2.0);


    if(model == car_model::wheel_2){
        double angular_vel = 2.0*forwardV*sin(alfa)/L;
        angular_vel = double_constrain(angular_vel, -max_angular_vel, max_angular_vel);
        return angular_vel;
    }else if(model == car_model::steer){
        double angular_vel = atan2(2*wheel_tred*sin(alfa), L);
        angular_vel = double_constrain(angular_vel, -max_angular_vel, max_angular_vel);
        return angular_vel;
    }

}


double PurePursuit::quat2yaw(const geometry_msgs::Quaternion& geometry_quat)
{
    double roll, pitch, yaw;
    tf::Quaternion quat;

    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
    //std::cout<<yaw<<std::endl;

    return yaw;
}

double PurePursuit::double_constrain(double val,double down_limit,double up_limit){
  if(val>up_limit){
    return up_limit;
  }
  if(val<down_limit){
    return down_limit;
  }
  return val;
}

}//end namespace ctr