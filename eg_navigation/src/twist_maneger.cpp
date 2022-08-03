#include <ros/ros.h>
#include <math.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include "eg_navigation/tf_position.h"
#include "eg_navigation/robot_status.h"


geometry_msgs::Twist cmd_vel;
void cmd_callback(const geometry_msgs::Twist& cmd_message)
{
    cmd_vel = cmd_message;
}

geometry_msgs::Twist recovery_cmd_vel;
void recovery_cmd_callback(const geometry_msgs::Twist& cmd_message)
{
    recovery_cmd_vel = cmd_message;
}

std_msgs::String mode;
void mode_callback(const std_msgs::String& mode_message)
{
    mode = mode_message;
}

std_msgs::String recovery_mode;
void recovery_mode_callback(const std_msgs::String& mode_message)
{
    recovery_mode = mode_message;
}

geometry_msgs::PoseStamped targetWpPose;
void targetWpPose_callback(const geometry_msgs::PoseStamped& poseStamp_message)
{
    targetWpPose = poseStamp_message;
}

geometry_msgs::Pose targetPose;
void targetPose_callback(const geometry_msgs::Pose& pose_message)
{
    targetPose = pose_message;
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "twist_maneger");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string map_id, base_link_id;
    pnh.param<std::string>("map_frame_id", map_id, "map");
    pnh.param<std::string>("base_link_frame_id", base_link_id, "base_link");
    double rate;
    pnh.param<double>("loop_rate", rate, 100);
    bool mode_angleAdjust;
    pnh.param<bool>("angleAdjust", mode_angleAdjust, true);
    double angleAdjustGain;
    pnh.param<double>("angleAdjustGain", angleAdjustGain, 1.5);

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("twist_maneger/cmd_vel", 10);
    ros::Publisher mode_pub = nh.advertise<std_msgs::String>("mode", 10);
    ros::Subscriber cmd_sub = nh.subscribe("cmd_vel", 10 , cmd_callback);
    ros::Subscriber recovery_cmd_sub = nh.subscribe("recovery/cmd_vel", 10, recovery_cmd_callback);
    ros::Subscriber mode_sub = nh.subscribe("mode_select/mode", 10 , mode_callback);
    ros::Subscriber recovery_mode_sub = nh.subscribe("recovery/mode", 10 , recovery_mode_callback);
    ros::Subscriber targetWpPose_sub = nh.subscribe("twist_maneger/targetWpPose_in", 50, targetWpPose_callback);
    ros::Subscriber targetPose_sub = nh.subscribe("twist_maneger/targetPose_in", 10 , targetPose_callback);

    ros::Rate loop_rate(rate);

    tf_position nowPosition(map_id, base_link_id, rate);
    mode.data = robot_status_str(robot_status::stop);

    bool run_init = true;
    bool angleAdjustfinish = true;
    bool recovery_init = false;

    std_msgs::Float32 vel, yawVel;
    while(ros::ok())
    {
        if(run_init){
            //run mode
            if(mode.data == robot_status_str(robot_status::run)){
                if(mode_angleAdjust){
                    double dx = targetPose.position.x - nowPosition.getPose().position.x;
                    double dy = targetPose.position.y - nowPosition.getPose().position.y;
                    double targetAngle = atan2(dy, dx);
                    double diffAngle = arrangeAngle(targetAngle - nowPosition.getYaw());

                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = diffAngle * angleAdjustGain;
                    if(abs(diffAngle) < 10*M_PI/180){
                        run_init = false;
                    }
                }else{//no angle adjust
                    run_init = false;
                }
                
            }
        }
        //stop mode
        if(mode.data == robot_status_str(robot_status::stop)){
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            run_init = true;
        }
        //angle adjust
        if(mode.data == robot_status_str(robot_status::angleAdjust)){
            if(mode_angleAdjust){ //angle adjust mode
                angleAdjustfinish = false;

                double diffAngle = arrangeAngle(quat2yaw(targetWpPose.pose.orientation) - nowPosition.getYaw());

                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = diffAngle * angleAdjustGain;
                if(abs(diffAngle) < 1*M_PI/180){
                    mode.data = robot_status_str(robot_status::stop);
                    angleAdjustfinish = true;
                }
            }else{ //no angle adjust
                mode.data = robot_status_str(robot_status::stop);
            }
        }
        //safety stop
        if(recovery_mode.data == robot_status_str(robot_status::safety_stop) && mode.data != robot_status_str(robot_status::stop)){
            mode.data = robot_status_str(robot_status::safety_stop);
        }
        //end safety stop
        if(recovery_mode.data == robot_status_str(robot_status::run) && mode.data == robot_status_str(robot_status::safety_stop)){
            mode.data = robot_status_str(robot_status::run);
            if(not(angleAdjustfinish)){
                mode.data = robot_status_str(robot_status::angleAdjust);
            }
        }
        //recovery mode
        if(recovery_mode.data == robot_status_str(robot_status::recovery) && mode.data != robot_status_str(robot_status::stop)){
            recovery_init = true;
            mode.data = robot_status_str(robot_status::recovery);
            cmd_vel = recovery_cmd_vel;
        }
        //end recovery
        if(recovery_init){
            if(!(recovery_mode.data == robot_status_str(robot_status::recovery))){
                recovery_init = false;

                run_init = true;
                mode.data = robot_status_str(robot_status::run);
            }
        }


        cmd_vel_pub.publish(cmd_vel);
        mode_pub.publish(mode);


        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}