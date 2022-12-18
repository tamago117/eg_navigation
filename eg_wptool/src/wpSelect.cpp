/**
* @file wpSelect.cpp
* @brief select target way point
* @author Michikuni Eguchi
* @date 2021.9.21
* @details 位置情報からpublishするtarget way pointとなるpose を選ぶ
*          (等間隔で設定されてない5〜10mの少し離れた間隔のwp pointをもとにする)
*/
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <std_msgs/UInt8MultiArray.h>
#include <string>
#include "eg_navigation/tf_position.h"
#include "eg_navigation/robot_status.h"

//poseStamp間の距離
double poseStampDistance(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2)
{
    double diffX = pose1.pose.position.x - pose2.pose.position.x;
    double diffY = pose1.pose.position.y - pose2.pose.position.y;
    //double diffZ = pose1.pose.position.z - pose2.pose.position.z;

    //return sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ);
    return sqrt(diffX * diffX + diffY * diffY);
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

nav_msgs::Path path;
void path_callback(const nav_msgs::Path path_message)
{
    path = path_message;
}

std_msgs::UInt8MultiArray mode_array;
void wpMode_callback(const std_msgs::UInt8MultiArray& modeArray_message)
{
    mode_array = modeArray_message;
}

bool isSuccessPlanning = true;
int failedPlanCount = 0;
void successPlan_callback(const std_msgs::Bool successPlan_message)
{
    isSuccessPlanning = successPlan_message.data;
}

std_msgs::String mode_in;
void mode_callback(const std_msgs::String& mode_message)
{
    mode_in = mode_message;
}

std_msgs::Int32 targetWp;
void wayPoint_set_callback(const std_msgs::Int32 wpset_message)
{
    targetWp = wpset_message;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpControll");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string map_id, base_link_id;
    pnh.param<std::string>("map_frame_id", map_id, "map");
    pnh.param<std::string>("base_link_frame_id", base_link_id, "base_link");
    double target_deviation, fin_tar_deviation;
    pnh.param<double>("target_deviation", target_deviation, 0.5);
    pnh.param<double>("final_target_deviation", fin_tar_deviation, 0.1);
    double failed_tar_deviation_rate;
    pnh.param<double>("failed_target_deviation_rate", failed_tar_deviation_rate, 2);
    double rate;
    pnh.param<double>("loop_rate", rate, 100);
    double maxVelocity;
    pnh.param<double>("maxVelocity", maxVelocity, 1.0);

    tf_position nowPosition(map_id, base_link_id, rate);

    ros::Subscriber path_sub = nh.subscribe("path", 50, path_callback);
    ros::Subscriber wpMode_sub = nh.subscribe("wayPoint/mode", 10, wpMode_callback);
    ros::Subscriber successPlan_sub = nh.subscribe("successPlan", 10, successPlan_callback);
    ros::Subscriber mode_sub = nh.subscribe("mode", 10 , mode_callback);
    ros::Subscriber wpSet_sub = nh.subscribe("wayPoint/set", 10, wayPoint_set_callback);
    ros::Publisher tarWp_pub = nh.advertise<std_msgs::Int32>("targetWp", 10);
    ros::Publisher tarPos_pub = nh.advertise<geometry_msgs::PoseStamped>("targetWpPose", 10);
    ros::Publisher mode_pub = nh.advertise<std_msgs::String>("mode_select/mode", 10);

    ros::Rate loop_rate(rate);

    bool trace_wp_mode = true;
    std_msgs::String mode_out;
    mode_out.data = robot_status_str(robot_status::angleAdjust);

    targetWp.data = 0;

    geometry_msgs::PoseStamped tarPos;
    while(ros::ok())
    {
        if(path.poses.size()>0){

            //if planning fail, increase the target deviation
            double target_deviation_;
            if(isSuccessPlanning){
                target_deviation_ = target_deviation;
            }else{
                target_deviation_ = failed_tar_deviation_rate * target_deviation;
            }

            if(trace_wp_mode){
                //target_deviationになるよう target way pointの更新
                while(!(poseStampDistance(path.poses[targetWp.data], nowPosition.getPoseStamped()) >= target_deviation_))
                {
                    //end point
                    if(targetWp.data >= (path.poses.size()-1)){
                        break;
                    }
                    //stop point
                    if(mode_array.data.at(targetWp.data) == (uint8_t)robot_status::stop){
                        break;
                    }
                    targetWp.data++;
                }
            }

            //angle adjust at specific wp
            //distance
            if(not(mode_in.data==robot_status_str(robot_status::angleAdjust) and mode_in.data==robot_status_str(robot_status::stop))){
                if(poseStampDistance(path.poses[targetWp.data], nowPosition.getPoseStamped()) <= fin_tar_deviation){
                    mode_pub.publish(mode_out);

                    if(!(targetWp.data >= (path.poses.size()-1))){
                        targetWp.data++;
                    }

                }
            }


            tarPos.header.frame_id = path.header.frame_id;
            tarPos.header.stamp = ros::Time::now();
            tarPos.pose = path.poses[targetWp.data].pose;

            tarPos_pub.publish(tarPos);
            tarWp_pub.publish(targetWp);

        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}