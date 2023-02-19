/**
* @file state_maneger.cpp
* @brief state change maneger
* @author Michikuni Eguchi
* @date 2022.2.25
* @details 状態遷移を管理（そのうち外部ライブラリに移行）
*/
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include "eg_navigation/tf_position.h"
#include "eg_navigation/robot_status.h"

std_msgs::UInt8MultiArray mode_array;
void wpMode_callback(const std_msgs::UInt8MultiArray& modeArray_message)
{
    mode_array = modeArray_message;
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
    ros::init(argc, argv, "state_maneger");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    //std::string map_id, base_link_id;
    //pnh.param<std::string>("map_frame_id", map_id, "map");
    //pnh.param<std::string>("base_link_frame_id", base_link_id, "base_link");
    double rate;
    pnh.param<double>("loop_rate", rate, 100);

    //tf_position nowPosition(map_id, base_link_id, rate);

    ros::Subscriber wpMode_sub = nh.subscribe("wayPoint/mode", 10, wpMode_callback);
    ros::Subscriber mode_sub = nh.subscribe("mode", 10 , mode_callback);
    ros::Subscriber wpSet_sub = nh.subscribe("wayPoint/targetWp", 10, wayPoint_set_callback);
    //ros::Publisher mode_pub = nh.advertise<std_msgs::String>("mode_select/mode", 10);
    ros::Publisher lanePlanStop_pub = nh.advertise<std_msgs::Bool>("lane_planner/planning_stop", 10);
    ros::Publisher astarPlanStop_pub = nh.advertise<std_msgs::Bool>("astar_planning_node/planning_stop", 10);
    ros::Publisher recoveryStop_pub = nh.advertise<std_msgs::Bool>("safety_limit/recovery_stop", 10);

    ros::Rate loop_rate(rate);

    targetWp.data = 0;
    std_msgs::Bool lanePlanStop, astarPlanStop, recoveryStop;
    lanePlanStop.data = false;
    astarPlanStop.data = false;
    recoveryStop.data = false;

    while(ros::ok())
    {
        if(mode_array.data.size()>0){
            //state "column"
            if(mode_array.data[targetWp.data] == (uint8_t)robot_status::column){
                lanePlanStop.data = true;
                astarPlanStop.data = true;
                recoveryStop.data = true;
            }else if(mode_array.data[targetWp.data] == (uint8_t)robot_status::stop){
                lanePlanStop.data = true;
                astarPlanStop.data = true;
                recoveryStop.data = true;
            }else if(mode_array.data[targetWp.data] == (uint8_t)robot_status::run){
                lanePlanStop.data = false;
                astarPlanStop.data = false;
                recoveryStop.data = false;
            }

            lanePlanStop_pub.publish(lanePlanStop);
            astarPlanStop_pub.publish(astarPlanStop);
            recoveryStop_pub.publish(recoveryStop);
        }
        

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}