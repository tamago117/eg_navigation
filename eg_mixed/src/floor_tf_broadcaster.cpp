/**
* @file floor_tf_broadcaster.cpp
* @brief broadcast tf map->floor
* @author Shunya Hara
* @date 2021.11.11
* @details map->floorを配信　floorはzだけbase_linkのtf
*/

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include <eg_mixed/tf_position.h>

int main(int argc, char **argv){
    
    ros::init(argc, argv, "floor_tf_broadcaster");
    ros::NodeHandle n;
    

    //param setting
    ros::NodeHandle pn("~");
    double publish_frequency;
    pn.param<double>("publish_frequency", publish_frequency, 10.0);
    std::string global_frame;
    pn.param<std::string>("global_frame", global_frame, "map");
    std::string robot_base_frame;
    pn.param<std::string>("robot_base_frame", robot_base_frame, "base_link");
    std::string floor_frame;
    pn.param<std::string>("floor_frame", floor_frame, "floor");

    //制御周期10Hz
    ros::Rate loop_rate(publish_frequency);

    tf_position tf_lis(global_frame,robot_base_frame, publish_frequency);
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.frame_id = global_frame;
    transformStamped.child_frame_id =  floor_frame;
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;

    while (n.ok())  {
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.transform.translation.z = tf_lis.getPose().position.z;
        br.sendTransform(transformStamped);

        ros::spinOnce();//subsucriberの割り込み関数はこの段階で実装される
        loop_rate.sleep();
    }
    
    return 0;
}