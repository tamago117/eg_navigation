/**
* @file wpRecord.cpp
* @brief record way point
* @author Michikuni Eguchi
* @date 2021.7.27
* @details robotの移動経路からrobotのnavigationコース用ポイント(way point)をcsv形式で記憶
           移動経路には/base_linkから/mapまでの座標変換の計算結果を用いる
*/

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose.h>
#include "eg_wptool/tf_position.h"

//poseStamp間の距離
double poseDistance(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2)
{
    double diffX = pose1.position.x - pose2.position.x;
    double diffY = pose1.position.y - pose2.position.y;
    double diffZ = pose1.position.z - pose2.position.z;

    return sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpRecord");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("path2csv", 10);

    std::string map_id, base_link_id;
    pnh.param<std::string>("map_frame_id",map_id,"map");
    pnh.param<std::string>("base_link_frame_id",base_link_id,"base_link");
    int rate;
    pnh.param<int>("loop_rate", rate, 50);
    double wp_pitch;
    pnh.param<double>("wayPoint_pitch", wp_pitch, 0.1);

    tf_position nowPosition(map_id, base_link_id, rate);

    ros::Rate loop_rate(rate);

    geometry_msgs::Pose nowPose, prePose;
    prePose = nowPosition.getPose();
    while (ros::ok())
    {
        
        //waypoint_pitchになる時の現在地をcsv出力
        if(poseDistance(nowPosition.getPose(), prePose) >= wp_pitch){
            prePose = nowPosition.getPose();

            //csvdata : [x, y, z, q_x, q_y, q_z, q_w]
            std_msgs::Float32MultiArray array;
            array.data.resize(9);
            nowPose = nowPosition.getPose();
            array.data[0] = nowPose.position.x;
            array.data[1] = nowPose.position.y;
            array.data[2] = nowPose.position.z;
            array.data[3] = nowPose.orientation.x;
            array.data[4] = nowPose.orientation.y;
            array.data[5] = nowPose.orientation.z;
            array.data[6] = nowPose.orientation.w;
            array.data[7] = 0;
            array.data[8] = 3.0;

            pub.publish(array);
        }
        

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
