/**
* @file costmap_merger.cpp
* @brief 2 costmap merge
* @author Shunya Hara
* @date 2021.11.12
* @details ２つのコストマップの高いほうの数値を選んで融合する
*/

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

ros::Publisher costmap_pub;

//callback functions
nav_msgs::OccupancyGrid sub_costmap;
nav_msgs::OccupancyGrid base_costmap;
void base_callback(const nav_msgs::OccupancyGrid& base_costmap_){
    base_costmap=base_costmap_;
    if(base_costmap.data.size()==sub_costmap.data.size()){
        for(int i=0; i<base_costmap.data.size(); i++){
            base_costmap.data[i]=std::max(base_costmap.data[i],sub_costmap.data[i]);
        }
    }
    costmap_pub.publish(base_costmap);
}

void sub_callback(const nav_msgs::OccupancyGrid& sub_costmap_){
    sub_costmap=sub_costmap_;
}


int main(int argc, char **argv){
    
    ros::init(argc, argv, "costmap_merger");
    ros::NodeHandle n;
    

    //param setting
    ros::NodeHandle pn("~");
    double width;
    pn.param<double>("width", width, 30.0);
    


    ros::NodeHandle lSubscriber("");

    //base subscliber
    ros::Subscriber base_sub = lSubscriber.subscribe("costmap_node/base_costmap", 50, base_callback);
    //sub subscliber
    ros::Subscriber sub_sub = lSubscriber.subscribe("costmap_node/sub_costmap", 50, sub_callback);

    //costmap publisher
    costmap_pub=n.advertise<nav_msgs::OccupancyGrid>("costmap_node/merged_costmap", 1);
    
    ros::spin();//subsucriberの割り込み関数はこの段階で実装される
    return 0;
}