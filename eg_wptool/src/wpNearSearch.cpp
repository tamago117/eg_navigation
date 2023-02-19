/**
* @file wpIdentify.cpp
* @brief way point identify
* @author Michikuni Eguchi
* @date 2021.9.3
* @details 現在地に最も近いway pointを特定する
*          kd tree を用いることで高速に処理することができる
*/

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <string>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "eg_navigation/tf_position.h"

class wpNearSearch
{
    public:
        wpNearSearch();
        void update();

    private:
        ros::NodeHandle nh;
        ros::NodeHandle pnh;

        ros::Publisher nearWp_pub;
        ros::Subscriber path_sub;

        std::string map_id, base_link_id;
        double rate;

        nav_msgs::Path path;
        void path_callback(const nav_msgs::Path& path_message){
            path = path_message;
        }

        void kdTree(int& wayPointIdx, const geometry_msgs::Pose& nowPos);
};

wpNearSearch::wpNearSearch() : pnh("~")
{
    // parameter
    pnh.param<std::string>("map_frame_id", map_id, "map");
    pnh.param<std::string>("base_link_frame_id", base_link_id, "base_link");
    pnh.param<double>("loop_rate", rate, 20);

    // publisher
    nearWp_pub = nh.advertise<std_msgs::Int32>("wayPoint/currentWp", 10);
    // subscribe
    path_sub = nh.subscribe("wayPoint/path", 50, &wpNearSearch::path_callback, this);
}

void wpNearSearch::update()
{
    tf_position nowPosition(map_id, base_link_id, rate);
    ros::Rate loop_rate(rate);

    std_msgs::Int32 currentWpIdx;
    while(ros::ok())
    {
        if(path.poses.size()>0){
            kdTree(currentWpIdx.data, nowPosition.getPose());

            nearWp_pub.publish(currentWpIdx);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void wpNearSearch::kdTree(int& wayPointIdx, const geometry_msgs::Pose& nowPos){
    // How to use a KdTree to search
    // Ref: http://pointclouds.org/documentation/tutorials/kdtree_search.php#kdtree-search

    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    // Use pcl::PointXYZRGB to visualize segmentation.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.resize(path.poses.size());

    for(int i=0; i<path.poses.size(); i++){
        cloud->points[i].x = path.poses[i].pose.position.x;
        cloud->points[i].y = path.poses[i].pose.position.y;
        cloud->points[i].z = path.poses[i].pose.position.z;
    }
    //set kdtree
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
    kdtree->setInputCloud(cloud);
    //set search point(now point)
    pcl::PointXYZ searchPoint;
    searchPoint.x = nowPos.position.x;
    searchPoint.y = nowPos.position.y;
    searchPoint.z = nowPos.position.z;

    // K nearest neighbor search
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    if ( kdtree->nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
        /*if(wayPointIdx != pointIdxNKNSearch[0]){
            std::cout<<"near way point update!"<<std::endl;
        }*/
        wayPointIdx = pointIdxNKNSearch[0];
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpNearSearch");
    wpNearSearch wpNS;
    wpNS.update();
    
    return 0;
}