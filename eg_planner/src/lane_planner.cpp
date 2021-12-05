#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <string>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include "eg_planner/tf_position.h"

nav_msgs::Path path;
void path_callback(const nav_msgs::Path& path_message)
{
    path = path_message;

}

nav_msgs::OccupancyGrid costmap;
void cost_callback(const nav_msgs::OccupancyGrid& costmap_message)
{
    costmap = costmap_message;

}

nav_msgs::Odometry odom;
void odom_callback(const nav_msgs::Odometry& odom_message)
{
    odom = odom_message;
}

bool planning_stop = false;
void planningStop_callback(const std_msgs::Bool planningStop_message)
{
    planning_stop = planningStop_message.data;
}



std_msgs::ColorRGBA set_color(double r, double g, double b, double a)
{
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;

    return color;
}
std_msgs::ColorRGBA green = set_color(0.0, 1.0, 0.0, 1.0);
std_msgs::ColorRGBA red = set_color(1.0, 0.0, 0.0, 1.0);

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

double getCost(double x, double y)
{
    //x = costmap.info.origin.position.x + x_grid*costmap.info.resolution
    int x_grid = abs((x - costmap.info.origin.position.x)/costmap.info.resolution);
    int y_grid = abs((y - costmap.info.origin.position.y)/costmap.info.resolution);

    int data_pos = costmap.info.width * y_grid + x_grid;

    if(data_pos > 0 && data_pos < costmap.data.size()){
        return costmap.data[data_pos];
    }else{
        return -INFINITY;
    }
}

template <class T> std::vector<T> normalize(std::vector<T>& num, T amin, T amax)
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
        n = (amax - amin)*(n - xmin) / (xmax - xmin);
    }
    return num;
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

double poseStampDistance(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2)
{
    double diffX = pose1.pose.position.x - pose2.pose.position.x;
    double diffY = pose1.pose.position.y - pose2.pose.position.y;
    double diffZ = pose1.pose.position.z - pose2.pose.position.z;

    return sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lane_planner");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~/lane_planner");

    std::string base_link_id, map_id;
    pnh.param<std::string>("base_link_frame_id", base_link_id, "base_link");
    pnh.param<std::string>("map_frame_id", map_id, "map");
    double l_width, v_width;
    pnh.param<double>("lane_width", l_width, 2.5);
    pnh.param<double>("vehicle_width", v_width, 0.8);
    int sampling_num;
    pnh.param<int>("sampling_number", sampling_num, 7);
    double distanceCost_gain, obstacleCost_gain;
    pnh.param<double>("distanceCost_gain", distanceCost_gain, 1.0);
    pnh.param<double>("obstacleCost_gain", obstacleCost_gain, 1.0);
    double obst_sense_dis;
    pnh.param<double>("obstacle_sense_distance", obst_sense_dis, 2.0);
    double markerSize;
    pnh.param<double>("markerSize", markerSize, 1.0);
    double maxVel;
    pnh.param<double>("maxVelocity", maxVel, 1.0);
    double disTarget_maxVel;
    pnh.param<double>("distanceTarget_maxVelocity", disTarget_maxVel, 1.0);
    double distance_rate;
    pnh.param("distance_rate", distance_rate, 0.3);
    double rate;
    pnh.param<double>("loop_rate", rate, 30);

    ros::Subscriber path_sub = nh.subscribe("path", 50, path_callback);
    ros::Subscriber cost_sub = nh.subscribe("lane_planner/costmap", 10, cost_callback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 10, odom_callback);
    ros::Subscriber planningStop_sub = nh.subscribe("lane_planner/planning_stop", 1, planningStop_callback);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose>("lane_planner/pose_out", 10);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("lane_planner/marker_array", 10);

    ros::Rate loop_rate(rate);

    tf_position nowPosition(map_id, base_link_id, rate);

    while(ros::ok())
    {
        //planning stop
        if(planning_stop){
            geometry_msgs::Pose pose;
            pose.position.x = path.poses.back().pose.position.x;
            pose.position.y = path.poses.back().pose.position.y;
            pose.position.z = path.poses.back().pose.position.z;
            pose.orientation = path.poses.back().pose.orientation;
            pose_pub.publish(pose);

            ros::spinOnce();
            loop_rate.sleep();

            continue;
        }

        if(path.poses.size() > 0){
            //change targer pose interval following to robot velocity
            double target_interval = ((1-distance_rate)*disTarget_maxVel/maxVel)*abs(odom.twist.twist.linear.x) + distance_rate*disTarget_maxVel;
            //updata target way point to let distance be target_deviation
            int targetPose = 1;
            while(!(poseStampDistance(path.poses[targetPose], nowPosition.getPoseStamped()) >= target_interval))
            {
                // path end point
                if(targetPose >= (path.poses.size()-1)){
                    break;
                }
                targetPose++;
            }

            std::vector<double> lane_center_x;
            std::vector<double> lane_center_y;
            std::vector<double> lane_heading;
            //calculate cost using point from start to targetPose
            for(int i=0; i<targetPose; i++){
                int index = constrain(i, 0, path.poses.size());

                lane_center_x.push_back(path.poses[index].pose.position.x);
                lane_center_y.push_back(path.poses[index].pose.position.y);
                lane_heading.push_back(arrangeAngle(quat2yaw(path.poses[index].pose.orientation)));
            }



            //set target point and calculate each costs
            std::vector<std::vector<double>> targetPoint;
            std::vector<double> distances;
            std::vector<double> costs;

            for(int i=0; i<sampling_num; i++){
                //calculate cost using 3 way points
                double distance = 0;
                double cost = 0;
                for(int j=0; j<targetPose; j++){
                    //set target point
                    double delta = (l_width - v_width)*i/(sampling_num-1) - 0.5*(l_width -v_width);
                    double xf = lane_center_x[j] - delta*sin(lane_heading[j]);
                    double yf = lane_center_y[j] + delta*cos(lane_heading[j]);
                    std::vector<double> temp{xf, yf, lane_heading[j]};

                    //set array target way point
                    if(j==targetPose-1){
                        targetPoint.push_back(temp);
                    }

                    //calculate cost
                    distance += sqrt(pow(xf - lane_center_x[j], 2)+pow(yf - lane_center_y[j], 2));
                    cost += getCost(xf, yf);
                }

                distances.push_back(distance);
                costs.push_back(cost);
            }


            //-> 0~100*repeatNum
            distances = normalize(distances, 0.0, 100.0*targetPose);

            //select best path
            double minCost = INFINITY;
            int bestPathNum;
            for(int i=0; i<sampling_num; i++){
                double final_cost = distanceCost_gain * distances[i] + obstacleCost_gain * costs[i];

                if(final_cost < minCost){
                    minCost = final_cost;
                    bestPathNum = i;
                }
            }

            //create marker array
            // config arrow shape
            geometry_msgs::Vector3 arrow;
            arrow.x = 0.02 * markerSize;
            arrow.y = 0.04 * markerSize;
            arrow.z = 0.1 * markerSize;

            visualization_msgs::MarkerArray marker_array;
            marker_array.markers.resize(sampling_num);
            for(int i=0; i<sampling_num; i++){
                geometry_msgs::Point linear_start;
                linear_start.x = targetPoint[i][0];
                linear_start.y = targetPoint[i][1];
                linear_start.z = path.poses[targetPose].pose.position.z + 0.1;
                geometry_msgs::Point linear_end;
                linear_end.x = 0.2 * markerSize * cos(targetPoint[i][2]) + targetPoint[i][0];
                linear_end.y = 0.2 * markerSize * sin(targetPoint[i][2]) + targetPoint[i][1];
                linear_end.z = path.poses[targetPose].pose.position.z + 0.1;


                marker_array.markers[i].header.frame_id = map_id;
                marker_array.markers[i].header.stamp = ros::Time::now();
                marker_array.markers[i].ns = "plannig_point";
                marker_array.markers[i].id = i;
                marker_array.markers[i].lifetime = ros::Duration(1.0);

                marker_array.markers[i].type = visualization_msgs::Marker::ARROW;
                marker_array.markers[i].action = visualization_msgs::Marker::ADD;
                marker_array.markers[i].scale = arrow;

                marker_array.markers[i].points.resize(2);
                marker_array.markers[i].points[0] = linear_start;
                marker_array.markers[i].points[1] = linear_end;

                if(i == bestPathNum){
                    marker_array.markers[i].color = red;
                }else{
                    marker_array.markers[i].color = green;
                }
            }

            //best pose head to target point
            geometry_msgs::Pose pose;
            pose.position.x = targetPoint[bestPathNum][0];
            pose.position.y = targetPoint[bestPathNum][1];
            pose.position.z = path.poses[targetPose].pose.position.z;
            pose.orientation = path.poses[targetPose].pose.orientation;

            pose_pub.publish(pose);
            marker_pub.publish(marker_array);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
