#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <string>
#include <vector>
#include <math.h>
#include "eg_planner/astar_planning.h"
#include "eg_planner/bezier_curve.h"
#include "eg_planner/tf_position.h"

geometry_msgs::PoseStamped goalPoseStamp;
void poseStamp_callback(const geometry_msgs::PoseStamped& poseStamp_message)
{
    goalPoseStamp = poseStamp_message;
}

nav_msgs::OccupancyGrid costmap;
void cost_callback(const nav_msgs::OccupancyGrid& costmap_message)
{
    costmap = costmap_message;
}

bool planning_stop = false;
void planningStop_callback(const std_msgs::Bool planningStop_message)
{
    planning_stop = planningStop_message.data;
}



int getCost(double x, double y)
{
    //x = costmap.info.origin.position.x + x_grid*costmap.info.resolution
    int x_grid = abs((x - costmap.info.origin.position.x)/costmap.info.resolution);
    int y_grid = abs((y - costmap.info.origin.position.y)/costmap.info.resolution);

    int data_pos = costmap.info.width * y_grid + x_grid;

    if(data_pos > 0 && data_pos < costmap.data.size()){
        return costmap.data[data_pos];
    }else{
        return std::numeric_limits<int>::min();
    }
}

geometry_msgs::Quaternion yaw_to_geometry_quat(double yaw){
    double roll = 0;
    double pitch = 0;
    tf::Quaternion quat=tf::createQuaternionFromRPY(roll,pitch,yaw);
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);
    return geometry_quat;
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
    ros::init(argc, argv, "astar_planning_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    int costmapThreshold;
    pnh.param<int>("costmapThreshold", costmapThreshold, 90);
    double heuristic_gain, resolution;
    pnh.param<double>("heuristic_gain", heuristic_gain, 1.0);
    pnh.param<double>("resolution", resolution, 0.1);
    double replanningRadius;
    pnh.param<double>("replanning_radius", replanningRadius, 1.0);
    int approNode;
    double approResolution;
    pnh.param<int>("approximationNode", approNode, 15);
    pnh.param<double>("approximationResolution", approResolution, 10);
    double rate;
    pnh.param<double>("loop_rate", rate, 10);
    std::string map_id, base_link_id;
    pnh.param<std::string>("map_frame_id", map_id, "map");
    pnh.param<std::string>("base_link_frame_id", base_link_id, "base_link");

    ros::Subscriber goalPose_sub = nh.subscribe("astar_plannnig_node/goal", 50, poseStamp_callback);
    ros::Subscriber cost_sub = nh.subscribe("astar_plannnig_node/costmap", 10, cost_callback);
    ros::Subscriber planningStop_sub = nh.subscribe("astar_plannnig_node/planning_stop", 1, planningStop_callback);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("astar_plannnig_node/path", 10);
    ros::Publisher bool_pub = nh.advertise<std_msgs::Bool>("astar_planning_node/successPlan", 10);

    ctr::a_star star(resolution, costmapThreshold, heuristic_gain);
    bezier_curve bezier;
    tf_position nowPosition(map_id, base_link_id, rate);

    std_msgs::Bool isSuccessPlanning;
    geometry_msgs::PoseStamped startPose;
    geometry_msgs::PoseStamped goalPose;

    ros::Rate loop_rate(rate);
    while(ros::ok())
    {
        if(planning_stop){
            //path message contains only a goalPoseStamp
            nav_msgs::Path plan_path;
            plan_path.header.frame_id = map_id;
            plan_path.header.stamp = ros::Time::now();
            plan_path.poses.push_back(goalPoseStamp);

            path_pub.publish(plan_path);

            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }

        if(costmap.data.size()>0){

            std::vector<double> path_x, path_y;
            //first pose stay in around obstacles
            if(getCost(nowPosition.getPose().position.x, nowPosition.getPose().position.y)<costmapThreshold){
                startPose = nowPosition.getPoseStamped();
                isSuccessPlanning.data = true;
            }else{
                //resampling start pose (8 positions)
                for(int i=0; i<8; ++i){
                    startPose.pose.position.x = nowPosition.getPose().position.x + replanningRadius*cos(2*M_PI*i/8);
                    startPose.pose.position.y = nowPosition.getPose().position.y + replanningRadius*sin(2*M_PI*i/8);
                    if(getCost(startPose.pose.position.x, startPose.pose.position.y)<costmapThreshold){
                        isSuccessPlanning.data = true;
                        break;
                    }
                    if(i==7){
                        isSuccessPlanning.data = false;
                    }
                }
            }

            //fail start pose
            if(!isSuccessPlanning.data){
                //path message contains only a goalPoseStamp
                nav_msgs::Path plan_path;
                plan_path.header.frame_id = map_id;
                plan_path.header.stamp = ros::Time::now();
                plan_path.poses.push_back(goalPoseStamp);

                path_pub.publish(plan_path);
                bool_pub.publish(isSuccessPlanning);
               //continue;
            }

            if(star.planning(path_x, path_y, startPose.pose, goalPoseStamp.pose, costmap)){
                isSuccessPlanning.data = true;
                goalPose = goalPoseStamp;
            }else{
                isSuccessPlanning.data = false;
                bool_pub.publish(isSuccessPlanning);
                //resampling goal pose (8 positions)
                for(int i=0; i<8; ++i){
                    goalPose.pose.position.x = goalPoseStamp.pose.position.x + replanningRadius*cos(2*M_PI*i/8);
                    goalPose.pose.position.y = goalPoseStamp.pose.position.y + replanningRadius*sin(2*M_PI*i/8);
                    if(star.planning(path_x, path_y, startPose.pose, goalPose.pose, costmap)){
                        isSuccessPlanning.data = true;
                        break;
                    }
                    if(i==7){
                        isSuccessPlanning.data = false;
                    }
                }
            }

            //fail end pose
            if(!isSuccessPlanning.data){
                //path message contains only a goalPoseStamp
                nav_msgs::Path plan_path;
                plan_path.header.frame_id = map_id;
                plan_path.header.stamp = ros::Time::now();
                plan_path.poses.push_back(goalPoseStamp);

                path_pub.publish(plan_path);
                bool_pub.publish(isSuccessPlanning);
                //continue;
            }

            //approximate bezier curve
            int node;
            std::vector<double> r_x, r_y;
            for(int i=0; i<path_x.size()/approNode+1; i++){
                node = i*approNode;
                std::vector<double> p_x, p_y;
                if(i<path_x.size()/approNode){
                    for(int j=0; j<approNode; j++){
                        p_x.push_back(path_x[node+j]);
                        p_y.push_back(path_y[node+j]);
                    }
                }else{
                    if(path_x.size() - node == 0){
                        break;
                    }
                    for(int j=0; j<path_x.size() - node; j++){
                        p_x.push_back(path_x[node+j]);
                        p_y.push_back(path_y[node+j]);
                    }
                }

                for (double t = 0; t <= 1.0; t += (1/approResolution)){
                    std::array<double, 2> point = bezier.getPos(t, p_x, p_y);
                    r_x.push_back(point[0]);
                    r_y.push_back(point[1]);
                }
            }

            //path heading angle
            std::vector<double> angles;
            for(int i=0; i<r_x.size(); i++){
                double angle;

                //start point
                //angle : -pi~pi
                if(i==0){
                    double dx = r_x[i+1] - r_x[i];
                    double dy = r_y[i+1] - r_y[i];
                    angle = arrangeAngle(atan2(dy, dx));
                //end point
                }else if(i==r_x.size()-1){
                    double dx = r_x[i] - r_x[i-1];
                    double dy = r_y[i] - r_y[i-1];
                    angle = arrangeAngle(atan2(dy, dx));
                }else{
                    double dx1 = r_x[i] - r_x[i-1];
                    double dy1 = r_y[i] - r_y[i-1];
                    double angle1 = arrangeAngle(atan2(dy1, dx1));

                    double dx2 = r_x[i+1] - r_x[i];
                    double dy2 = r_y[i+1] - r_y[i];
                    double angle2 = arrangeAngle(atan2(dy2, dx2));

                    /*if(angle1-angle2<M_PI){
                        angle = (angle1 + angle2)/2;
                    }else{
                        angle = angle2;
                    }*/
                    //うまくいかないので一旦これで
                    angle = angle1;
                }
                angles.push_back(angle);

            }

            //convert path message
            nav_msgs::Path plan_path;
            for(int i=0; i<r_x.size(); i++){
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = map_id;
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = r_x[i];
                pose.pose.position.y = r_y[i];
                pose.pose.position.z = nowPosition.getPose().position.z;
                pose.pose.orientation = yaw_to_geometry_quat(angles[i]);

                plan_path.poses.push_back(pose);
            }
            plan_path.header.frame_id = map_id;
            plan_path.header.stamp = ros::Time::now();

            if(isSuccessPlanning.data){
                path_pub.publish(plan_path);
            }
            bool_pub.publish(isSuccessPlanning);
        }

        startPose = nowPosition.getPoseStamped();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
