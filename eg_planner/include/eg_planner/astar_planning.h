#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace ctr{

class a_star
{
public:
    a_star(double resolution_, int costmapThreshold_, double w_gain_ = 1.0);
    bool planning(std::vector<double>& fx, std::vector<double>& fy, const geometry_msgs::Pose& startPos, const geometry_msgs::Pose& goalPos, const nav_msgs::OccupancyGrid& costmap);


private:
    struct Node
    {
        int x;
        int y;
        double sum_cost;
        Node* p_node = NULL;
    };

    std::vector<Node> get_motion_model();
    bool verify_node(Node* node, int min_ox, int max_ox, int min_oy, int max_oy);
    int getCost(double x, double y);
    void calc_final_path(Node* goal, std::vector<double>& fx, std::vector<double>& fy);

    int xwidth;
    int ywidth;
    double resolution;
    int costmapThreshold;
    double w_gain;
    nav_msgs::OccupancyGrid costmap;
};

a_star::a_star(double resolution_, int costmapThreshold_, double w_gain_) : resolution(resolution_), costmapThreshold(costmapThreshold_), w_gain(w_gain_)
{

}

bool a_star::planning(std::vector<double>& fx, std::vector<double>& fy, const geometry_msgs::Pose& startPos, const geometry_msgs::Pose& goalPos, const nav_msgs::OccupancyGrid& costmap_)
{
    costmap = costmap_;
    double sx = startPos.position.x;
    double sy = startPos.position.y;
    double gx = goalPos.position.x;
    double gy = goalPos.position.y;

    if(abs(gx - sx)>costmap.info.width*costmap.info.resolution/2 || abs(gy - sy)>costmap.info.width*costmap.info.resolution/2){
        std::cout<<"goal pose cross over costmap range!"<<std::endl;
        return false;
    }


    Node* nstart = new Node{(int)std::round(sx/resolution), (int)std::round(sy/resolution), 0.0};
    Node* ngoal = new Node{(int)std::round(gx/resolution), (int)std::round(gy/resolution), 0.0};

    int min_ox = (int)std::round(costmap.info.origin.position.x/resolution);
    int max_ox = (int)std::round((costmap.info.origin.position.x + costmap.info.resolution * costmap.info.width)/resolution);
    int min_oy = (int)std::round(costmap.info.origin.position.y/resolution);
    int max_oy = (int)std::round((costmap.info.origin.position.y + costmap.info.resolution * costmap.info.height)/resolution);

    xwidth = max_ox-min_ox;
    ywidth = max_oy-min_oy;

    std::vector<std::vector<int>> visit_map(xwidth, std::vector<int>(ywidth, 0));
    std::vector<std::vector<double>> path_cost(xwidth, std::vector<double>(ywidth, std::numeric_limits<double>::max()));

    //position jump measure
    if(!((nstart->x-min_ox>=0 && nstart->x-min_ox<xwidth) && (nstart->y-min_oy>=0 && nstart->y-min_oy<ywidth))){
        return false;
    }
    path_cost[nstart->x-min_ox][nstart->y-min_oy] = 0;

    auto cmp = [](const Node* left, const Node* right){return left->sum_cost > right->sum_cost;};
    std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> pq(cmp);
    pq.push(nstart);

    std::vector<Node> motion = get_motion_model();

    while(true)
    {
        if(pq.empty()){
            std::cout<<"path don't find!"<<std::endl;
            //delete ngoal;
            //delete nstart;
            return false;
        }
        Node* node = pq.top();

        if(visit_map[node->x-min_ox][node->y-min_oy] == 1){
            //searched area
            pq.pop();
            delete node;
            continue;
        }else{
            //unsearched area
            pq.pop();
            visit_map[node->x-min_ox][node->y-min_oy] = 1;
        }

        //goal
        if(node->x == ngoal->x && node->y == ngoal->y){
            ngoal->sum_cost = node->sum_cost;
            ngoal->p_node = node;
            break;
        }

        for(int i=0; i<motion.size(); i++){
            //search new node according to motion model
            double distance_goal = sqrt(pow(ngoal->x - (node->x + motion[i].x), 2) + pow(ngoal->y - (node->y + motion[i].y), 2));
            Node* new_node = new Node{node->x + motion[i].x, node->y + motion[i].y,
                           path_cost[node->x-min_ox][node->y-min_oy] + motion[i].sum_cost + w_gain*distance_goal + getCost(node->x*resolution, node->y*resolution),
                           node};

            //avoid obstract area
            if (!verify_node(new_node, min_ox, max_ox, min_oy, max_oy)){
                delete new_node;
                continue;
            }
            //searched area
            if (visit_map[new_node->x-min_ox][new_node->y-min_oy]){
                delete new_node;
                continue;
            }

            //std::cout<<"node_x:"<<node->x-min_ox<<" node_y:"<<node->y-min_oy<<" new_node_x:"<<new_node->x-min_ox<<" new_node_y:"<<new_node->y-min_oy<<std::endl;

            //select min cost path
            /*if (path_cost[node->x][node->y]+motion[i].sum_cost < path_cost[new_node->x][new_node->y]){
                path_cost[new_node->x][new_node->y]=path_cost[node->x][node->y]+motion[i].sum_cost;
                pq.push(new_node);
            }*/
            if (new_node->sum_cost < path_cost[new_node->x-min_ox][new_node->y-min_oy]){
                path_cost[new_node->x-min_ox][new_node->y-min_oy] = new_node->sum_cost;
                pq.push(new_node);
            }

        }
    }


    calc_final_path(ngoal, fx, fy);
    delete ngoal;
    delete nstart;

    return true;

}

std::vector<a_star::Node> a_star::get_motion_model()
{
    //avoid result vibration
    static double bias = 0.1;

    return {Node{1, 0, 1+bias},
            Node{0, 1, 1+bias},
            Node{-1, 0, 1},
            Node{0, -1, 1},
            Node{-1, -1, sqrt(2)},
            Node{-1, 1, sqrt(2)},
            Node{1, -1, sqrt(2)+bias},
            Node{1, 1, sqrt(2)+bias}};
}

bool a_star::verify_node(Node* node, int min_ox, int max_ox, int min_oy, int max_oy)
{
    if(node->x < min_ox || node->y < min_oy || node->x >= max_ox || node->y >= max_oy){
        return false;
    }
    if(getCost(node->x* resolution, node->y* resolution) >= costmapThreshold){
        return false;
    }

    return true;
}

int a_star::getCost(double x, double y)
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

void a_star::calc_final_path(Node* goal, std::vector<double>& fx, std::vector<double>& fy)
{
    std::vector<double> rx;
    std::vector<double> ry;
    Node* node = goal;

    while(node->p_node != NULL)
    {
        node = node->p_node;
        rx.push_back(node->x * resolution);
        ry.push_back(node->y * resolution);

    }
    //delete final index(now robot position)
    rx.pop_back();
    ry.pop_back();

    //inverse
    std::reverse(std::begin(rx), std::end(rx));
    std::reverse(std::begin(ry), std::end(ry));

    fx = rx;
    fy = ry;
}


}//namespace ctr