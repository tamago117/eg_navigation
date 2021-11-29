#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <std_srvs/SetBool.h>

bool costmap_reset = false;
void costmapReset_callback(const std_msgs::Bool costmapReset_message)
{
    costmap_reset = costmapReset_message.data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "costmap");
    #if ROS_VERSION_MINIMUM(1,14,0)
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        costmap_2d::Costmap2DROS costmap("my_costmap", tfBuffer);
    #else
        tf::TransformListener tf(ros::Duration(10));
        costmap_2d::Costmap2DROS costmap("my_costmap", tf);
    #endif

    ros::NodeHandle nh;
    costmap.start();

    ros::Subscriber costmapReset_sub = nh.subscribe("costmap_node/my_costmap/costmap_reset", 1, costmapReset_callback);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        if(costmap_reset){
            costmap.resetLayers();
            costmap_reset = false;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    /*const int rate = 5;
    ros::Rate loop_rate(rate);
    int count = 0;
    while(ros::ok()){
      if(count == 5*rate){
        costmap.resetLayers();
        count = 0;
      }else{
        count++;
      }

      ros::spinOnce();
      loop_rate.sleep();
    }*/

    //ros::spin();
    return 0;
}
