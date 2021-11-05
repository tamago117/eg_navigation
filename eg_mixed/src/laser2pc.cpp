#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

class laser2pc
{
public:
    laser2pc();

private:
    ros::NodeHandle nh;
    laser_geometry::LaserProjection projector;
    ros::Subscriber laser_sub;
    ros::Publisher pc_pub;

    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& laser_message);

};

laser2pc::laser2pc()
{
    laser_sub = nh.subscribe("scan", 10, &laser2pc::laser_callback, this);
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("laser2pc/pc2", 1);
}

void laser2pc::laser_callback(const sensor_msgs::LaserScan::ConstPtr& laser_message)
{
    sensor_msgs::PointCloud2 cloud;

    //transform LaserScan -> PointCloud2
    projector.projectLaser(*laser_message, cloud);

    pc_pub.publish(cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser2pc");
    laser2pc l2p;

    ros::spin();
    return 0;
}