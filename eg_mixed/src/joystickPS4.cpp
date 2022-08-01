#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

double v_control_rate;
double w_control_rate;

geometry_msgs::Twist cmd_vel;
void joy_callback(const sensor_msgs::Joy& joy_msg)
{
  cmd_vel.linear.x = v_control_rate * joy_msg.axes[4];
  cmd_vel.angular.z = w_control_rate * joy_msg.axes[0];
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystickPS4");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Subscriber joy_sub = nh.subscribe("joy", 10, joy_callback);

  pnh.param<double>("v_control_rate", v_control_rate, 1.0);
  pnh.param<double>("w_control_rate", w_control_rate, 1.0);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    cmd_pub.publish(cmd_vel);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}