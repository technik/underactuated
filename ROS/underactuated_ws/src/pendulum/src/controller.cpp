#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle n;

  ros::Publisher controller_pub = n.advertise<geometry_msgs::Wrench>("control_u", 1);
  ros::Rate loop_rate(250);

  while (ros::ok())
  {
    geometry_msgs::Wrench control_u;

    controller_pub.publish(control_u);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}