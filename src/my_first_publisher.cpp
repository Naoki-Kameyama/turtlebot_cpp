#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publisher_node");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise <std_msgs::String> ("my_topic", 1000);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    std_msgs::String msg;
    msg.data = "hello! thank you to subscribed this topic naoki";
    ROS_DEBUG("%s", msg.data.c_str());
    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
