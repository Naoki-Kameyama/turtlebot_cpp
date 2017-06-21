#include <ros/ros.h>
#include <std_msgs/String.h>
#include <autonomous_mapping/my_msgType.h>

#include <sys/time.h>
#include <time.h>


void messageCallback(const autonomous_mapping::my_msgType::ConstPtr& msg)
{
  ROS_INFO_STREAM("I heard: "
                  << msg->local_date << " "
                  << msg->local_time << " "
                  << msg->topic_msgs);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscriber_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("my_topic2", 1000, messageCallback);
  ros::spin();
  return 0;
}
