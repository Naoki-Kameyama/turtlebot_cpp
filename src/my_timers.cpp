#include <ros/ros.h>
#include <boost/function.hpp>

void timer_callback(const ros::TimerEvent&)
{
  ROS_INFO("Timer Callback triggered");
}

void wall_timer_callback(const ros::TimerEvent&)
{
  ROS_INFO("Wall-Timer Callback triggered");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Timers");
  ros::NodeHandle nh;

  ros::Timer timer;
  timer = nh.createTimer(ros::Duration(0.1), timer_callback);
  ros::WallTimer wall_timer;
  wall_timer = nh.createWallTimer(ros::WallDuration(1.0), wall_timer_callback);
  ros::spin();
  return 0;
}
