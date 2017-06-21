#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
//#include <map_based_navigation/frontier.h>

void robot_rotate(){
  	ros::NodeHandle nhhh ;
	ros::Publisher pub_twist = nhhh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.2;
	cmd_vel.angular.z = 0.5;
	ros::Duration timeout(2);
	ros::Time start_time = ros::Time::now();

	while(ros::Time::now() - start_time < timeout){
		ros::Rate rate(10.0);
		pub_twist.publish(cmd_vel);
		rate.sleep();
  }
}


int main(int argc, char** argv){
	ros::init(argc, argv, "move_turtelbot");
	ros::NodeHandle nh ;

	//std::cout << "start:360°回転" << std::endl;
	robot_rotate();
	//std::cout << "end  :360°回転" << std::endl;
  //ros::Subscriber map_sub = nh.subscribe("/scan",1000,laser_scan);
	ros::spin();

	return 0;
}
