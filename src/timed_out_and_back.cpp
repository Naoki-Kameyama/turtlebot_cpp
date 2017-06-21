#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "out_and_back");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);

  int rate = 50;

  double linear_speed = 0.5;
  double goal_distance = 1.5;
  double linear_duration = goal_distance/linear_speed;
  double angular_speed = 1.0;
  double goal_angle = 3*M_PI;

  ROS_INFO_STREAM("goal angular: " << goal_angle);

  double angular_duration = goal_angle/angular_speed;

  geometry_msgs::Twist move_cmd, empty;
  move_cmd.linear.x = 0.0;
  move_cmd.linear.y = 0.0;
  move_cmd.linear.z = 0.0;
  move_cmd.angular.x = 0.0;
  move_cmd.angular.y = 0.0;
  move_cmd.angular.z = 0.0;
  empty = move_cmd;

  ros::Rate r(rate);
  for(int i=0;i<2;i++){
    move_cmd.linear.x = linear_speed;
    int ticks = int(linear_duration * rate);

    for(int t=0;t<ticks;t++){
      pub.publish(move_cmd);
      r.sleep();
    }
    pub.publish(empty);
    ros::Duration(1.0).sleep();

    move_cmd.linear.x = 0;
    move_cmd.angular.z = angular_speed;

    ticks = int(angular_duration * rate);
    ROS_INFO_STREAM("angular ticks: " << ticks);

    for(int i=0;i<ticks;i++){
      pub.publish(move_cmd);
      r.sleep();
    }
    move_cmd.angular.z = 0;
    pub.publish(empty);
    ros::Duration(1.0).sleep();
  }
  pub.publish(empty);
  ros::Duration(1.0).sleep();
  return 0;
}
