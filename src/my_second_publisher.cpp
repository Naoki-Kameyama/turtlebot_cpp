#include <ros/ros.h>
#include <std_msgs/String.h>
#include <autonomous_mapping/my_msgType.h>

#include <sys/time.h>
#include <time.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_pub2");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise <autonomous_mapping::my_msgType> ("my_topic2", 1000);

  char date_[20], time_[20];
  struct tm *date;
  time_t now;
  int year, month, day, hour, minute, second;

  ros::Rate loop_rate(10);
  while(ros::ok()){
    time(&now);
    date = localtime(&now);

    year = date->tm_year + 1900;
    month = date->tm_mon + 1;
    day = date->tm_mday;
    hour = date->tm_hour;
    minute = date->tm_min;
    second = date->tm_sec;
    sprintf(date_, "%04d/%02d/%02d", year, month, day);
    sprintf(time_, "%02d:%02d:%02d", hour, minute, second);

    autonomous_mapping::my_msgType msg;
    msg.local_date = std::string(date_);
    msg.local_time = std::string(time_);
    msg.topic_msgs = "hello! thak you to subscribed this topic!";
    pub.publish(msg);
    //std::cout << &now << " : " << now << " : " << date << std::endl;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
