#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>

class rsj_robot_test_node
{
  public:
    ros::Subscriber sub_odom;
	  ros::Subscriber sub_scan;
    ros::Publisher pub_twist;
    sensor_msgs::LaserScan latest_scan;

  //  void cb_odom(const nav_msgs::Odometry::ConstPtr& msg)
  //  {
//    }

    void cb_scan(const sensor_msgs::LaserScan::ConstPtr& msg)
	  {
		    latest_scan = *msg;
	  }


     rsj_robot_test_node()
    {
        ros::NodeHandle nh("~");
        pub_twist = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 5);
      //  sub_odom = nh.subscribe("/odom", 5,&rsj_robot_test_node::cb_odom, this);
		    sub_scan = nh.subscribe("/scan", 5,&rsj_robot_test_node::cb_scan, this);
    }

    void mainloop()
    {
      ros::Rate rate(10.0);
      while(ros::ok())
      {
        ros::spinOnce();

        std::cout << "distp : " <<  latest_scan.ranges.size() << std::endl;
        std::cout << "distr : " <<  latest_scan.ranges.size() << std::endl;

			  if(latest_scan.ranges.size() > 0)
			  {
				  // LaserScanメッセージをすでに受け取っている場合a
  				geometry_msgs::Twist cmd_vel;
			  	int i = (-latest_scan.angle_min) / latest_scan.angle_increment;
				  if(latest_scan.ranges[i+2] < latest_scan.range_min || // エラー値の場合
						latest_scan.ranges[i+2] > latest_scan.range_max || // 測定範囲外の場合
						std::isnan(latest_scan.ranges[i+2])) // 無限遠の場合
          {
            // 正面に十分な距離がある (測定範囲以上)
					  ROS_INFO("front-range: measurement error");
        //    std::cout << "dist : " <<  latest_scan.ranges[i+2] << std::endl;
					  cmd_vel.linear.x = 0.2;
					  cmd_vel.angular.z = 0.0;
				   }
				   else
				   {
					   ROS_INFO("front-range: %0.3f", latest_scan.ranges[i+2]);
					   if(latest_scan.ranges[i+2] > 0.57)
					   {
						   // 50cm以上距離がある
						   cmd_vel.linear.x = 0.2;
						   cmd_vel.angular.z = 0.0;
					    }
					    else
					    {
						   // 50cm以下になった
						   cmd_vel.linear.x = 0.0;
						   cmd_vel.angular.z = 0.0;
					    }
			      }
				    pub_twist.publish(cmd_vel);
			    }
          rate.sleep();
        }
    }
};



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rsj_robot_test_node");
    rsj_robot_test_node robot_test;
    robot_test.mainloop();
}
