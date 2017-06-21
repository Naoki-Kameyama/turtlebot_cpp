#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <map_based_navigation/frontier.h>
#include "std_msgs/String.h" 
#include <sstream>
#include <sensor_msgs/LaserScan.h>


//フロンティアを検索する関数//
void scanner(const nav_msgs::OccupancyGrid::ConstPtr& msg){

//地図データを配列に格納//
	int fro_num;
	std::vector<float> fro_x;//見つけたフロンティアのx座標
	std::vector<float> fro_y;//見つけたフロンティアのy座標
	
	nav_msgs::MapMetaData info = msg->info;//地図の設定を取得
	std::vector<int8_t> data = msg->data;//地図の値を取得
	int x = info.width;//地図の横サイズ
	int y = info.height;//地図の縦サイズ
	int8_t map_array[x][y];//地図を行列に格納
	int frontier_flag[x][y];//探査済みと未探査の境界を判定するフラグ
	int i,j;//for文
	int k = 0;//for文

	ros::NodeHandle nnh ;
	ros::Publisher frontier_pub = nnh.advertise<map_based_navigation::frontier>("frontier", 1000);
	map_based_navigation::frontier frontier;

	std::cout << "start:地図データを配列に格納" << std::endl;

	for(i=0;i<y;i++){
    		for(j=0;j<x;j++){
      			map_array[j][i] = data[k];
			if(map_array[j][i]!=0 && map_array[j][i]!=100 && map_array[j][i]!=-1){
					std::cout << "exception:" << map_array[j][i] << std::endl;		
			}
			frontier_flag[j][i] = 0;
      			k++;
    		}
  	}
	std::cout << "end  :地図データを配列に格納" << std::endl;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//横方向で未探査と探査済の境界を探す//

	std::cout << "start:横方向で境界を検索" << std::endl;

	for(i=0;i<y;i++){
    		for(j=0;j<(x-1);j++){
      			if(map_array[j][i] == 0 && map_array[j+1][i] == -1){
	       			frontier_flag[j][i] = 1;
			}
        		else if(map_array[j][i] == -1 && map_array[j+1][i] == 0){
				frontier_flag[j+1][i] = 1;	
			}
    		}
  	}

	std::cout << "end  :横方向で境界を検索" << std::endl;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
//縦方向で未探査と探査済の境界を探す/////////////////////////////////////////////////////////////////////////////

	std::cout << "start:縦方向で境界を検索" << std::endl;

	for(j=0;j<x;j++){
    		for(i=0;i<(y-1);i++){
      			if(map_array[j][i] == 0 && map_array[j][i+1] == -1){
	       			frontier_flag[j][i] = 1;
			}
        		else if(map_array[j][i] == -1 && map_array[j][i+1] == 0){
				frontier_flag[j][i+1] = 1;	
			}
    		}
  	}

	std::cout << "end  :縦方向で境界を検索" << std::endl;

////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
//横方向にフラグが8マス続いてる場所を探す////////////////////////////////////////////////////////////////////////
	float m_per_cell = info.resolution;//[m/cell]
	float robot_diameter = 0.4; //ロボットの直径[m]
	int robot_cellsize = robot_diameter / m_per_cell;//セル換算したロボットサイズ
	int frontier_sum;//フラグが続いているかの判定用
	float frontier_center;//フロンティア境界の中点
	float low_left_x = info.origin.position.x;//地図の左下のx座標
	float low_left_y = info.origin.position.y;//地図の左下のy座標
	fro_num = 0;//フロンティアの個数を初期化

	std::cout << "start:横方向で境界が連続している場所を検索" << std::endl;

	for(i=1;i<(y-1);i++){
		
    		for(j=0;j<(x-robot_cellsize);j++){
			frontier_sum = 0;
			for(k=j;k<(j+robot_cellsize);k++){
				frontier_sum=frontier_sum+frontier_flag[k][i];
				if(frontier_flag[k][i] == 0 && (frontier_flag[k][i-1] || frontier_flag[k][i+1])){
					frontier_sum++;
				}
			}
      			if(frontier_sum == robot_cellsize){
				frontier_center = (j+robot_cellsize-1)-(robot_cellsize/2);
				fro_x.push_back(frontier_center * m_per_cell + low_left_x);
				fro_y.push_back(low_left_y + (m_per_cell * i));
				fro_num++;
			}			
    		}
  	}
	std::cout << "end  :横方向で境界が連続している場所を検索" << std::endl;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

//縦方向にフラグが8マス続いてる場所を探す////////////////////////////////////////////////////////////////////////

	std::cout << "start:縦方向で境界が連続している場所を検索" << std::endl;

	for(j=1;j<(x-1);j++){
    		for(i=0;i<(y-robot_cellsize);i++){
			frontier_sum = 0;
			for(k=i;k<(i+robot_cellsize);k++){
				frontier_sum=frontier_sum+frontier_flag[j][k];
				if(frontier_flag[j][k] == 0 && (frontier_flag[j-1][k] || frontier_flag[j+1][k])){
					frontier_sum++;
				}	
			}
			if(frontier_sum == robot_cellsize){
				frontier_center = (i+robot_cellsize -1)-(robot_cellsize/2);
				fro_x.push_back(low_left_x + (m_per_cell * j));
				fro_y.push_back(frontier_center * m_per_cell + low_left_y);
				fro_num++;
			}
    		}
  	}
	std::cout << "end  :縦方向で境界が連続している場所を検索" << std::endl;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//見つけた領域をパブリッシュ///////////////////////////////////////////////////////////////////////////////////////
	frontier.fro_x = fro_x;
	frontier.fro_y = fro_y;
	frontier.fro_num = fro_num;
	frontier_pub.publish(frontier);

	std::cout << "＊＊＊＊＊＊＊＊＊＊見つけた領域を配信＊＊＊＊＊＊＊＊＊＊\n" << std::endl;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////	


//360[deg]回転////////////////////////////////////////////////////////////////////////////////////////////////
void robot_rotate(){
  	ros::NodeHandle nhhh ;
	ros::Publisher vel_pub = nhhh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
	geometry_msgs::Twist vel;
	vel.angular.z = 0.5;
	ros::Duration timeout(16.8);	
	ros::Time start_time = ros::Time::now();
	
	while(ros::Time::now() - start_time < timeout){
		ros::Rate rate(10.0);
		vel_pub.publish(vel);
		rate.sleep();
	}
}


int main(int argc, char** argv){

  	ros::init(argc, argv, "scanner"); //ノード名を定義しノードの初初期化を行う
  	ros::NodeHandle nh ; //ノードハンドラのインスタンスを生成
	
	ros::Rate rate(1);
	while(ros::ok()) {
	std::cout << "start" << std::endl;
	ros::Subscriber sub = nh.subscribe("/scan",1000,scanner);
	rate.sleep();
	}
	return 0;
}
