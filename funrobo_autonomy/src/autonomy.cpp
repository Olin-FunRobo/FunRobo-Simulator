#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

void handle_camera(const sensor_msgs::Image::ConstPtr& msg){
	//
}

int main(int argc, char* argv[]){
	ros::init(argc,argv, "autonomy");
	ros::NodeHandle n;
	ros::Subscriber camera_sub1 = n.subscribe("camera_1/image_raw", 10, &handle_camera);
	// ...
	return 0;
}
