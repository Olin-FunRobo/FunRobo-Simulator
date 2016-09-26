#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>

constexpr double rad(double _deg){
	return _deg * M_PI / 180;
}
constexpr double deg(double _rad){
	return _rad * 180 / M_PI;
}

const double EARTH_CIRCUM = 40.075*1000000;
const double sx = 360 / EARTH_CIRCUM; //degree per distance
const double sy = 360 / EARTH_CIRCUM * std::cos(rad(42.293460)); //degree per distance

sensor_msgs::NavSatFix fix_msg;
geometry_msgs::TwistStamped vel_msg;
sensor_msgs::TimeReference time_msg;

ros::Publisher fix_pub;
ros::Publisher vel_pub;
ros::Publisher time_pub;

ros::Subscriber sub;

void translate(const gazebo_msgs::ModelStates::ConstPtr& local_msg){

	int n = local_msg->name.size();
	for(int i=0; i<n; ++i){
		if(local_msg->name[i] == "funrobo"){

			fix_msg.header.stamp = ros::Time::now();
			fix_msg.header.frame_id = ""; //global...?

			fix_msg.status.status = fix_msg.status.STATUS_FIX;
			fix_msg.status.service = fix_msg.status.SERVICE_GPS;

			auto& p = local_msg->pose[i].position;

			fix_msg.longitude = -71.263935 + p.x*sx;
			fix_msg.latitude = 42.293460 + p.y*sy;
			fix_msg.altitude = p.z;

			fix_msg.position_covariance_type = fix_msg.COVARIANCE_TYPE_UNKNOWN;

			fix_pub.publish(fix_msg);

			break;
		}
	}

}

int main(int argc, char** argv){

	ros::init(argc,argv,"gps");

	ros::NodeHandle nh("gps");

	fix_pub = nh.advertise<sensor_msgs::NavSatFix>("fix",1000);
	vel_pub = nh.advertise<geometry_msgs::TwistStamped>("vel",1000);
	time_pub = nh.advertise<sensor_msgs::TimeReference>("time",1000);

	sub = nh.subscribe("/gazebo/model_states", 100, translate);
	ros::spin();
	return 0;
}
