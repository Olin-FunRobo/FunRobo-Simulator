#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/NavSatFix.h>

constexpr double rad(double _deg){
	return _deg * M_PI / 180;
}
constexpr double deg(double _rad){
	return _rad * 180 / M_PI;
}

const double EARTH_CIRCUM = 40.075*1000000;
const double sx = 360 / EARTH_CIRCUM; //degree per distance
const double sy = 360 / EARTH_CIRCUM * std::cos(rad(42.293460)); //degree per distance

sensor_msgs::NavSatFix gps_msg;
ros::Publisher pub;
ros::Subscriber sub;

void translate(const gazebo_msgs::ModelStates::ConstPtr& local_msg){

	int n = local_msg->name.size();
	for(int i=0; i<n; ++i){
		if(local_msg->name[i] == "olin"){

			gps_msg.header.stamp = ros::Time::now();
			gps_msg.header.frame_id = ""; //global...?

			gps_msg.status.status = gps_msg.status.STATUS_FIX;
			gps_msg.status.service = gps_msg.status.SERVICE_GPS;

			auto& p = local_msg->pose[i].position;

			gps_msg.longitude = -71.263935 + p.x*sx;
			gps_msg.latitude = 42.293460 + p.y*sy;
			gps_msg.altitude = p.z;

			gps_msg.position_covariance_type = gps_msg.COVARIANCE_TYPE_UNKNOWN;

			pub.publish(gps_msg);

			break;
		}
	}

}

int main(int argc, char** argv){
	ros::init(argc,argv,"gps");
	ros::NodeHandle nh;
	pub = nh.advertise<sensor_msgs::NavSatFix>("/gps",1000);
	sub = nh.subscribe("/gazebo/model_states", 100, translate);
	ros::spin();
	return 0;
}
