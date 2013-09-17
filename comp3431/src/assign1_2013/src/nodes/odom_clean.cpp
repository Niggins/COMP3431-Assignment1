/**
 * Remove orientation from odom message
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

class odom_clean {
private:
	ros::NodeHandle nh;
	ros::Subscriber odomSub;
	ros::Publisher odomPub;

	void odomCallback(const nav_msgs::Odometry &msg){
		nav_msgs::Odometry odom;
		odom.header = msg.header;
		odom.child_frame_id = msg.child_frame_id;
		odom.pose = msg.pose;
		odom.twist = msg.twist;
		odom.twist.covariance[35] = 9999;
		odomPub.publish(odom);
	}

public:
	odom_clean(){
    odomPub = nh.advertise<nav_msgs::Odometry>("clean_odom", 1);
    odomSub = nh.subscribe("odom", 1, &odom_clean::odomCallback, this);
	}

};

int main(int argc, char** argv){
	ros::init(argc, argv, "odom_clean");
	odom_clean o;
	ros::spin();
	return 0;
}