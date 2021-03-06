/**
 * Trig.h header file for trig calculations
 * Pass in one or two beacons seen, the distance to them and the offset from the centre in radians
 */

#pragma once

#include <geometry_msgs/PoseWithCovariance.h>
#include <assign1_2013/beacons.h>
struct SpottedBeacon {
public:
	SpottedBeacon() {beacon = NULL;}
	void set(SpottedBeacon b) {beacon = b.beacon; distance = b.distance; angle = b.angle;}
	comp3431::Beacon *beacon;
	double distance;
	float angle;
};

class trig {
private:
	void getPoint(geometry_msgs::Point *ret, geometry_msgs::Point left, long rLeft,
	geometry_msgs::Point right, long rRight);
	void getSinglePoint(geometry_msgs::Point *ret, geometry_msgs::Point left, long rLeft, 
		geometry_msgs::PoseWithCovariance prev);
	void getOrientation(geometry_msgs::Quaternion *ret, geometry_msgs::Point *point,
	comp3431::Beacon left, float aLeft);
	void setPrev(geometry_msgs::PoseWithCovariance *ret);
	boost::array<double, 36ul> setCovariance(double val);
	boost::array<double, 36ul> increaseCov(boost::array<double, 36ul> prev, long inc);

	public:
		trig();
		void getVoPose(geometry_msgs::PoseWithCovariance *ret, SpottedBeacon left, SpottedBeacon right, 
			geometry_msgs::PoseWithCovariance prev);
};
