#include <ros/ros.h>
#include <assign1_2013/beacons.h>
#include <assign1_2013/trig.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <cmath>

#define HIGH_COV 9999
#define LOW_COV 0.01
#define UP_COV 0.2
#define PI 3.1415926
/**
 * Logic: determine intersecting points from position and distances
 * Determine which is correct one by which is on the left/right
 * http://paulbourke.net/geometry/circlesphere/
 */

class trig { 
private:
	geometry_msgs::PoseWithCovariance prev;
	void getPoint(geometry_msgs::Point *ret, geometry_msgs::Point left, long rLeft, 
		geometry_msgs::Point right, long rRight){

		geometry_msgs::Point pointI; // perp intersection (check website)

		double d = sqrt(pow(left.x - right.x,2) + pow(left.y - right.y ,2)); // distance between poles
		double a = (pow(rLeft, 2) - pow(rRight, 2) + pow(d, 2))/(2*d); // distance between left & perp intersection
		double h = sqrt(abs(pow(rLeft, 2) - pow(a, 2))); // distance between perp intersection & an intersection

		pointI.x = left.x + (a*(right.x - left.x)/d);
		pointI.y = left.y + (a*(right.y - left.y)/d);

		ret->x = pointI.x + h*(right.x - left.x)/d;
		ret->y = pointI.y - h*(right.y - left.y)/d;
		ret->z = 0;
	}

	/**
	 * Logic: determine orientation by using position and angle to left 
	 * Orientation should be determined by angle if facing + left
	 */
	void getOrientation(geometry_msgs::Quaternion *ret, geometry_msgs::Point *point, 
		geometry_msgs::Point left, float aLeft){

		float toZero;
		if ((point->x - left.x) == 0) {
			toZero = PI/2;
		} else {
			toZero = atan((point->y - left.y)/(point->x - left.x));
		}
		ret->z = toZero - aLeft;
		ret->x = ret->y = ret->w = 0.0;
	}

	boost::array<double, 36ul> setCovariance(double val){

		boost::array<double, 36ul> tmp =  	{{val         , 0, 0, 0, 0, 0,
											  0, val         , 0, 0, 0, 0,
											  0, 0,     HIGH_COV, 0, 0, 0,
											  0, 0, 0,     HIGH_COV, 0, 0,
											  0, 0, 0, 0,     HIGH_COV, 0,
											  0, 0, 0, 0, 0,  		val}};

		return tmp;
	}

	boost::array<double, 36ul> increaseCov(boost::array<double, 36ul> prev){
		boost::array<double, 36ul> tmp =  	{{prev[0]+UP_COV, 0, 0, 0, 0, 0,
											  0, prev[7]+UP_COV, 0, 0, 0, 0,
											  0, 0,       HIGH_COV, 0, 0, 0,
											  0, 0, 0,       HIGH_COV, 0, 0,
											  0, 0, 0, 0,       HIGH_COV, 0,
											  0, 0, 0, 0, 0,  prev[35]+UP_COV}};

		return tmp;
	}

public:
	trig(){
		prev.pose.position.x = prev.pose.position.y = prev.pose.position.z = 0;
		prev.pose.orientation.x = prev.pose.orientation.y = prev.pose.orientation.z = 0;
	}

	void getVoPose(geometry_msgs::PoseWithCovariance *ret, comp3431::Beacon *left, long dLeft, float aLeft,
			 comp3431::Beacon *right, long dRight, float aRight){
		if (left == NULL || right == NULL){
			//Cannot be certain about the position or orientation
			//Increase since last response 
			ret->pose.position = prev.pose.position;
			ret->pose.orientation = prev.pose.orientation;
			ret->covariance = increaseCov(prev.covariance);
			return;
		}

		getPoint(&ret->pose.position, left->position, dLeft, right->position, dRight);
		getOrientation(&ret->pose.orientation, &ret->pose.position, left->position, aLeft);
		ret->covariance = setCovariance(LOW_COV);

		prev.pose.position = ret->pose.position;
		prev.pose.orientation = ret->pose.orientation;
		prev.covariance = ret->covariance;
	}
};