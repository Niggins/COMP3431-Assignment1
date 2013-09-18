#include <ros/ros.h>
#include <assign1_2013/beacons.h>
#include <assign1_2013/trig.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <cmath>

#define HIGH_COV 9999
#define LOW_COV 0.01
#define ONE_BEACON 0.2
#define NO_BEACONS 1
#define PI 3.1415926

//a nan value != itself
#define CHECKNAN(x,y) if (y == y) x = y
/**
* Logic: determine intersecting points from position and distances
* Determine which is correct one by which is on the left/right
* http://paulbourke.net/geometry/circlesphere/
*/

	void trig::getSinglePoint(geometry_msgs::Point *ret, geometry_msgs::Point left, long rLeft, 
		geometry_msgs::PoseWithCovariance prev){
	  /**
	   * Logic: use latest position and distance to beacon to determine most likely spot, pass with increased covariance
	   * Angle can be determined from the supposed location and the beacon
	   */
	    geometry_msgs::Point temp;
	    geometry_msgs::Point pos1;
	    geometry_msgs::Point pos2;
	    geometry_msgs::Point myAdj;

	    //myAdj allows assumption that circle is at 0,0
	    myAdj.x = prev.pose.position.x - left.x;
	    myAdj.y = prev.pose.position.y - left.y;

	    //temp is distance 1 away from previous known pos from 
	    temp.x = myAdj.x + cos(prev.pose.orientation.z);
	    temp.y = myAdj.y + sin(prev.pose.orientation.z);
	    //x1y2-x2y1 Discriminant
	   	double D = myAdj.x*temp.y - temp.x*myAdj.y;
	   	double delta = rLeft*rLeft - D*D;

	   	if (delta < 0) {
	   		//No intersection between line and circle
            double vX = prev.pose.position.x - left.x;
		    double vY = prev.pose.position.y - left.y;
		    double magV = sqrt(vX*vX + vY*vY);
		    ret->x = left.x + vX / (magV * rLeft);
		    ret->y = left.y + vY / (magV * rLeft);
			ret->z = 0.0;
		} else {
			//At least one connection
			int sign = 1;
			double dy = temp.y - myAdj.y;
			double dx = temp.x - myAdj.x; 
			if (dy < 0)
				sign = -1;
			pos1.x = D*dy + sign*dx*sqrt(delta);
			pos1.y = -1*D*dx + abs(dy)*sqrt(delta);
			pos2.x = D*dy - sign*dx*sqrt(delta);
			pos2.y = -1*D*dx - abs(dy)*sqrt(delta);

			double d1 = pow(myAdj.x-pos1.x, 2)+pow(myAdj.y-pos1.y, 2);
			double d2 = pow(myAdj.x-pos2.x, 2)+pow(myAdj.y-pos2.y, 2);
			if (d1 > d2) {
				ret->x = pos2.x + left.x;
				ret->y = pos2.y + left.y;
			} else {
				ret->x = pos1.x + left.x;
				ret->y = pos1.y + left.y;
			}
			ret->z = 0;
		}
		ROS_INFO("lx:%f ly:%f", left.x, left.y);
		ROS_INFO("x:%f y:%f", ret->x, ret->y);
	}

	void trig::getPoint(geometry_msgs::Point *ret, geometry_msgs::Point left, long rLeft,
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
		//ROS_INFO("x:%f y:%f", ret->x, ret->y);
	}

	/**
	* Logic: determine orientation by using position and angle to left
	* Orientation should be determined by angle if facing + left
	*/
	void trig::getOrientation(geometry_msgs::Quaternion *ret, geometry_msgs::Point *point,
	geometry_msgs::Point left, float aLeft){

		float toZero;
		if ((point->x - left.x) == 0) {
			toZero = PI/2;
		} else {
			toZero = atan((point->y - left.y)/(point->x - left.x));
		}
		ret->z = toZero + aLeft;
		ret->x = ret->y = ret->w = 0.0;
		ROS_INFO("point x:%f left x:%f  point y:%f left y: %f aLeft:%f toZero:%f", point->x, left.x, point->y, left.y, aLeft, toZero);
	}

	boost::array<double, 36ul> trig::setCovariance(double val){

		boost::array<double, 36ul> tmp = {{val , 0, 0, 0, 0, 0,
										0, val , 0, 0, 0, 0,
										0, 0, HIGH_COV, 0, 0, 0,
										0, 0, 0, HIGH_COV, 0, 0,
										0, 0, 0, 0, HIGH_COV, 0,
										0, 0, 0, 0, 0, val}};

		return tmp;
	}

	boost::array<double, 36ul> trig::increaseCov(boost::array<double, 36ul> prev, long inc){
		boost::array<double, 36ul> tmp = {{prev[0]+inc, 0, 0, 0, 0, 0,
		0, prev[7]+inc, 0, 0, 0, 0,
		0, 0, HIGH_COV, 0, 0, 0,
		0, 0, 0, HIGH_COV, 0, 0,
		0, 0, 0, 0, HIGH_COV, 0,
		0, 0, 0, 0, 0, prev[35]+inc}};

		return tmp;
	}

	trig::trig(){}

	void trig::getVoPose(geometry_msgs::PoseWithCovariance *ret, SpottedBeacon left, SpottedBeacon right, 
		geometry_msgs::PoseWithCovariance prev){
		ROS_INFO("Left: %f %f", left.distance, left.angle);
		if (right.beacon != NULL)
			ROS_INFO("Right: %f %f", right.distance, right.angle);
		if (left.beacon == NULL){
	  		//No beacons spotted
			ret->pose.position.x = prev.pose.position.x;
			ret->pose.position.y = prev.pose.position.y;
			ret->pose.position.z = prev.pose.position.z;
			ret->pose.orientation.x = prev.pose.orientation.x;
			ret->pose.orientation.y = prev.pose.orientation.y;
			ret->pose.orientation.z = prev.pose.orientation.z;
			ret->pose.orientation.w = prev.pose.orientation.w;
			ret->covariance = setCovariance(NO_BEACONS);
			//ROS_INFO("Trig Pose 0 x: %f, y:%f z:%f", ret->pose.position.x, ret->pose.position.y, ret->pose.orientation.z);		
			return;
		}
		else if (right.beacon == NULL) {
			//One beacon spotted
			//Get position closest to the last known position on the circle around the beacon
			//Get angle from this assumed position
			getSinglePoint(&ret->pose.position, left.beacon->position, left.distance, prev);
			getOrientation(&ret->pose.orientation, &ret->pose.position, left.beacon->position, left.angle);
			ret->covariance = setCovariance(ONE_BEACON);
			//ROS_INFO("Trig Pose 1 x: %f, y:%f z:%f", ret->pose.position.x, ret->pose.position.y, ret->pose.orientation.z);		

			return;      
    	}
    
		getPoint(&ret->pose.position, left.beacon->position, left.distance, right.beacon->position, right.distance);
		getOrientation(&ret->pose.orientation, &ret->pose.position, left.beacon->position, left.angle);
		ret->covariance = setCovariance(LOW_COV);

        //ROS_INFO("Trig Pose 2 x: %f, y:%f z:%f", ret->pose.position.x, ret->pose.position.y, ret->pose.orientation.z);		
	}

