/**
 * Trig.h header file for trig calculations
 * Pass in one or two beacons seen, the distance to them and the offset from the centre in radians
 */

#include <geometry_msgs/PoseWithCovariance.h>
#include <assign1_2013/beacons.h>

void getVoPose(geometry_msgs::PoseWithCovariance *ret, struct beacon left, long dLeft, float aLeft,
		 struct beacon right, long dRight, float aRight);