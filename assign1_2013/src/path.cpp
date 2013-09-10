/*
 * beacons.cpp
 *
 *  Created on: 25/08/2013
 *      Author: rescue
 */


#include <assign1_2013/path.h>
#include <ros/ros.h>
#include <XmlRpcException.h>

#include <assign1_2013/beacons.h>

namespace comp3431 {

Path::Path() {
	ros::NodeHandle nh;
	XmlRpc::XmlRpcValue pathCfg;
	nh.getParam("/path", pathCfg);
	try {
		int i = 0;
		do {
			char pointName[256];
			sprintf(pointName, "point%d", i);
			if (!pathCfg.hasMember(pointName))
				break;

			XmlRpc::XmlRpcValue pointCfg = pathCfg[std::string(pointName)];
			geometry_msgs::Point p;
			if (!(pointCfg.hasMember("x") && pointCfg.hasMember("y")))
				continue;

			p.x = FLOAT_PARAM(pointCfg["x"]);
			p.y = FLOAT_PARAM(pointCfg["y"]);
			p.z = 0;

			points.push_back(p);
		} while ((++i) != 0);
	} catch (XmlRpc::XmlRpcException& e) {
		ROS_ERROR("Unable to parse path parameter. (%s)", e.getMessage().c_str());
	}
}

} // namespace comp3431
