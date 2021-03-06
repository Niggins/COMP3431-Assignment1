/*
 * beacons.cpp
 *
 *  Created on: 25/08/2013
 *      Author: rescue
 */


#include <assign1_2013/beacons.h>
#include <ros/ros.h>
#include <XmlRpcException.h>

namespace comp3431 {

Beacons::Beacons() {
	ros::NodeHandle nh;
	XmlRpc::XmlRpcValue beaconsCfg;
	nh.getParam("/beacons", beaconsCfg);
	try {
		int i = 0;
		do {
			char beaconName[256];
			sprintf(beaconName, "beacon%d", i);
			if (!beaconsCfg.hasMember(beaconName))
				break;

			XmlRpc::XmlRpcValue beaconCfg = beaconsCfg[std::string(beaconName)];
			Beacon b;
			if (!(beaconCfg.hasMember("top") && beaconCfg.hasMember("bottom") &&
					beaconCfg.hasMember("x") && beaconCfg.hasMember("top")))
				continue;
			b.top = (std::string)beaconCfg[("top")];
			b.bottom = (std::string)beaconCfg["bottom"];
			b.position.x = FLOAT_PARAM(beaconCfg["x"]);
			b.position.y = FLOAT_PARAM(beaconCfg["y"]);
			b.position.z = 0;

			beacons.push_back(b);
		} while ((++i) != 0);
	} catch (XmlRpc::XmlRpcException& e) {
		ROS_ERROR("Unable to parse beacon parameter. (%s)", e.getMessage().c_str());
	}
}

comp3431::Beacon *Beacons::getBeacon(std::string const& topColor, std::string const& botColor) {
	for (int i = 0; i < beacons.size(); i++) {
		if (beacons[i].top == topColor && beacons[i].bottom == botColor)
			return &beacons[i];
	}
}

} // namespace comp3431
