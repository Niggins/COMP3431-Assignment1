/*
 * visual.cpp
 *
 *  Created on: 20/08/2013
 *      Author: rescue
 */

#include <ros/ros.h>
#include <assign1_2013/beacons.h>
#include <assign1_2013/path.h>


#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <tf/transform_listener.h>

#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>

#include <string.h>
#include <cmath>

namespace comp3431 {

#define FIELD_RESOLUTION        0.02

#define FIELD_LENGTH            9.0
#define FIELD_WIDTH				6.0
#define FIELD_BORDER            0.7
#define FIELD_LINE_THICKNESS    0.05
#define FIELD_CIRCLE_RADIUS     0.75
#define FIELD_BOX_LENGTH        0.6
#define FIELD_BOX_WIDTH         2.2
#define FIELD_CROSS_LENGTH      0.1
#define FIELD_SPOT_LENGTH       2.7

#define TURTLEBOT_RADIUS        0.16

#define ORIGIN_X                (FIELD_BORDER + FIELD_WIDTH / 2)
#define ORIGIN_Y                (FIELD_BORDER + FIELD_LENGTH / 2)

#define IMAGE_X(X)              (((X) + ORIGIN_X) / FIELD_RESOLUTION)
#define IMAGE_Y(Y)              ((ORIGIN_Y - (Y)) / FIELD_RESOLUTION)

#define DRAW_LINE(X1,Y1,X2,Y2,COLOUR,THICKNESS)  cv::line(*cvMat, cv::Point(IMAGE_X(X1), IMAGE_Y(Y1)), cv::Point(IMAGE_X(X2), IMAGE_Y(Y2)), (COLOUR), (THICKNESS))
#define DRAW_FIELD_LINE(X1,Y1,X2,Y2)  DRAW_LINE(X1,Y1,X2,Y2, white, lineThickness);


#define DEFAULT_BASE_FRAME		"base_link"
#define DEFAULT_FIELD_FRAME		"odom"

void drawBeacon(cv::Mat& mat, geometry_msgs::Point& p, cv::Scalar topColour, cv::Scalar bottomColour) {
	float centerX = IMAGE_X(p.x), centerY = IMAGE_Y(p.y);

	cv::Point rect[4];
	rect[0].x = centerX - 5;
	rect[0].y = centerY;
	rect[1].x = centerX + 5;
	rect[1].y = centerY;
	rect[2].x = centerX + 5;
	rect[2].y = centerY + 5;
	rect[3].x = centerX - 5;
	rect[3].y = centerY + 5;
	cv::fillConvexPoly(mat, rect, 4, bottomColour);

	rect[0].x = centerX - 5;
	rect[0].y = centerY;
	rect[1].x = centerX + 5;
	rect[1].y = centerY;
	rect[2].x = centerX + 5;
	rect[2].y = centerY - 5;
	rect[3].x = centerX - 5;
	rect[3].y = centerY - 5;
	cv::fillConvexPoly(mat, rect, 4, topColour);
}

class Visualiser {
public:
	Beacons beacons;
	Path path;
	std::vector< tf::Vector3 > history;
	ros::Publisher imagePub;
	ros::Subscriber odomSub;
	tf::TransformListener tfListener;

	std::string baseFrame, fieldFrame;

	nav_msgs::OdometryConstPtr latestOdom;

	cv::Scalar white, pink, yellow, green, blue, black, red, orange, purple, fullBlue;
	int lineThickness;
	Visualiser() : white(255,255,255), pink(128,128,255), yellow(0,255,255), green(0,128,0), blue(255,128,128), black(0, 0, 0),
			red(0,0,255), orange(0,128,255), purple(128,0,128), fullBlue(255,0,0),
			lineThickness(FIELD_LINE_THICKNESS/FIELD_RESOLUTION)
	{

		if (lineThickness < 1)
			lineThickness = 1;
	}

	void callbackOdom(const nav_msgs::OdometryConstPtr& odom) {
//		ROS_ERROR("Odom received.");
		latestOdom = odom;
	}

	void init() {
		ros::NodeHandle nh;
		imagePub = nh.advertise<sensor_msgs::Image>("image", 1);
		odomSub = nh.subscribe("vo", 1, &Visualiser::callbackOdom, this);

		ros::NodeHandle priv("~");
		priv.param< std::string >("base_frame", baseFrame, DEFAULT_BASE_FRAME);
		priv.param< std::string >("map_frame", fieldFrame, DEFAULT_FIELD_FRAME);
	}


	cv::Scalar lookupColour(const std::string& name) {
		if (strcasecmp(name.c_str(), "white") == 0) {
			return white;
		} else if (strcasecmp(name.c_str(), "pink") == 0) {
			return pink;
		} else if (strcasecmp(name.c_str(), "yellow") == 0) {
			return yellow;
		} else if (strcasecmp(name.c_str(), "blue") == 0) {
			return blue;
		} else if (strcasecmp(name.c_str(), "green") == 0) {
			return green;
		} else if (strcasecmp(name.c_str(), "black") == 0) {
			return black;
		} else if (strcasecmp(name.c_str(), "red") == 0) {
			return red;
		} else if (strcasecmp(name.c_str(), "orange") == 0) {
			return orange;
		} else if (strcasecmp(name.c_str(), "purple") == 0) {
			return purple;
		}

		return cv::Scalar(0,0,0);
	}

	void update() {
		sensor_msgs::ImagePtr imagePtr(new sensor_msgs::Image);

		imagePtr->encoding = sensor_msgs::image_encodings::BGR8;
		imagePtr->height = (FIELD_LENGTH + 2 * FIELD_BORDER) / FIELD_RESOLUTION;
		imagePtr->width = (FIELD_WIDTH + 2 * FIELD_BORDER) / FIELD_RESOLUTION;

		imagePtr->is_bigendian = false;
		imagePtr->step = imagePtr->width * 3;
		imagePtr->step = imagePtr->step + ((4 - (imagePtr->step % 4)) % 4);

		imagePtr->data.resize(imagePtr->step * imagePtr->height);

		for (uint32_t y = 0; y < imagePtr->height; ++y) {
			for (uint32_t x = 0; x < imagePtr->width; ++x) {
				uint8_t *pxl = &imagePtr->data[imagePtr->step * y + 3 * x];
				pxl[0] = 0;
				pxl[1] = 255;
				pxl[2] = 0;
			}
		}

		cv_bridge::CvImageConstPtr cvImageConst = (cv_bridge::toCvShare(imagePtr));
		cv::Mat *cvMat = (cv::Mat *)(&cvImageConst->image);

		DRAW_FIELD_LINE(FIELD_WIDTH/2, FIELD_LENGTH/2, FIELD_WIDTH/2, -FIELD_LENGTH/2);
		DRAW_FIELD_LINE(FIELD_WIDTH/2, -FIELD_LENGTH/2, -FIELD_WIDTH/2, -FIELD_LENGTH/2);
		DRAW_FIELD_LINE(-FIELD_WIDTH/2, -FIELD_LENGTH/2, -FIELD_WIDTH/2, FIELD_LENGTH/2);
		DRAW_FIELD_LINE(-FIELD_WIDTH/2, FIELD_LENGTH/2, FIELD_WIDTH/2, FIELD_LENGTH/2);
		DRAW_FIELD_LINE(-FIELD_WIDTH/2, 0, FIELD_WIDTH/2, 0);

		DRAW_FIELD_LINE(FIELD_BOX_WIDTH/2, FIELD_LENGTH/2 - FIELD_BOX_LENGTH, -FIELD_BOX_WIDTH/2, FIELD_LENGTH/2 - FIELD_BOX_LENGTH);
		DRAW_FIELD_LINE(FIELD_BOX_WIDTH/2, FIELD_LENGTH/2 - FIELD_BOX_LENGTH, FIELD_BOX_WIDTH/2, FIELD_LENGTH/2);
		DRAW_FIELD_LINE(-FIELD_BOX_WIDTH/2, FIELD_LENGTH/2 - FIELD_BOX_LENGTH, -FIELD_BOX_WIDTH/2, FIELD_LENGTH/2);

		DRAW_FIELD_LINE(FIELD_BOX_WIDTH/2, -(FIELD_LENGTH/2 - FIELD_BOX_LENGTH), -FIELD_BOX_WIDTH/2, -(FIELD_LENGTH/2 - FIELD_BOX_LENGTH));
		DRAW_FIELD_LINE(FIELD_BOX_WIDTH/2, -(FIELD_LENGTH/2 - FIELD_BOX_LENGTH), FIELD_BOX_WIDTH/2, -FIELD_LENGTH/2);
		DRAW_FIELD_LINE(-FIELD_BOX_WIDTH/2, -(FIELD_LENGTH/2 - FIELD_BOX_LENGTH), -FIELD_BOX_WIDTH/2, -FIELD_LENGTH/2);

		cv::ellipse(*cvMat, cv::Point(IMAGE_X(0), IMAGE_Y(0)),
				cv::Size(FIELD_CIRCLE_RADIUS/FIELD_RESOLUTION, FIELD_CIRCLE_RADIUS/FIELD_RESOLUTION),
				0, 0, 360, white, lineThickness);

		DRAW_FIELD_LINE(0, -FIELD_CROSS_LENGTH/2, 0, FIELD_CROSS_LENGTH/2);
		DRAW_FIELD_LINE(0, FIELD_SPOT_LENGTH + FIELD_CROSS_LENGTH/2, 0, FIELD_SPOT_LENGTH - FIELD_CROSS_LENGTH/2);
		DRAW_FIELD_LINE(-FIELD_CROSS_LENGTH/2, FIELD_SPOT_LENGTH, FIELD_CROSS_LENGTH/2, FIELD_SPOT_LENGTH);
		DRAW_FIELD_LINE(0, -FIELD_SPOT_LENGTH + FIELD_CROSS_LENGTH/2, 0, -FIELD_SPOT_LENGTH - FIELD_CROSS_LENGTH/2);
		DRAW_FIELD_LINE(-FIELD_CROSS_LENGTH/2, -FIELD_SPOT_LENGTH, FIELD_CROSS_LENGTH/2, -FIELD_SPOT_LENGTH);

		for (size_t i = 0; i < beacons.beacons.size(); ++i) {
			Beacon& b = beacons.beacons[i];
			if (!std::isfinite(b.position.x) || !std::isfinite(b.position.y))
				continue;

			drawBeacon(*cvMat, b.position, lookupColour(b.top), lookupColour(b.bottom));
		}

		for (size_t i = 1; i < path.points.size(); ++i) {
			const geometry_msgs::Point& p1 = path.points[i-1],
					p2 = path.points[i];
			DRAW_LINE(p1.x, p1.y, p2.x, p2.y, orange, 1);
		}
		for (size_t i = 0; i < path.points.size(); ++i) {
			const geometry_msgs::Point& p = path.points[i];

			std::vector< cv::Point > turtle;
			cv::ellipse2Poly(cv::Point(IMAGE_X(p.x), IMAGE_Y(p.y)),
					cv::Size(3, 3),
					0, 0, 360, 60, turtle);
			cv::fillConvexPoly(*cvMat, &turtle[0], turtle.size(), red);
		}

		try {
			tf::StampedTransform transform;

//			ROS_ERROR("Getting transform.");
			tfListener.lookupTransform(fieldFrame, baseFrame, ros::Time(0), transform);
//			ROS_ERROR("Got transform.");
			tf::Vector3 origin = transform.getOrigin();
			history.push_back(origin);

			for (size_t i = 1; i < history.size(); ++i) {
				const tf::Vector3& p1 = history[i-1],
						p2 = history[i];
				DRAW_LINE(p1.x(), p1.y(), p2.x(), p2.y(), purple, 1);
			}


			std::vector< cv::Point > turtle;
			cv::ellipse2Poly(cv::Point(IMAGE_X(origin.x()), IMAGE_Y(origin.y())),
					cv::Size(TURTLEBOT_RADIUS/FIELD_RESOLUTION, TURTLEBOT_RADIUS/FIELD_RESOLUTION),
					0, 0, 360, 10, turtle);
			cv::fillConvexPoly(*cvMat, &turtle[0], turtle.size(), white);
			const cv::Point *pts = &turtle[0]; int npts = turtle.size();
			cv::polylines(*cvMat, &pts, &npts, 1, true, black, 1);
			double yaw = tf::getYaw(transform.getRotation());
			DRAW_LINE(origin.x(), origin.y(), origin.x() + TURTLEBOT_RADIUS * cos(yaw), origin.y() + TURTLEBOT_RADIUS * sin(yaw), black, 1);

		} catch (tf::TransformException& tfe) {
//			ROS_ERROR("Unable to get robot pose for visualisation. (%s)", tfe.what());
		}

		if (latestOdom != NULL && (ros::Time::now() - latestOdom->header.stamp).toSec() < 5) {
			std::vector< cv::Point > odom;
			cv::ellipse2Poly(cv::Point(IMAGE_X(latestOdom->pose.pose.position.x), IMAGE_Y(latestOdom->pose.pose.position.y)),
					cv::Size(2, 2), 0, 0, 360, 15, odom);
			cv::fillConvexPoly(*cvMat, &odom[0], odom.size(), fullBlue);
		}

		imagePub.publish(imagePtr);
	}
};

}  // namespace comp3431


#define UPDATE_INTERVAL		0.1F
int main(int argc, char **argv) {
	ros::init(argc, argv, "visual");

	ros::NodeHandle nh;
	comp3431::Visualiser visualiser;
	visualiser.init();

	ros::Time lastUpdate = ros::Time::now();
	while (ros::ok()) {
		ros::spinOnce();

		ros::Time now = ros::Time::now();
		if ((now - lastUpdate).toSec() > UPDATE_INTERVAL) {
			visualiser.update();

			lastUpdate += ros::Duration(UPDATE_INTERVAL);

			if ((lastUpdate + ros::Duration(UPDATE_INTERVAL)) < now) {
				lastUpdate = now;
			}
		}

		usleep(10000);
	}

	return 0;
}
