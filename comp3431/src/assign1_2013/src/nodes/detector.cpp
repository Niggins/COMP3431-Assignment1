#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <string>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <assign1_2013/beacons.h>
#include <assign1_2013/trig.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

//Radians offset for laser distance detection
#define LASER_MARGIN 0.1
//Width of kinect FOV in rads
#define CAM_WIDTH 0.994
#define HIGH_COV 9999
#define FOCAL_LENGTH 525
#define IMAGE_WIDTH 640
#define COLUMN_COLOR_HEIGHT 0.1

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  cv::Mat src, proc, out, erosion, dilation, elementRect, elementCross;

  cv::Mat threshold_output;
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;

  ros::Subscriber laserScan;
	ros::Publisher vo;
  sensor_msgs::LaserScan scan;
	nav_msgs::Odometry odomMsg;
	trig trig_;
	comp3431::Beacons beaconList;
	std::string beaconColors[4];	
  geometry_msgs::PoseWithCovariance prevPoint;
  ros::Subscriber odomSub;
  ros::Subscriber kalmanSub;
  geometry_msgs::PoseWithCovariance betterPose;

public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("in", 1, &ImageConverter::imageCb, this);
    laserScan = nh_.subscribe("/scan", 1, &ImageConverter::scanCallback, this);
    odomSub = nh_.subscribe("robot_pose_ekf/odom_combined", 1, &ImageConverter::odomCallback, this);
    kalmanSub = nh_.subscribe("kalman_output", 1, &ImageConverter::kalmanCB, this);
    odomMsg.child_frame_id = "base_footprint";
    odomMsg.header.frame_id = "odom_combined";
		vo = nh_.advertise<nav_msgs::Odometry>("/vo", 1);
		odomMsg.twist.twist.linear.x = odomMsg.twist.twist.linear.y = odomMsg.twist.twist.linear.z = 0;
		odomMsg.twist.twist.angular.x = odomMsg.twist.twist.angular.y = odomMsg.twist.twist.angular.z = 0;
		boost::array<double, 36ul> tmp =  	{{HIGH_COV    , 0, 0, 0, 0, 0,
																				  0, HIGH_COV    , 0, 0, 0, 0,
																				  0, 0,     HIGH_COV, 0, 0, 0,
																				  0, 0, 0,     HIGH_COV, 0, 0,
										  										0, 0, 0, 0,     HIGH_COV, 0,
										  										0, 0, 0, 0, 0,  	HIGH_COV}};
		odomMsg.twist.covariance = tmp;
    beaconColors[0] = "pink";
    beaconColors[1] = "blue";
    beaconColors[2] = "green";
    beaconColors[3] = "yellow";
    
    cv::namedWindow(WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }

  void kalmanCB(const geometry_msgs::PoseWithCovariance& ret) {
    betterPose.pose.position.x = ret.pose.position.x;
    betterPose.pose.position.y = ret.pose.position.y;
    betterPose.pose.orientation.z = ret.pose.orientation.z;
    
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
		odomMsg.header.stamp = ros::Time::now();
		long imageWidth = msg->width;
		std::vector< SpottedBeacon > spottedBeacons;
		bool pinkTop = false;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
      src = cv_ptr->image.clone();

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::cvtColor(src, proc, CV_BGR2HSV);
    cv::GaussianBlur(proc, proc, cv::Size(3, 3), 0, 0);

    cv::Mat filteredLayer[4], combOut[2], coloredLayer[4];
    cv::Scalar boxColor[4];
    // Pink layer filter
    cv::inRange(proc, cv::Scalar(158, 120, 120), cv::Scalar(168, 200, 200), filteredLayer[0]);
    boxColor[0] = cv::Scalar(127, 0, 255);
    // Blue layer filter
    cv::inRange(proc, cv::Scalar(100, 127, 77), cv::Scalar(110, 192, 179), filteredLayer[1]);
    boxColor[1] = cv::Scalar(255, 0, 0);
    // Green layer filter
    cv::inRange(proc, cv::Scalar(75, 127, 35), cv::Scalar(90, 230, 110), filteredLayer[2]);
    boxColor[2] = cv::Scalar(0, 255, 0);
    // Yellow layer filter
    cv::inRange(proc, cv::Scalar(22, 178, 102), cv::Scalar(30, 255, 179), filteredLayer[3]);
    boxColor[3] = cv::Scalar(0, 255, 255);

    // Filtering stuff
    erosion.create(proc.rows, proc.cols, proc.type());
    dilation.create(proc.rows, proc.cols, proc.type());

    std::vector<cv::Rect> pinkRect;
    // Iterate through the different color layers
    for (int k=0; k < 4; k++)
    {
      // Filter out noise
      int numberOfIterations = 5;
      int erosion_size=1; //width of the "brush" for the process
      elementCross = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size( 2*erosion_size + 1, 2*erosion_size + 1),
                                               cv::Point(erosion_size, erosion_size));
      elementRect = cv::getStructuringElement(cv::MORPH_RECT, cv::Size( 2*erosion_size + 1, 2*erosion_size + 1 ),
                                              cv::Point(erosion_size, erosion_size));
      cv::erode(filteredLayer[k], erosion, elementCross);
      cv::medianBlur(erosion, erosion, 5);
      cv::dilate(erosion, filteredLayer[k], elementRect);

      // Find the contours of the current color layer
      cv::findContours(filteredLayer[k], contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
      std::vector<cv::Rect> minRect(contours.size());
      coloredLayer[k] = cv::Mat::zeros(filteredLayer[k].size(), CV_8UC3);
      for(int i = 0; i < contours.size(); i++)
      { 
        // Create rectangles containing the counours of the current color
        minRect[i] = cv::boundingRect(cv::Mat(contours[i]));;
        minRect[i].height += 7;

        // Store all the pink rectangles
        if (k == 0)
        {
          pinkRect.push_back(minRect[i]);
        }

        // If the current color is not pink, iterate through the pink rectangles
        else
        {
          for (int a = 0; a < pinkRect.size(); a++) {
            // Checks if a pink rectangle intersects the current colored rectangle
            if ((pinkRect[a] & minRect[i]).area())
            {
              cv::Point center;
              SpottedBeacon spotted;
              int centerX = ((pinkRect[a].x + pinkRect[a].width/2)+(minRect[i].x + minRect[i].width/2))/2;
              // Checks if pink rectangle is on top
              if (pinkRect[a].y < minRect[i].y)
              {
                center = calcRectMid(pinkRect[a], minRect[i]);               
                spotted.beacon = beaconList.getBeacon(beaconColors[0], beaconColors[k]);
              }
              // Otherwise pink rectangle is on the bottom
              else
              {
                center = calcRectMid(minRect[i], pinkRect[a]);
                spotted.beacon = beaconList.getBeacon(beaconColors[k], beaconColors[0]);
              }
              // Draw a red dot at the center point of the 2 rectangles
              cv::circle(coloredLayer[k], center, 3, cv::Scalar(0, 0, 255), -1);
							//Add to spotted beacon list
							if (centerX >= 0 && centerX < IMAGE_WIDTH) {
                spotted.angle = getAngle(centerX);
                spotted.distance = getDist(pinkRect[a].y+pinkRect[a].height-7, pinkRect[a].y, spotted.angle);
							  ROS_INFO("Spotted Beacon %s:%s", spotted.beacon->top.c_str(), spotted.beacon->bottom.c_str());
							  spottedBeacons.push_back(spotted);
							} else {
							  ROS_INFO("Centerx %d", centerX);
							}
            }
          }
        }
      }

			publish(spottedBeacons, imageWidth);

      // Draw bounding rectangles the same color as the color contour
      for(int i = 0; i < contours.size(); i++ )
      {
        cv::rectangle(coloredLayer[k], minRect[i], boxColor[k]);
      }
    }

    cv::bitwise_or(coloredLayer[0], coloredLayer[1], combOut[0]);
    cv::bitwise_or(coloredLayer[2], coloredLayer[3], combOut[1]);
    cv::bitwise_or(combOut[0], combOut[1], threshold_output);

    cv::imshow(WINDOW, threshold_output);
    cv::waitKey(3);
  }

  cv::Point calcRectMid(cv::Rect const& topRect, cv::Rect const& botRect)
  {
    float mid, topMid, botMid;
    topMid = topRect.x + topRect.width/2;
    botMid = botRect.x + botRect.width/2;
    mid = (topMid + botMid)/2;
    return cv::Point(mid, botRect.y);
  }

  void scanCallback(const sensor_msgs::LaserScan &laserscan) {
    scan = laserscan;
    //ROS_INFO("LaserScan Reading##########################################################");
  }

  // Pass angle in radians?? what is easiest
  //Checks small subset 0.1 radians either side of assumed position of pillar to
  //get distance to pillar (assumed closest)
  double getDist(double top, double bottom, float angle) {

    double range = COLUMN_COLOR_HEIGHT*FOCAL_LENGTH/(top-bottom);

/*    double increment = scan.angle_increment;
    int pos = angle/increment;
    int offset = LASER_MARGIN/increment;
    double range = 0.0;
    for (int i = pos-offset; i <= pos + offset; i++){
      if (i >= 0 && i < scan.ranges.size()){
        //ROS_INFO("Distance Scan range %f", scan.ranges[i]);
        if ((scan.ranges[i] < range || range == 0) && scan.ranges[i] > scan.range_min){
          range = scan.ranges[i];
        }
      }
    }*/
    ROS_INFO("Distance %f t %f b %f", range, top, bottom);
    return range;
  }

	float getAngle(long pos){
		double halfIm = IMAGE_WIDTH/2;
		float angle = (pos-halfIm)*(CAM_WIDTH/2)/(halfIm);
    ROS_INFO("Beacon angle A:%f pos:%ld imWidth:%d", angle, pos, IMAGE_WIDTH);
		return angle;
	}

  void odomCallback(const geometry_msgs::PoseWithCovarianceStamped &msg){
    prevPoint = msg.pose;
  }
	//Pass in beacon structs and their centre position in the image
  void publish(std::vector <SpottedBeacon> spottedBeacons, long imageWidth){
		odomMsg.header.seq++;
		SpottedBeacon left;
		SpottedBeacon right;
		for (std::vector<SpottedBeacon>::iterator it = spottedBeacons.begin(); it != spottedBeacons.end(); it++){
			//distance is temporarily used to describe distance from edge of image
//			it->angle = getAngle(it->distance, imageWidth);
//			it->distance = getDist(it->angle);
      if (it->distance == 0)
        continue;
			if (left.beacon == NULL) {
				left.set(*it);
			} 
      else {
				if (left.distance < it->distance) {
					right.set(*it);
				} else {
					right.set(left);
					left.set(*it);
				}
			}
		}
		trig_.getVoPose(&odomMsg.pose, left, right, betterPose);
    //ROS_INFO("Left Beacon %s:%s", left.beacon->top.c_str(), left.beacon->bottom.c_str());
		ROS_INFO("Pose x: %f, y:%f z:%f \n", odomMsg.pose.pose.position.x, odomMsg.pose.pose.position.y, odomMsg.pose.pose.orientation.z);
		vo.publish(odomMsg);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
