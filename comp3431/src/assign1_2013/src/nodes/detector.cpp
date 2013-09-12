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
#include <assign1_2013/beacons.h>
#include <assign1_2013/trig.h>

//Radians offset for laser distance detection
#define LASER_MARGIN 0.1
//Width of kinect FOV in rads
#define CAM_WIDTH 0.994
#define HIGH_COV 9999

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  cv::Mat src, proc, out, erosion, dilation, elementRect, elementCross, coloured;

  cv::Mat threshold_output, drawing;
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;

  ros::Subscriber laserScan;
	ros::Publisher vo;
  sensor_msgs::LaserScan scan;
	nav_msgs::Odometry odomMsg;
	trig trig_;

public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("in", 1, &ImageConverter::imageCb, this);
    laserScan = nh_.subscribe("/scan", 1, &ImageConverter::scanCallback, this);
		vo = nh_.advertise<nav_msgs::Odometry>("/vo", 1);
		odomMsg.header.frame_id = "/camera_depth_frame";
		odomMsg.child_frame_id = "/base_frame";						//CHECK THESE!!!!!!!!
		odomMsg.twist.twist.linear.x = odomMsg.twist.twist.linear.y = odomMsg.twist.twist.linear.z = 0;
		odomMsg.twist.twist.angular.x = odomMsg.twist.twist.angular.y = odomMsg.twist.twist.angular.z = 0;
		boost::array<double, 36ul> tmp =  	{{HIGH_COV    , 0, 0, 0, 0, 0,
																				  0, HIGH_COV    , 0, 0, 0, 0,
																				  0, 0,     HIGH_COV, 0, 0, 0,
																				  0, 0, 0,     HIGH_COV, 0, 0,
										  										0, 0, 0, 0,     HIGH_COV, 0,
										  										0, 0, 0, 0, 0,  	HIGH_COV}};
		odomMsg.twist.covariance = tmp;

    cv::namedWindow(WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
		odomMsg.header.stamp = ros::Time::now();
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

    cv::Mat colorOut[4], combOut[2];
    cv::Scalar boxColor[4];
    // Pink Layer
    cv::inRange(proc, cv::Scalar(158, 120, 120), cv::Scalar(168, 200, 200), colorOut[0]);
    boxColor[0] = cv::Scalar(255, 20, 147);
    // Blue Layer
    cv::inRange(proc, cv::Scalar(100, 127, 77), cv::Scalar(110, 192, 179), colorOut[1]);
    boxColor[1] = cv::Scalar(0, 0, 255);
    // Green layer
    cv::inRange(proc, cv::Scalar(75, 127, 35), cv::Scalar(90, 230, 110), colorOut[2]);
    boxColor[2] = cv::Scalar(0, 255, 0);
    // Yellow layer
    cv::inRange(proc, cv::Scalar(22, 178, 102), cv::Scalar(30, 255, 179), colorOut[3]);
    boxColor[3] = cv::Scalar(255, 255, 0);
    cv::String name[4] = {"Pink", "Blue", "Green", "Yellow"};
    /*cv::bitwise_or(colorOut[0], colorOut[1], combOut[0]);
    cv::bitwise_or(colorOut[2], colorOut[3], combOut[1]);
    cv::bitwise_or(combOut[0], combOut[1], threshold_output);*/

    //setting dimensions of new images
    erosion.create(proc.rows, proc.cols, proc.type());
    dilation.create(proc.rows, proc.cols, proc.type());

    std::vector<cv::Rect> pinkRect;
    for (int k=0; k < 4; k++)
    {
      int numberOfIterations = 5;
      int erosion_size=1; //width of the "brush" for the process
      elementCross = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size( 2*erosion_size + 1, 2*erosion_size + 1),
                                               cv::Point(erosion_size, erosion_size));
      elementRect = cv::getStructuringElement(cv::MORPH_RECT, cv::Size( 2*erosion_size + 1, 2*erosion_size + 1 ),
                                              cv::Point(erosion_size, erosion_size));

      //erozja wlasciwa
      cv::erode(colorOut[k], erosion, elementCross);
      cv::medianBlur(erosion, erosion, 5);
      cv::dilate(erosion, colorOut[k], elementRect);

      cv::findContours(colorOut[k], contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
      std::vector<cv::Rect> minRect(contours.size());
      for(int i = 0; i < contours.size(); i++)
      { 
        minRect[i] = cv::boundingRect(cv::Mat(contours[i]));;
        minRect[i].height += 7;

        if (k == 0)
        {
          pinkRect.push_back(minRect[i]);
        }
        else
        {
          for (int a = 0; a < pinkRect.size(); a++)
          {
            if ((pinkRect[a] & minRect[i]).area())
            {
              cv::Point circ;
              if (k == 1)
              {
                if (pinkRect[a].y > minRect[i].y)
                  circ = cv::Point(pinkRect[a].x, pinkRect[a].y);
                else
                  circ = cv::Point(minRect[i].x + minRect[i].width, minRect[i].y);
              }
              else if (k == 2)
              {
                if (pinkRect[a].y > minRect[i].y)
                  circ = cv::Point(pinkRect[a].x, pinkRect[a].y);
                else
                  circ = cv::Point(minRect[i].x + minRect[i].width, minRect[i].y);
              }
              else
              {
                if (pinkRect[a].y > minRect[i].y)
                  circ = cv::Point(pinkRect[a].x, pinkRect[a].y);
                else
                  circ = cv::Point(minRect[i].x + minRect[i].width, minRect[i].y);
              }
              cv::circle(colorOut[k], circ, 3, cv::Scalar(255, 0, 0), -1);
            }
          }
        }
      }

      drawing = cv::Mat::zeros(colorOut[k].size(), CV_8UC3);
      for(int i = 0; i < contours.size(); i++ )
      {
        // contour
        //cv::drawContours(drawing, contours, i, contColor, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
        cv::rectangle(colorOut[k], minRect[i], boxColor[k]);
      }
    }

    cv::bitwise_or(colorOut[0], colorOut[1], combOut[0]);
    cv::bitwise_or(colorOut[2], colorOut[3], combOut[1]);
    cv::bitwise_or(combOut[0], combOut[1], threshold_output);

    cv::imshow(WINDOW, threshold_output);
    cv::waitKey(3);
  }


  void scanCallback(const sensor_msgs::LaserScan &laserscan) {
    scan = laserscan;
  }

  // Pass angle in radians?? what is easiest
  //Checks small subset 0.1 radians either side of assumed position of pillar to
  //get distance to pillar (assumed closest)
  double getDist(float angle){
    double increment = scan.angle_increment;
    int pos = angle/increment;
    int offset = LASER_MARGIN/increment;
    double range = 9999;
    for (int i = pos-offset; i <= pos + offset; i++){
      if (i >= 0 && i < scan.ranges.size()){
        if (scan.ranges[i] < range){
          range = scan.ranges[i];
        }
      }
    }
    return range;
  }

	float getAngle(double pos, double imageWidth){
		double halfIm = imageWidth/2;
		float angle = (pos-halfIm)*(CAM_WIDTH/2)/(halfIm);
		return angle;
	}
	
	//Pass in beacon structs and their centre position in the image
  void publish(comp3431::Beacon *left, double centerL, comp3431::Beacon *right, double centerR, double imageWidth){
		odomMsg.header.seq++;
		float aLeft = getAngle(centerL, imageWidth);
		float aRight = getAngle(centerR, imageWidth);
		trig_.getVoPose(&odomMsg.pose, left, getDist(aLeft), aLeft, right, getDist(aRight), aRight);
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
