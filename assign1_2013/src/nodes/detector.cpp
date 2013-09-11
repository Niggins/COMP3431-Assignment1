#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <sensor_msgs/LaserScan.h>

//Radians offset for laser distance detection
#define LASER_MARGIN 0.1

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber laserScan;
  sensor_msgs::LaserScan scan;
public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("in", 1, &ImageConverter::imageCb, this);
    laserScan = nh_.subscribe("/scan", 1, &scanCallback);

    cv::namedWindow(WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    IplImage cvImage = (IplImage) cv_ptr->image;
    detect_beacons(&cvImage);

    cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);

    image_pub_.publish(cv_ptr->toImageMsg());
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
      if (i >= && i < scan.ranges.size()){
        if (scan.ranges[i] < range){
          range = scan.ranges[i];
        }
      }
    }
    return range;
  }
private:
	void detect_beacons(IplImage *cvImage){
		//color  detection
		IplImage *frame = 0;
		IplImage* imgThresh = 0;
		frame = cvImage;
		cvSmooth(frame, frame, CV_GAUSSIAN,3,3);

		IplImage* imgHSV = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
		cvCvtColor(frame, imgHSV, CV_BGR2HSV);

		imgThresh=cvCreateImage(cvGetSize(imgHSV),IPL_DEPTH_8U, 1);
		cvInRangeS(imgHSV, cvScalar(212,160,60), cvScalar(227,255,255), imgThresh);
		cvSmooth(imgThresh, imgThresh, CV_GAUSSIAN,3,3);
		cvShowImage("Found beacons", imgThresh);

	}

};

int main(int argc, char** argv)
{
  cvNamedWindow("Found beacons");
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  cvDestroyWindow("Found beacons");
  return 0;
}
