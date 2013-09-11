#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "geometry_msgs/Polygon.h"
//#include "cvBlobsLib/BlobResult.h"

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber laserScan;
  cv::Mat src, src_hsv, dst;
  sensor_msgs::LaserScan scan;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("in", 1, &ImageConverter::imageCb, this);

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

    src = cv_ptr->image.clone();
    cv::cvtColor(src, src_hsv, CV_BGR2HSV);

    cvInRangeS(src.image, cvScalar(20, 100, 100), cvScalar(30, 255, 255), dst.image);
    //threshold( src_hsv, dst, cvScalar(20, 100, 100), max_BINARY_value,threshold_type );
    

    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    cv::imshow(WINDOW, dst.image);
    cv::waitKey(3);
    
    image_pub_.publish(dst.toImageMsg());
    
  }

  void scanCallback(const sensor_msgs::LaserScan &laserscan) {
    scan = laserscan;
  }

  // Pass angle in radians
  double getDist(float angle){
		float increment = scan.angle_increment; //Get angle between range recorded
		int pos = angle/increment + scan.ranges.size()/2; //Get the offset from angle/increment and add it to the middle
		if (pos >= scan.ranges.size()) {
			return scan.ranges[scan.ranges.size() -1];
		} else if (pos < 0) {
			return scan.ranges[0];
		}
		return scan.ranges[pos];
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
	laserscan = n.subscribe("/scan", 1, &scanCallback);
  ImageConverter ic;
  ros::spin();
  return 0;
}
