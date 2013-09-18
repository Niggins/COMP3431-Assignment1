#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <sensor_msgs/Image.h>

static const char WINDOW_NAME[] = "Depth View";

class Distance {
private:
	double min_range_;
	double max_range_;
	cv_bridge::CvImagePtr bridge;

public:
	Distance(min_range, max_range): min_range_(min_range_), max_range_(max_range) {}

	void getDistance(const sensor_msgs::ImageConstPtr& image)
	{
		// convert to cv image
		try
		{
		 bridge = cv_bridge::toCvCopy(image, "32FC1");
		}
		catch (cv_bridge::Exception& e)
		{
		 ROS_ERROR("Failed to transform depth image.");
		 return;
		}

		// convert to something visible
		cv::Mat img(bridge->image.rows, bridge->image.cols, CV_8UC1);
		for(int i = 0; i < bridge->image.rows; i++)
		{
		 float* Di = bridge->image.ptr<float>(i);
		 char* Ii = img.ptr<char>(i);
		 for(int j = 0; j < bridge->image.cols; j++)
		 {   
		     Ii[j] = (char) (255*((Di[j]-min_range_)/(max_range_-min_range_)));
		 }   
	 }

 // display
 cv::imshow(WINDOW_NAME, img);
 cv::waitKey(3);
}

/*
int main( int argc, char* argv[] )
{
ros::init( argc, argv, "depth_viewer" );
ros::NodeHandle n;
ros::NodeHandle nh("~");

nh.param("min_range", min_range_, 0.5);
nh.param("max_range", max_range_, 5.5);

cv::namedWindow(WINDOW_NAME);

ros::Subscriber sub = n.subscribe("/camera/depth/image_raw", 3, &depthCb);
ros::spin();
}*/