#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  cv::Mat src, proc, out, erosion, dilation, elementRect, elementCross, coloured;


public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("in", 1, &ImageConverter::imageCb, this);
    cv::namedWindow(WINDOW);
    cv::namedWindow("Erosion");
    cv::namedWindow("No noise");
    cv::namedWindow("Coloured");
  }

  ~ImageConverter()
  {
    cv::destroyAllWindows();
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
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
    image_pub_.publish(cv_ptr->toImageMsg()); //needs to be moved
   image_processing();
  }

  void image_processing(){
    cv::cvtColor(src, proc, CV_BGR2HSV);
    cv::blur(proc, proc, cv::Size(3, 3));
    cv::inRange(proc, cv::Scalar(158, 120, 120), cv::Scalar(167, 200, 200), out);
 //   cv::blur(out, out, cv::Size(3, 3));
    //erozja

       //setting dimensions of new images
    erosion.create(proc.rows, proc.cols, proc.type());
    dilation.create(proc.rows, proc.cols, proc.type());

       int numberOfIterations = 5;
       int erosion_size=1; //width of the "brush" for the process
       elementCross = getStructuringElement( cv::MORPH_CROSS, cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                              cv::Point( erosion_size, erosion_size ) );
       elementRect = getStructuringElement( cv::MORPH_RECT, cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                                     cv::Point( erosion_size, erosion_size ) );

       //erozja wlasciwa
       erode(out, erosion, elementCross);
       medianBlur(erosion, erosion, 5);
       dilate(erosion, dilation, elementRect);

//       cv::imshow("Erosion", erosion);
       cv::imshow("No noise", dilation);

 //   cv::imshow(WINDOW, out);
    cv::waitKey(3);

  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
