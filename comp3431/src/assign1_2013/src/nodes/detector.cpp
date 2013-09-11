#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <vector>
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
      src = cv_ptr->image.clone();

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::cvtColor(src, proc, CV_BGR2HSV);
    cv::blur(proc, proc, cv::Size(4, 4));

    cv::Mat pinkOut, blueOut, greenOut, yellowOut, pbOut, gyOut;
    cv::inRange(proc, cv::Scalar(158, 120, 120), cv::Scalar(168, 200, 200), pinkOut);
    cv::inRange(proc, cv::Scalar(100, 127, 77), cv::Scalar(110, 192, 179), blueOut);
    cv::inRange(proc, cv::Scalar(75, 127, 35), cv::Scalar(90, 230, 110), greenOut);
    cv::inRange(proc, cv::Scalar(23, 120, 120), cv::Scalar(30, 170, 200), yellowOut);
    cv::bitwise_or(pinkOut, blueOut, pbOut);
    cv::bitwise_or(greenOut, yellowOut, gyOut);
    cv::bitwise_or(pbOut, gyOut, threshold_output);
    threshold_output = blueOut;

    //setting dimensions of new images
    erosion.create(proc.rows, proc.cols, proc.type());
    dilation.create(proc.rows, proc.cols, proc.type());

    int numberOfIterations = 5;
    int erosion_size=1; //width of the "brush" for the process
    elementCross = cv::getStructuringElement( cv::MORPH_CROSS, cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                        cv::Point( erosion_size, erosion_size ) );
    elementRect = cv::getStructuringElement( cv::MORPH_RECT, cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                               cv::Point( erosion_size, erosion_size ) );

    //erozja wlasciwa
    cv::erode(threshold_output, erosion, elementCross);
    cv::medianBlur(erosion, erosion, 5);
    cv::dilate(erosion, threshold_output, elementRect);



    cv::findContours(threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    std::vector<cv::RotatedRect> minRect(contours.size());
    for(int i = 0; i < contours.size(); i++)
    { 
      minRect[i] = cv::minAreaRect(cv::Mat(contours[i]));
    }

    cv::Scalar contColor = cv::Scalar(255, 0, 0);
    cv::Scalar recColor = cv::Scalar(0, 0, 255);
    cv::Scalar becColor = cv::Scalar(0, 255, 0);
    drawing = cv::Mat::zeros( threshold_output.size(), CV_8UC3 );
    for(int i = 0; i< contours.size(); i++ )
    {
      // contour
      cv::drawContours(drawing, contours, i, contColor, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
      // rotated rectangle
      cv::Point2f rect_points[4]; minRect[i].points(rect_points);
      for(int j = 0; j < 4; j++ )
        cv::line(drawing, rect_points[j], rect_points[(j+1)%4], recColor, 1, 8 );
    }

    cv::imshow(WINDOW, drawing);
    cv::waitKey(3);

    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
