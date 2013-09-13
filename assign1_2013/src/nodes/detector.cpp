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

//static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  cv::Mat src, proc, out, erosion, dilation, elementRect, elementCross, coloured;
  cv::Mat pinkOut, blueOut, greenOut, yellowOut, pbOut, gyOut;
  cv::Scalar pink, blue, green, yellow;
  ros::Subscriber laserScan;
  sensor_msgs::LaserScan scan;

  cv::Mat threshold_output, drawing;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("in", 1, &ImageConverter::imageCb, this);
		laserScan = nh_.subscribe("/scan", 1, &ImageConverter::scanCallback, this);
    pink.val[0] = 180;
    pink.val[1] = 64;
    pink.val[2] = 255;
    blue.val[0] = 255;
    blue.val[1] = 208;
    blue.val[2] = 64;
    green.val[0] = 104;
    green.val[1] = 191;
    green.val[2] = 26;
    yellow.val[0] = 0;
    yellow.val[1] = 240;
    yellow.val[2] = 255;

//    cv::namedWindow("WINDOW");
//    cv::namedWindow("Erosion");
//    cv::namedWindow("No noise");
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
    cv::inRange(proc, cv::Scalar(158, 120, 120), cv::Scalar(168, 200, 200), pinkOut);
    cv::inRange(proc, cv::Scalar(100, 127, 77), cv::Scalar(110, 192, 179), blueOut);
    cv::inRange(proc, cv::Scalar(75, 127, 35), cv::Scalar(90, 230, 110), greenOut);
    cv::inRange(proc, cv::Scalar(23, 120, 120), cv::Scalar(30, 170, 200), yellowOut);

    //erozja

    //setting dimensions of new images
     erosion.create(proc.rows, proc.cols, proc.type());
     dilation.create(proc.rows, proc.cols, proc.type());
     coloured.create(proc.rows, proc.cols, proc.type());
    coloured.setTo(cv::Scalar(0,0,0));
  //  white.create(proc.rows, proc.cols, proc.type());

       int numberOfIterations = 5;
       int erosion_size=1; //width of the "brush" for the process
       elementCross = getStructuringElement( cv::MORPH_CROSS, cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                              cv::Point( erosion_size, erosion_size ) );
       elementRect = getStructuringElement( cv::MORPH_RECT, cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                                     cv::Point( erosion_size, erosion_size ) );

       //filter and colouring
       filter(&pinkOut);
       colour(&pink);
       filter(&blueOut);
       colour(&blue);
       filter(&greenOut);
       colour(&green);
       filter(&yellowOut);
       colour(&yellow);

       cv::bitwise_or(pinkOut, blueOut, pbOut);
       cv::bitwise_or(greenOut, yellowOut, gyOut);
       cv::bitwise_or(pbOut, gyOut, threshold_output);
       threshold_output = blueOut;



/*


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
             cv::drawContours(coloured, contours, i, contColor, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
             // rotated rectangle
             cv::Point2f rect_points[4]; minRect[i].points(rect_points);
             for(int j = 0; j < 4; j++ )
               cv::line(coloured, rect_points[j], rect_points[(j+1)%4], recColor, 1, 8 );
           }
*/
//       cv::imshow("WINDOW", drawing);


//       cv::imshow("Erosion", erosion);
//       cv::imshow("No noise", dilation);
       cv::imshow("Coloured", coloured);


 //   cv::imshow(WINDOW, out);
    cv::waitKey(3);

  }

  void scanCallback(const sensor_msgs::LaserScan &laserscan) {
    scan = laserscan;
  }

  // Pass angle in radians?? what is easiest
  //Checks small subset 0.1 radians either side of assumed position of pillar to
  //get distance to pillar (assumed closest)
  /*double getDist(float angle){
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
  }*/

private:
  void filter(cv::Mat *colour){
	  erode(*colour, erosion, elementCross);
	  medianBlur(erosion, erosion, 5);
	  dilate(erosion, dilation, elementRect);
  }

  void colour(cv::Scalar *colour){
	  for (int x=0; x<dilation.cols; x++){
	      	   for (int y=0; y<dilation.rows; y++){
	      		   if (dilation.at<unsigned char>(y, x)==255){
	      	//	   if(dilation.at<cv::Vec3b>(cv::Point(x, y))[0] == 255 && dilation.at<cv::Vec3b>(cv::Point(x, y))[1] == 255 && dilation.at<cv::Vec3b>(cv::Point(x, y))[2] == 255){
	      			   coloured.at<cv::Vec3b>(cv::Point(x, y))[0] = colour->val[0];
	      			   coloured.at<cv::Vec3b>(cv::Point(x, y))[1] = colour->val[1];
	      			   coloured.at<cv::Vec3b>(cv::Point(x, y))[2] = colour->val[2];
	      		   }
	      	   }
	  }
	  erosion.setTo(cv::Scalar(0,0,0));
	  dilation.setTo(cv::Scalar(0,0,0));
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
