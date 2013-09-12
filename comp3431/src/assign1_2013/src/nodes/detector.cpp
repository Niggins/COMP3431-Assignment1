#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <string>
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
    cv::GaussianBlur(proc, proc, cv::Size(3, 3), 0, 0);

    cv::Mat colorOut[4], combOut[2], coloredLayers[4];
    cv::Scalar boxColor[4];
    // Pink Layer
    cv::inRange(proc, cv::Scalar(158, 120, 120), cv::Scalar(168, 200, 200), colorOut[0]);
    //boxColor[0] = cv::Scalar(160, 230, 255);
    boxColor[0] = cv::Scalar(127, 0, 255);
    // Blue Layer
    cv::inRange(proc, cv::Scalar(100, 127, 77), cv::Scalar(110, 192, 179), colorOut[1]);
    boxColor[1] = cv::Scalar(255, 0, 0);
    // Green layer
    cv::inRange(proc, cv::Scalar(75, 127, 35), cv::Scalar(90, 230, 110), colorOut[2]);
    boxColor[2] = cv::Scalar(0, 255, 0);
    // Yellow layer
    cv::inRange(proc, cv::Scalar(22, 178, 102), cv::Scalar(30, 255, 179), colorOut[3]);
    boxColor[3] = cv::Scalar(0, 255, 255);
    cv::String name[4] = {"Pink", "Blue", "Green", "Yellow"};

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
      coloredLayers[k] = cv::Mat::zeros(colorOut[k].size(), CV_8UC3);
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
                if (pinkRect[a].y < minRect[i].y)
                  circ = calcRectMid(pinkRect[a], minRect[i]);
                else
                  circ = calcRectMid(minRect[i], pinkRect[a]);
              }
              else if (k == 2)
              {
                if (pinkRect[a].y < minRect[i].y)
                  circ = calcRectMid(pinkRect[a], minRect[i]);
                else
                  circ = calcRectMid(minRect[i], pinkRect[a]);
              }
              else
              {
                if (pinkRect[a].y < minRect[i].y)
                  circ = calcRectMid(pinkRect[a], minRect[i]);
                else
                  circ = calcRectMid(minRect[i], pinkRect[a]);
              }
              cv::circle(coloredLayers[k], circ, 3, cv::Scalar(0, 0, 255), -1);
            }
          }
        }
      }

      for(int i = 0; i < contours.size(); i++ )
      {
        // contour
        //cv::drawContours(drawing, contours, i, boxColor[k], 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
        cv::rectangle(coloredLayers[k], minRect[i], boxColor[k]);
      }
      //cv::imshow(name[k], coloredLayers[k]);
    }

    /*cv::bitwise_or(colorOut[0], colorOut[1], combOut[0]);
    cv::bitwise_or(colorOut[2], colorOut[3], combOut[1]);
    cv::bitwise_or(combOut[0], combOut[1], threshold_output);*/

    cv::bitwise_or(coloredLayers[0], coloredLayers[1], combOut[0]);
    cv::bitwise_or(coloredLayers[2], coloredLayers[3], combOut[1]);
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
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
