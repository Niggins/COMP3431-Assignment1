#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#define CHECKNAN(x,y) if (y == y) x = y

using namespace cv;
using namespace std;
using namespace ros;


class RosKalmanFilter {
private:
	NodeHandle nh_;
	Subscriber voSub, odomSub;
	Publisher odomCombPub;
    KalmanFilter KF;
    Mat_<float> state; /* (x, y, theta) */
    Mat processNoise;
    Mat_<float> measurement;
    geometry_msgs::Pose odomPoint, voPoint;

public:
	RosKalmanFilter() {
		KF = KalmanFilter (3, 2, 0);
	    state = Mat_<float> (3, 1);
	    processNoise = Mat (3, 1, CV_32F);
	    measurement = Mat_<float> (2,3); 

		voSub = nh_.subscribe("vo", 1, &RosKalmanFilter::voCallback, this);
		odomSub = nh_.subscribe("odom", 1, &RosKalmanFilter::odomCallback, this);
		odomCombPub = nh_.advertise<geometry_msgs::PoseWithCovariance>("kalman_output", 1);
		measurement.setTo(Scalar(0));
		KF.statePre.at<float>(0) = 0;
		KF.statePre.at<float>(1) = 0;
		KF.statePre.at<float>(2) = 0;
		KF.transitionMatrix = *(Mat_<float>(3, 3) << 1,0,0,   0,1,0,  0,0,1);
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(KF.errorCovPost, Scalar::all(.1));
	}

	//~KalmanFilter() {}

	void odomCallback(const nav_msgs::Odometry &msg) {
		odomPoint = getPose(msg);
	}

		
    void voCallback(const nav_msgs::Odometry &msg)
    {
	   	voPoint = getPose(msg);
		
      Mat prediction = KF.predict();
      Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
      float PredictTheta(prediction.at<float>(2));
		
      measurement = (Mat_<float>(2, 3) << voPoint.position.x, voPoint.position.y, voPoint.orientation.z,
        	odomPoint.position.x, odomPoint.position.y, odomPoint.orientation.z);
		
		Point measPt(measurement(0), measurement(1));
        // generate measurement
        //measurement += KF.measurementMatrix*state;

		Mat estimated = KF.correct(measurement);
		//Point statePt(estimated.at<float>(0),estimated.at<float>(1));
		//kalmanv.push_back(statePt);			
		
//            randn( processNoise, Scalar(0), Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
//            state = KF.transitionMatrix*state + processNoise;

		publish(measurement);
    }

    void publish(Mat_<float> measurement) {
    	geometry_msgs::PoseWithCovariance ret;
    	ret.pose.position.x = measurement(0);
    	ret.pose.position.y = measurement(1);
    	ret.pose.orientation.z = measurement(2);
    	odomCombPub.publish(ret);
      ROS_INFO("****************************Published Kalman*******************************");
    } 

    geometry_msgs::Pose getPose(const nav_msgs::Odometry &msg){
    //ROS_INFO("****************************SAVED ODOM*******************************");
    //ROS_INFO("Passed Pose x: %f, y: %f, z: %f", msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z);
    geometry_msgs::Pose ret;
    CHECKNAN(ret.position.x, msg.pose.pose.position.x);
    CHECKNAN(ret.position.y, msg.pose.pose.position.y);
    CHECKNAN(ret.position.z, msg.pose.pose.position.x);
    CHECKNAN(ret.orientation.x, msg.pose.pose.orientation.x);
    CHECKNAN(ret.orientation.y, msg.pose.pose.orientation.y);
    CHECKNAN(ret.orientation.z, msg.pose.pose.orientation.z);
    CHECKNAN(ret.orientation.w, msg.pose.pose.orientation.w);
    return ret;
  }
};

int main(int argc, char** argv)
{
  init(argc, argv, "kalman_filter");
  RosKalmanFilter rcf;
  spin();
  return 0;
}