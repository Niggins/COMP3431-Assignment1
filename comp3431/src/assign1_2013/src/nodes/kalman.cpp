#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv.h>

using namespace cv;

if (!transformer_.canTransform("base_footprint","vo", filter_time)) {
	ROS_ERROR("filter time older than vo message buffer");
	return false;
}
transformer_.lookupTransform("vo", "base_footprint", filter_time, vo_meas_);
if (vo_initialized_) {
// convert absolute vo measurements to relative vo measurements
	Transform vo_rel_frame =  filter_estimate_old_ * vo_meas_old_.inverse() * vo_meas_;
	ColumnVector vo_rel(6);
	decomposeTransform(vo_rel_frame, vo_rel(1),  vo_rel(2), vo_rel(3), vo_rel(4), vo_rel(5), vo_rel(6));
	angleOverflowCorrect(vo_rel(6), filter_estimate_old_vec_(6));
	// update filter
	vo_meas_pdf_->AdditiveNoiseSigmaSet(vo_covariance_ * pow(dt,2));
	filter_->Update(vo_meas_model_, vo_rel);
}
else vo_initialized_ = true;
vo_meas_old_ = vo_meas_;

ROS_DEBUG("Process odom meas");
if (!transformer_.canTransform("base_footprint","wheelodom", filter_time)){
	ROS_ERROR("filter time older than odom message buffer");
	return false;
}
transformer_.lookupTransform("wheelodom", "base_footprint", filter_time, odom_meas_);
if (odom_initialized_) {
	// convert absolute odom measurements to relative odom measurements in horizontal plane
	Transform odom_rel_frame =  Transform(tf::createQuaternionFromYaw(filter_estimate_old_vec_(6)), 
					      filter_estimate_old_.getOrigin()) * odom_meas_old_.inverse() * odom_meas_;
	ColumnVector odom_rel(6); 
	decomposeTransform(odom_rel_frame, odom_rel(1), odom_rel(2), odom_rel(3), odom_rel(4), odom_rel(5), odom_rel(6));
	angleOverflowCorrect(odom_rel(6), filter_estimate_old_vec_(6));
	// update filter
	odom_meas_pdf_->AdditiveNoiseSigmaSet(odom_covariance_ * pow(dt,2));

    ROS_DEBUG("Update filter with odom measurement %f %f %f %f %f %f", 
              odom_rel(1), odom_rel(2), odom_rel(3), odom_rel(4), odom_rel(5), odom_rel(6));
	filter_->Update(odom_meas_model_, odom_rel);
	diagnostics_odom_rot_rel_ = odom_rel(6);
}
else {
	odom_initialized_ = true;
	diagnostics_odom_rot_rel_ = 0;
  }
odom_meas_old_ = odom_meas_;

KalmanFilter kalman = KalmanFilter(3, 3, 0);
Mat state = Mat(3, 1, CV32_FC1);