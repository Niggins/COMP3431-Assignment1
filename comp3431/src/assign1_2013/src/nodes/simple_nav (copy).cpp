
#include <ros/ros.h>
#include <assign1_2013/path.h>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

//Check Numbers
#define POS_THRESHOLD 0.5
#define COV_THRESHOLD 50
#define TURN_SPEED 0.3
#define FORWARD_SPEED 0.3
#define SPIN_SEARCH_SPEED 0.2
#define COLLISION_DISTANCE 0.3
#define ANGLE_THRESHOLD 0.1

class simple_nav{
private:
    ros::NodeHandle n;
    ros::Publisher geomPub;
    ros::Subscriber voSub;
    ros::Subscriber odomSub;
    std::vector< geometry_msgs::Point > dest;
    geometry_msgs::Twist cmd;
    geometry_msgs::Pose prev;
    geometry_msgs::Pose lastKnownVo;
    geometry_msgs::Pose savedOdom;
    bool unknownVo;

    float facingDest(geometry_msgs::Pose _pose, geometry_msgs::Point _dest){
      float adj = _pose.position.x - _dest.x;
      float opp = _pose.position.y - _dest.y;
      float theta = atan(opp/adj);
      return _pose.orientation.z - theta;
    }

    bool acceptableCov(const boost::array<double, 36ul> cov){
      return cov[0] < COV_THRESHOLD && cov[7] < COV_THRESHOLD && cov[35] < COV_THRESHOLD;
    }

public:
  void voCallback(const nav_msgs::Odometry &msg){
    ROS_INFO("VO Callback occured");
    if (!tfListener.canTransform("odom_combined", msg.header.frame_id, ros::Time(0))){
      ROS_ERROR("CANNOT GET TRANSFORM NOW");
      return;
    }
    geometry_msgs::Pose temp;
    tfListener.transformPose("odom_combined", msg.header.stamp, msg.pose.pose, "vo", temp);

    if (!acceptableCov(msg.pose.covariance)){
      //Not acceptable covariance
      if (unknownVo){
        return;
      } else {
        unknownVo = true;
        lastKnownVo = prev;
        return;
      }
    }
    unknownVo = false;


  }

  void odomCallback(const nav_msgs::Odometry &msg){
    ROS_INFO("Odom Callback");
    if (unknownVo){
      if ()
    }

  }

  void init(){
    comp3431::Path path;
    dest = path.points;
    ros::NodeHandle nh;
    geomPub = nh.advertise<geometry_msgs::Twist>("unsafe_cmd_vel", 1);
    voSub = nh.subscribe("vo", 1, &simple_nav::voCallback, this);
    odomSub = nh.subscribe("odom", 1, &simple_nav::odomCallback, this);
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.angular.z = TURN_SPEED;
    unknownVo = false;
  }

  void publish(){
    geomPub.publish(cmd);
  }
};

  int main(int argc, char** argv){
    ros::init(argc, argv, "simple_nav");
    simple_nav nav;
    nav.init();
    ros::Rate loop_rate(3);

    while (ros::ok()){
        nav.publish();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
  }
