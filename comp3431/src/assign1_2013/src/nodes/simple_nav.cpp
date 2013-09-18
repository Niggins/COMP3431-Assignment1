
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
    ros::Subscriber posSub;
    std::vector< geometry_msgs::Point > dest;
    geometry_msgs::Twist cmd;

    float facingDest(geometry_msgs::Pose _pose, geometry_msgs::Point _dest){
      float adj = _pose.position.x - _dest.x;
      float opp = _pose.position.y - _dest.y;
      float theta = atan(opp/adj);
      return _pose.orientation.z - theta;
    }

    bool acceptableCov(const boost::array<double, 36ul> cov){
      //Covariance Matrix input (6*6)
      //Is it within acceptable limits
      //I think it is the following:x,0 y,7 z,14 ax,21 ay,28 az,35
      //Care about 0, 7, 35
      return cov[0] < COV_THRESHOLD && cov[7] < COV_THRESHOLD && cov[35] < COV_THRESHOLD;
    }

public:
  /**
   * Takes in the current position & Covariance
   * Determines whether next destination is and turns towards it
   * and travels forward if nothing is in the way
   */
  void posCallback(const geometry_msgs::PoseWithCovarianceStamped &msg){
    //ROS_INFO("Pos Callback occured");
    if (dest.empty()){
      //Done
      ROS_INFO("Done");
      return;
    }

    //Check if we have reached our target
    if (msg.pose.pose.position.x > dest.front().x - POS_THRESHOLD &&
          msg.pose.pose.position.x < dest.front().x + POS_THRESHOLD) {
      if (msg.pose.pose.position.y > dest.front().y - POS_THRESHOLD &&
          msg.pose.pose.position.y < dest.front().y + POS_THRESHOLD) {
          ROS_INFO("Reached the next destination");   
          dest.erase(dest.begin());
          if (dest.empty()){
            return;
          }
      }
    }
    
    //Do we need transforms here... does it matter??
    //I don't think transforms are required... we want to be in the odom frame 
    // not a frame relative to the base of the robot

    //Check covariance... if greater than threshold, spin
    if (!acceptableCov(msg.pose.covariance)){
      ROS_INFO("High Covariance, trying to stabilise");
      cmd.linear.y = cmd.linear.x = 0.0;
      cmd.angular.z = SPIN_SEARCH_SPEED;
      return;
    }

    float diffAngle = facingDest(msg.pose.pose, dest.front());
    if (diffAngle < ANGLE_THRESHOLD && diffAngle > -ANGLE_THRESHOLD){
      //Correct facin g direction
      //ROS_INFO("Moving forward towards target");
      cmd.linear.x = FORWARD_SPEED;
      cmd.linear.y = cmd.angular.z = 0.0;
    } else {
      //Turn to face correct direction     
      //ROS_INFO("Turning towards next target");
      cmd.linear.x = cmd.linear.y = 0.0;
      cmd.angular.z = TURN_SPEED;
      if (diffAngle < 0)
        cmd.angular.z = -TURN_SPEED;
    }
  }

  void init(){
    comp3431::Path path;
    dest = path.points;
    ros::NodeHandle nh;
    geomPub = nh.advertise<geometry_msgs::Twist>("unsafe_cmd_vel", 1);
    posSub = nh.subscribe("robot_pose_ekf/odom_combined", 1, &simple_nav::posCallback, this);
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.angular.z = TURN_SPEED;
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
