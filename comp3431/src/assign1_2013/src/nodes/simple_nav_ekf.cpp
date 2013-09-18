
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
#define COV_THRESHOLD 1
#define TURN_SPEED 0.3
#define FORWARD_SPEED 0.3
#define SPIN_SEARCH_SPEED 0.2
#define COLLISION_DISTANCE 0.3
#define ANGLE_THRESHOLD 0.1
#define MAX_ODOM_USE 50

//a nan value != itself
#define CHECKNAN(x,y) if (y == y) x = y

class simple_nav{
private:
    ros::NodeHandle n;
    ros::Publisher geomPub;
    ros::Publisher posPub;
    ros::Subscriber voSub;
    ros::Subscriber odomSub;
    std::vector< geometry_msgs::Point > dest;
    geometry_msgs::Twist cmd;
    geometry_msgs::Pose prev;
    geometry_msgs::Pose lastKnownVo;
    geometry_msgs::Pose savedOdom;
    bool unknownVo;
    int odomUse;

    float facingDest(geometry_msgs::Pose _pose, geometry_msgs::Point _dest){
      float adj = _pose.position.x - _dest.x;
      float opp = _pose.position.y - _dest.y;
      float theta = atan(opp/adj);
      return _pose.orientation.z - theta;
    }

    bool acceptableCov(const boost::array<double, 36ul> cov){
      return cov[0] < COV_THRESHOLD && cov[7] < COV_THRESHOLD && cov[35] < COV_THRESHOLD;
    }


  void setCmd(geometry_msgs::Pose pos, bool spin){
    posPub.publish(pos);
    if (dest.empty()){
      //Done
      ROS_INFO("Done");
      return;
    }

    //Check if we have reached our target
    if (pos.position.x > dest.front().x - POS_THRESHOLD &&
          pos.position.x < dest.front().x + POS_THRESHOLD) {
      if (pos.position.y > dest.front().y - POS_THRESHOLD &&
          pos.position.y < dest.front().y + POS_THRESHOLD) {
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

    float diffAngle = facingDest(pos, dest.front());
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

public:
  void voCallback(const nav_msgs::Odometry &msg){
    ROS_INFO("VO Callback occured");
  /*  if (!tfListener.canTransform("odom_combined", msg.header.frame_id, ros::Time(0))){
      ROS_ERROR("CANNOT GET TRANSFORM NOW");
      return;
    }
    geometry_msgs::Pose temp;
    tfListener.transformPose("odom_combined", msg.header.stamp, msg.pose.pose, "vo", temp);
*/
    if (!acceptableCov(msg.pose.covariance)){
      //Not acceptable covariance
      if (unknownVo){
        return;
      } else {
        unknownVo = true;
        savedOdom.position.x = prev.position.x;
        savedOdom.position.y = prev.position.y;
        savedOdom.orientation.z = prev.orientation.z;
        return;
      }
    }
    unknownVo = false;
    odomUse = 0;
    //Take vo as postion
    CHECKNAN(prev.position.x, msg.pose.pose.position.x);
    CHECKNAN(prev.position.y, msg.pose.pose.position.y);
    CHECKNAN(prev.orientation.z, msg.pose.pose.orientation.z);
    setCmd(prev, false);
  }

  void odomCallback(const nav_msgs::Odometry &msg){
    ROS_INFO("Odom Callback");
    if (unknownVo){
      geometry_msgs::Pose delta;
      delta.position.x = msg.pose.pose.position.x - savedOdom.position.x;
      delta.position.y = msg.pose.pose.position.y - savedOdom.position.y;
      delta.orientation.z = msg.pose.pose.orientation.z - savedOdom.orientation.z;
      ROS_INFO("dx: %f dy: %f dz: %f", delta.position.x, delta.position.y, delta.orientation.z);
      savedOdom.position.x = msg.pose.pose.position.x;
      savedOdom.position.y = msg.pose.pose.position.y;
      savedOdom.orientation.z = msg.pose.pose.orientation.z;

      prev.position.x += delta.position.x;
      prev.position.y += delta.position.y;
      prev.orientation.z += delta.orientation.z;

      odomUse++;
      setCmd(prev, odomUse > MAX_ODOM_USE);

    }

  }

  void init(){
    comp3431::Path path;
    dest = path.points;
    ros::NodeHandle nh;
    geomPub = nh.advertise<geometry_msgs::Twist>("unsafe_cmd_vel", 1);
    posPub = nh.advertise<geometry_msgs::Pose>("position", 1);
    voSub = nh.subscribe("vo", 1, &simple_nav::voCallback, this);
    odomSub = nh.subscribe("odom", 1, &simple_nav::odomCallback, this);
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.angular.z = TURN_SPEED;
    unknownVo = false;
    odomUse = 0;
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
