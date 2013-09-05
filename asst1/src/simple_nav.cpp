/*
 * Logic:
 * Read in path to take, 
 * Listen to map output from costmap2d or whatever
 * Output geo_msgs/twist for pathing
 * 
 */

#include <ros/ros.h>
#include <assign1_2013/path.h>
#include <cmath>
#include <geometry_msgs/Twist.h>

//Check Numbers
#define POS_THRESHOLD 0.5
#define COV_THRESHOLD 10
#define TURN_SPEED 1
#define FORWARD_SPEED 1
#define SPIN_SEARCH_SPEED 1
#define COLLISION_DISTANCE 0.3

class simple_nav{
private:
    ros::NodeHandle n;
    ros::Publisher geomPub;
    ros::Subscriber posSub;
    std::vector< geometry_msgs::Point > dest;
    boolean collision = false;
    geometry_msgs::Twist stop;

    float facingDest(geometry_msgs::Pose _pose, geometry_msgs::Point _dest){
      float adj = _pose.linear->x - _dest.x;
      float opp = _pose.linear->y - _dest.y;
      float theta = atan(opp/adj);
      return _pose->angular->z - theta;
    }

    float acceptableCov(float64[36] cov){
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
  void posCallback(const nav_msgs::Odometr & msg){
    ROS_INFO("Pos Callback occured");
    if (dest.empty()){
      //Done
      return;
    }

    //Check if we have reached our target
    if (msg->pose->position->x > dest.front()->x - POS_THRESHOLD &&
          msg->pose->position->x < does.front()->x + POS_THRESHOLD) {
      if (msg->pose->position->y > dest.front()->y - POS_THRESHOLD &&
          msg->pose->position->y < does.front()->y + POS_THRESHOLD) {
     
          dest.erase(dest.begin);
          if (dest.emply()){
            return;
          }
      }
    }
    
    //Do we need transforms here... does it matter??

    geometry_msgs::Twist cmd;
    cmd.linear.y = cmd.linear.x = cmd.angular.z = 0.0;
    //Check covariance... if greater than threshold, spin

    if (!acceptableCov(msg->covariance)){
      cmd.angular.z = SPIN_SEARCH_SPEED;
      geomPub.publish(cmd);
      return;
    }

    float diffAngle = facingDest(msg->pose, dest.front());
    if (diffAngle < ANGLE_THRESHOLD && diffAngle > -ANGLE_THRESHOLD){
      //Correct facin g direction
      cmd.linear.x = FORWARD_SPEED;
    } else {
      //Turn to face correct direction     
      cmd.angular.z = TURN_SPEED;
      if (diffAngle < 0)
        cmd.angular.z = -TURN_SPEED;
    }

    geomPub.publish(cmd);

  }

  int main(int argc, char** argv){
    ros::init(argc, argv, "simple_nav");

    comp3431::Path::Path();
    dest = comp3431::Path::points;

    stop.linear.x = stop.linear.y = stop.angular.z = 0.0;

    ros::NodeHandle n;
    geomPub = n.advertise<geometry_msgs::Twist>("unsafe_cmd_vel", 1);
    posSub = n.subscribe("robot_pose_ekf/odom_combined", 1, posCallback);

    ros::spin();
    return 0;
  }
}