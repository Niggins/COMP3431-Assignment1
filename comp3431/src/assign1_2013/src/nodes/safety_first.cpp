#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include <cstdlib>
#include <iostream>
#include <sys/time.h>

#define ROBOT_MAXSPEED		0.2
#define TURTLEBOT_RADIUS	0.16
#define MIN_DISTANCE		0.5
#define PI					3.14159265

double actualDistance = 0;
double laserAngleMin, laserAngleMax, laserAngleIncrement;
double laserCenterDistance = 0;
int leftScanNumber = 0, rightScanNumber = 0, frontScanNumber = 0;

int whichRange = -1;
int turnDirection = 0;
bool isSafetyNeeded = false, normalRun = true;

geometry_msgs::Twist safe_cmdVel; // = NULL; *** Cannot do this
geometry_msgs::Twist original_cmdVel; // = NULL;


void laser_scan(const sensor_msgs::LaserScan & laserscan)
{
	/*
	 * This function checks if an obstacle is present within a distance smaller than
	 * MIN_DISTANCE in 90 degrees both left and right direction.
	 */
	laserAngleMin = laserscan.angle_min;
	laserAngleMax = laserscan.angle_max;
	laserAngleIncrement = laserscan.angle_increment;
	laserCenterDistance = laserscan.ranges[(laserscan.ranges.size() / 2)];
	frontScanNumber = laserscan.ranges.size()/2;
	rightScanNumber = frontScanNumber + PI/(2*laserAngleIncrement);
	leftScanNumber = frontScanNumber - PI/(2*laserAngleIncrement);

	for (int i=frontScanNumber; i<rightScanNumber; i++){
			if (laserscan.ranges[i] < MIN_DISTANCE && laserscan.ranges[i] > laserscan.range_min){
				//ROS_INFO ("right: %f", laserscan.ranges[i]);
				whichRange = i;
				isSafetyNeeded = true;
				turnDirection = -1; //will turn counter-clockwise
			}
		}
	for (int i=frontScanNumber; i>leftScanNumber; i--){
			if (laserscan.ranges[i] < MIN_DISTANCE && laserscan.ranges[i] > laserscan.range_min){
				//ROS_INFO ("left: %f", laserscan.ranges[i]);
				whichRange = i;
				isSafetyNeeded = true;
				turnDirection = 1; //will turn clockwise
			}
		}

	if (whichRange==-1)
		isSafetyNeeded = false;

	whichRange = -1;

	ROS_INFO ("range: %d", laserscan.ranges.size());


}

void original_cmdVel_callBack(const geometry_msgs::Twist & twistMsg){
	original_cmdVel = twistMsg;
}

void avoid_obstacle (){

	//if an obstacle is present, robot stops and turn away from it
	safe_cmdVel.linear.x = 0;
   	safe_cmdVel.angular.z = turnDirection*0.2;
   	normalRun = false; //but it can't go back to normal path
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "safety_first");
    ros::NodeHandle n;
    ros::Rate loop_rate(3);
/*
 * Publisher for cmd_vel
 * Todo: connect it with a function publishing speed to follow the track
 */
    ros::Publisher pub_cmdVel = n.advertise<geometry_msgs::Twist > ("/cmd_vel_mux/input/navi", 1); //This is the topic we need
//    ros::Publisher pub_cmdVel = n.advertise<geometry_msgs::Twist > ("/cmd_vel", 1); //Test topic.. robot will not move

/*
 * Subscriber for laser scan msg and original cmdVel
 */
    ros::Subscriber laserScan = n.subscribe("/scan", 1, &laser_scan);
    ros::Subscriber origCmdVel = n.subscribe("unsafe_cmd_vel", 1, &original_cmdVel_callBack);
    ROS_INFO("Object avoiding started\n");

    bool finished = false;
    while (ros::ok())
    {


        if (!isSafetyNeeded && normalRun){
               	//pub_cmdVel.publish(original_cmdVel);
		safe_cmdVel.linear.x = 0.3;
		pub_cmdVel.publish(safe_cmdVel);		
//ROS_INFO("Continue Onwards\n");
               }

        if (isSafetyNeeded){
        	avoid_obstacle();
        	pub_cmdVel.publish(safe_cmdVel);
		      ROS_INFO("Robot started avoiding\n");
            ros::spinOnce();
        	continue;
        }
        if (!isSafetyNeeded && !normalRun){
        	safe_cmdVel.linear.x = 0.2;
        	safe_cmdVel.angular.z = 0;
        	pub_cmdVel.publish(safe_cmdVel);
        	normalRun = true;
		ROS_INFO("Robot finished avoiding\n");
        }



        if (finished) // safe exit when robot reaches target
        {
            safe_cmdVel.linear.x = 0;
            safe_cmdVel.angular.z = 0;
            pub_cmdVel.publish(safe_cmdVel);
            ROS_INFO("Finished!!!");
            ros::spinOnce();
            return 0;
        }

        ros::spinOnce();
        loop_rate.sleep();

    }

    //safe stop when program is interrupted
    safe_cmdVel.linear.x = 0;
    safe_cmdVel.angular.z = 0;
    pub_cmdVel.publish(safe_cmdVel);

    return 0;
}

