#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"
#include <sstream>
#include <array>

#define START_ANGLE 90-30
#define END_ANGLE 90+31
#define WHEEL_OFFSET 20

float right, left;	//Right and Left ultrasonic sensor
float track_width = 244;	//Units = cm
float danger = (track_width/10);	//danger zone threshold
float warning = (track_width/3.3);	//warning zone threshold

bool right_danger, left_danger, right_warning, left_warning;

float outputAngle;


void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	float movingAverage[121]; // 61 because 60 to 120 is 61 points
	int numDataPointsEachSide = 5;
	float temp;
	float internalAngle;
    float largestDistance;
    float largestAngle;
    static int timeIteration = 0;
    float timeAverage[5][121];
	
	// CALCULATE MOVING AVERAGES 
	// 1. add the current data point plus 5 data points on either side.
	// 2. change any infinite values to 10
	// 3. average the points (sum divided by 11. 11 because 5+1+5)

    //ranges[0] is straight ahead (0 degrees)
	
	for(int i=-60; i<=60; i++){
		temp = 0;
		for(int j = (i-numDataPointsEachSide); j <= (i+numDataPointsEachSide); j++){
            float distance;
            if(j<0) {
                distance = scan->ranges[360+j];
            } else {
                distance = scan->ranges[j];
            }
			
			if(isinf(distance)){
				distance = 8;
			}
			temp = temp + distance;
		}
		temp = temp / (numDataPointsEachSide + numDataPointsEachSide + 1);
		timeAverage[timeIteration][60+i] = temp; // offset by 60 because the loop starts at i=-60
        
        //ROS_INFO("timeAverage: %f", timeAverage[timeIteration][60+i]);
	} 
	
	// FIND LARGEST DISTANCE FROM CALCULATED AVERAGES
    if(timeIteration==4) {
        for(int i = 0; i<120; i++) {
            for(int j = 0; j<3; j++) {
                movingAverage[i] += timeAverage[j][i];
                //ROS_INFO("timeAverage: %f, indices: %i,%i", timeAverage[j][i],j,i);
                movingAverage[i] = movingAverage[i]/3;
            } 
        }
        
        temp = 0;
	    for(int i=0; i<=120; i++){
		    if(movingAverage[i] > temp){
			    temp = i;
                largestDistance = movingAverage[i];
                largestAngle = i - 60;
			    internalAngle = -60+temp; // not sure about this line. adapted it from (-90+i)*-1 for an i=60 loop
		    }
        
	    }
        
    	outputAngle = internalAngle;
	    ROS_INFO("optimal wheel angle: %f , largest distance: %f at %f degrees", outputAngle, largestDistance, largestAngle);

    }

	// maybe add a wait for multiple same flag in a row
    //changed from if(internalAngle>0)
	/*if(true){
		if(right_danger){
			internalAngle = -5;
			ROS_INFO("right danger");
		}
		else if(right_warning){
			internalAngle = (internalAngle*right)/warning;
			ROS_INFO("right warning");
		}
	}

    //changed from if(internalAngle<=0)
	if(true){
		if(left_danger){
			internalAngle = 5;
			ROS_INFO("left danger");
		}
		else if(left_warning){
			internalAngle = (internalAngle*left)/warning;
			ROS_INFO("left warning");
		}
	}*/


    
    timeIteration++;
    if(timeIteration>4) timeIteration = 0;


}

void chatterCallback(const std_msgs::Int32::ConstPtr& msg)
{
  if(msg->data > 400) {
     //ROS_INFO("Right: %dcm\n", msg->data-400);
     right = msg->data-400;
  } else if(msg->data==-1) {
     //ROS_INFO("Left: inf\n");
     left = -1;
  } else if(msg->data==-2) {
     //ROS_INFO("Right: inf\n");
     right = -2;
  } else {
     //ROS_INFO("Left: %dcm\n", msg->data);
     left = msg->data;
  }


  //============================MAIN LOGIC GOES HERE======================================
  if(right<=danger && right>0) {
	right_danger = true;
	right_warning = false;
  } else if(right<=warning && right>danger) {
	right_warning = true;
	right_danger = false;
  } else {
	right_danger = false;
	right_warning = false;
  }

  if(left<=danger && left>0) {
	left_danger = true;
	left_warning = false;
  } else if(left<=warning && left>danger) {
	left_warning = true;
	left_danger = false;
  } else {
	left_danger = false;
	left_warning = false;
  }

  //Debugging outputs
  if(right_danger) {
    //ROS_INFO("Right Danger set");
  } else if(right_warning) {
    //ROS_INFO("Right Warning set");
  }

  if(left_danger) {
    //ROS_INFO("Left Danger set");
  } else if(left_warning) {
    //ROS_INFO("Left Warning set");
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "master_node");  // Required, string specified what the node name will be

  //ros::NodeHandle n;			//Must specify a node handle
  ros::NodeHandle t;
  ros::Subscriber sub = t.subscribe("scan", 1000, lidarCallback);
  //ros::Subscriber sub2 = t.subscribe("chatter", 1000, chatterCallback);	//Required, specify topic you will subscribe to and function that will be called when data is detected
  ros::Publisher master_pub = t.advertise<std_msgs::Float32>("master_output", 1000);

  ros::Rate loop_rate(10);
  int count = 0;
  while(ros::ok())
  {
    	std_msgs::Float32 output;
	output.data = outputAngle;
	master_pub.publish(output);


	ros::spinOnce();
  	loop_rate.sleep();
  	++count;
  }

  return 0;
}


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */


  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */


  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */


  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */

