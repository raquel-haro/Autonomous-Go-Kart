#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"
#include <sstream>
#include <array>

#define START_ANGLE 30
#define CENTER_ANGLE 90
#define END_ANGLE 151
#define WHEEL_OFFSET 15

float right, left;	//Right and Left ultrasonic sensor
float track_width = 244;	//Units = cm
float danger = (track_width/10)+WHEEL_OFFSET;	//danger zone threshold
float warning = (track_width/3.5)+WHEEL_OFFSET;	//warning zone threshold

bool right_danger, left_danger, right_warning, left_warning;

float outputAngle;

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int temp;
	float sumBinElements;
	float largestAverage;
	float internalAngle;
	float numValidPoints;
	int numElementsInBin = 5;
	float longestBin;
	
	ROS_INFO("straight ahead: %f", scan->ranges[CENTER_ANGLE]);
	int numAngles = 25;
	int angles[numAngles] = {
		-60, -55, -50, -45, -40,
		-35, -30, -25, -20, -15, 
		-10, -5, 0, 5, 10, 15, 20, 
		25, 30, 35, 40, 45, 50, 55, 60
	};
	float dataBins[numAngles-1][numElementsInBin] = {0}; 
	
	// go thru each data point and put into bins
	for(int i=START_ANGLE; i<END_ANGLE; i++){
		float degree = (-CENTER_ANGLE+i)*-1;
		float distance = scan->ranges[i];
		//ROS_INFO("angle: %f distance: %f", degree, distance);
        if(isinf(distance)) {
            distance = 4.35;
		}
		if(!isinf(distance)){                                       	//ignore any data points where distance value is inf
			for(int bin=0; bin<numAngles-1; bin++){                 //check element against each bin
				if(degree>angles[bin] && degree<=angles[bin+1]){	// if element fits in current bin
					temp=0;
					while(dataBins[bin][temp]){                     // place element in first non-zero element of current bin
						temp++;
					}
					dataBins[bin][temp] = distance;
					//ROS_INFO("angle: %f distance: %f", degree, distance);

				}
			}
		}
	} 
	
	// find largest bin average
	largestAverage = 0;
	for(int bin=0; bin<numAngles-1; bin++){
		numValidPoints = 0;
		sumBinElements = 0;
		for(int i=0; i<numElementsInBin; i++){
			sumBinElements = sumBinElements + dataBins[bin][i]; // running sum of bin elements
			if(dataBins[bin][i] != 0){    // keep track of non-zero elements in bin
				numValidPoints++;
			}
		}
		//ROS_INFO("bin: %i to %i avg: %f", angles[bin], angles[bin+1], sumBinElements/numValidPoints);
		if( ((sumBinElements/numValidPoints)>largestAverage) && (numValidPoints>0) ){
			largestAverage = (sumBinElements/numValidPoints);
			internalAngle = angles[bin] + 2.5;
		}
		//ROS_INFO("optimal wheel angle: %f", outputAngle);
	}

	// maybe add a wait for multiple same flag in a row
	if(internalAngle>0){
		if(right_danger){
			internalAngle = -5;
			ROS_INFO("Right Danger");
		}
		else if(right_warning){
			internalAngle = (internalAngle*right)/warning;
			ROS_INFO("Right Warning");
		}
	}

	if(internalAngle<=0){
		if(left_danger){
			internalAngle = 5;
			ROS_INFO("Left Danger");
		}
		else if(left_warning){
			internalAngle = (internalAngle*left)/warning;
			ROS_INFO("Left Warning");
		}
	}

	outputAngle = internalAngle;
	ROS_INFO("optimal wheel angle: %f", outputAngle);


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
  /*if(right_danger) {
    ROS_INFO("Right Danger set");
  } else if(right_warning) {
    ROS_INFO("Right Warning set");
  }

  if(left_danger) {
    ROS_INFO("Left Danger set");
  } else if(left_warning) {
    ROS_INFO("Left Warning set");
  }*/
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "master_node");  // Required, string specified what the node name will be

  //ros::NodeHandle n;			//Must specify a node handle
  ros::NodeHandle t;
  ros::Subscriber sub = t.subscribe("scan", 1000, lidarCallback);
  ros::Subscriber sub2 = t.subscribe("chatter", 1000, chatterCallback);	//Required, specify topic you will subscribe to and function that will be called when data is detected
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


	// clear all elements in bins
	/*for(int i=0; i<numAngles-1; i++){ // set all bin elements to 0
		for(int j=0; j<5; j++){
			dataBins[i][j] = 0;
		}
	}*/
