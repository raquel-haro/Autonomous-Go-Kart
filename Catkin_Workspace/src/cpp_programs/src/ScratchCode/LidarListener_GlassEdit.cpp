/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h" //REQUIRED FOR isinf() FUNCTION

#define RAD2DEG(x) ((x)*180./M_PI)


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	int temp;
	float sumBinElements;
	float largestAverage;
	float outputAngle;
	float numValidPoints;
	int numElementsInBin = 5;
	float longestBin;
	
	int numDataPoints = scan->scan_time / scan->time_increment;
	int numAngles = 13;
	int angles[numAngles] = {
		-30, -25, -20, -15, -10,
		-5, 0, 5, 10, 15, 20, 
		25, 30
	};
	float dataBins[numAngles-1][numElementsInBin]; 
	float binAverages[numAngles-1][5];
	
	// clear all elements in bins
	for(int i=0; i<numAngles-1; i++){ // set all bin elements to 0
		for(int j=0; j<5; j++){
			dataBins[i][j] = 0;
		}
	}
	
	// go thru each data point and put into bins
	for(int i=150; i<211; i++){
		float degree = (-180+i)*-1;
		float distance = scan->ranges[i];
		//ROS_INFO("angle: %f distance: %f", degree, distance);
		
		if(!isinf(distance)){                                       //ignore any data points where distance value is inf
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
			outputAngle = angles[bin] + 2.5;
		}
		ROS_INFO("optimal wheel angle: %f", outputAngle);
	}
	
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
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
  ros::init(argc, argv, "lidarListen");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

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
// %Tag(SUBSCRIBER)%
  ros::Subscriber sub = n.subscribe("/scan", 1000, scanCallback);
// %EndTag(SUBSCRIBER)%

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%

