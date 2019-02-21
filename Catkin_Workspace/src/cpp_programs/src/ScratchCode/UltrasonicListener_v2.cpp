#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"

#include <sstream>

int right, left;	//Right and Left ultrasonic sensor
float track_width = 250;	//Units = cm
float danger = track_width/10;	//danger zone threshold
float warning = track_width/3.3;	//warning zone threshold

bool right_danger, left_danger, right_warning, left_warning;


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
    ROS_INFO("Right Danger set");
  } else if(right_warning) {
    ROS_INFO("Right Warning set");
  }

  if(left_danger) {
    ROS_INFO("Left Danger set");
  } else if(left_warning) {
    ROS_INFO("Left Warning set");
  }


}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ultrasonic_processed");  // Required, string specified what the node name will be

  ros::NodeHandle n;			//Must specify a node handle for subscriber
  ros::NodeHandle t;			//Must specify a node handler for publisher

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);	//Specify topic you will subscribe to and function that will be called when data is detected
  ros::Publisher chatter_pub = t.advertise<std_msgs::Int32>("ultrasonic_processed_talker", 1000);  //Specify topic you will publish to
  
/*A ros::Rate object allows you to specify a frequency that you would like to loop at. 
It will keep track of how long it has been since the last call to Rate::sleep(), and sleep for the correct amount  of time.
In this case we tell it we want to run at 10Hz. */
  ros::Rate loop_rate(10);

  int count = 0;
  while(ros::ok())
  {
  
  std_msgs::Int32 rightd;
  std_msgs::Int32 rightw;
  std_msgs::Int32 leftd;
  std_msgs::Int32 leftw;

  rightd.data = 0;
  rightw.data = 1;
  leftd.data = 2;
  leftw.data = 3;
  
  /*
  if rightd is set, send 0
  if rightw is set, send 1
  if leftd is set, send 2
  if leftw is set, send 3
  */

  if(right_danger) {
    chatter_pub.publish(rightd);
  }
  if(right_warning) {
    chatter_pub.publish(rightw);
  }
  if(left_danger) {
    chatter_pub.publish(leftd);
  }
  if(left_warning) {
    chatter_pub.publish(leftw);
  }

  ros::spinOnce();		//This line is required at the end of your main loop/program
  
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

