#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>


// Register the servo motors of each joint
Servo motor;    

// Initialize the ROS node
ros::NodeHandle  nh;

/*
 * This function is called each time a new message is published on the topic /servo_actuate
 */
void arm_actuate_cb( const std_msgs::UInt16& msg){
  motor.write(msg.data);
  delay(5);
}

// Define the subscriber to the topic /servo_actuate where are published UInt16MultiArray messages
// Define the function that will be triggered each time a new message is published on this topic
ros::Subscriber<std_msgs::UInt16> sub("robot/arm_actuate", &arm_actuate_cb );


void setup() {
  // Attach and Initialize each Servo to the robot pin where it is connected
  motor.attach(8); 

  // Set a common start point for each joint
  // This way, the start status of each joint is known
  motor.write(90);

  // Inizialize the ROS node on the robot
  nh.initNode();
  // Inform ROS that this node will subscribe to messages on a given topic
  nh.subscribe(sub);
}

void loop() {
  // Keep the ROS node up and running
  nh.spinOnce();
  delay(1);
}
