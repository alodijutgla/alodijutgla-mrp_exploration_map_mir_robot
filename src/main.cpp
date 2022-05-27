#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <ros/service_client.h>
#include <actionlib/client/simple_action_client.h>

// main function
int main(int argc, char **argv) {
    ROS_INFO("Starting the workflow!");

    // initialize node
    ros::init(argc, argv, "search");

    // node handle
    ros::NodeHandle *n = new ros::NodeHandle();


}
