#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <ros/service_client.h>
#include <actionlib/client/simple_action_client.h>

#include "../include/Map.cpp"
#include "../include/Explore.cpp"

// main function
int main(int argc, char **argv) {
    ROS_INFO("Starting the workflow!");

    // initialize node
    ros::init(argc, argv, "search");

    // node handle
    ros::NodeHandle *n = new ros::NodeHandle();

    std::vector<int> map_data = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                                 1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                 1, 0, 0, 0, 0, 0, 1, 0, 0, 1,
                                 1, 0, 0, 0, 0, 0, 1, 0, 0, 1,
                                 1, 0, 0, 0, 0, 0, 1, 0, 0, 1,
                                 1, 0, 0, 0, 0, 0, 1, 0, 0, 1,
                                 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,};


    Map map(map_data, 10);

    Explore exp(map, 3, 3);
    exp.start();


}