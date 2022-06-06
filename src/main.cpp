#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <ros/service_client.h>
#include <nav_msgs/OccupancyGrid.h>
#include "tf/transform_listener.h"

#include "../include/Map.cpp"
#include "../include/Explore.cpp"


void mapUpdate(const nav_msgs::OccupancyGrid &new_map);

auto getCurrentPosition(ros::NodeHandle *n) -> std::pair<float, float>;

void mapReadData(const nav_msgs::OccupancyGrid &new_map);


nav_msgs::OccupancyGrid map_read;
ros::Subscriber found_sub, map_sub, map_get_data;
bool map_valid = false;

std::vector<int> map_raw_data;
Map updatingMap;

void print(std::vector<int> const &input) {
    std::copy(input.begin(),
              input.end(),
              std::ostream_iterator<int>(std::cout, " "));
}

// main function
int main(int argc, char **argv) {
    ROS_INFO("[MRP LOG] Starting the workflow!");

    // initialize node
    ros::init(argc, argv, "search");

    // node handle
    ros::NodeHandle *n = new ros::NodeHandle();

    map_sub = n->subscribe("/map", 1, mapUpdate);

    map_get_data = n->subscribe("/map", 1, mapReadData);

    while (!map_valid) {
        ros::Duration(1).sleep();
        ros::spinOnce(); // check for map update
    }

    int mapWidth = map_read.info.width;
    int mapHeight = map_read.info.height;
    ROS_INFO("Map width:%d, Map Height:%d", mapWidth, mapHeight);


    auto currPos = getCurrentPosition(n);
    Explore exp(updatingMap, currPos.first, currPos.second);
    exp.start();

}

// callback function that gets the map
void mapUpdate(const nav_msgs::OccupancyGrid &new_map) {
    map_read = new_map;
    map_valid = true;
}

auto getCurrentPosition(ros::NodeHandle *n) -> std::pair<float, float> {
    tf::TransformListener listener;
    tf::StampedTransform transform;

    while (n->ok()) {
        try {
            ROS_INFO("[MRP LOG] Attempting to read pose...");
            listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
            break;
        }
        catch (tf::TransformException ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(5).sleep();
            ros::spinOnce();
        }
    }

    ROS_INFO("[MRP LOG] Got the position x = %f, y = %f", transform.getOrigin().x(), transform.getOrigin().y());
    return std::make_pair(transform.getOrigin().x(), transform.getOrigin().getY());
}

void mapReadData(const nav_msgs::OccupancyGrid &new_map) {
    ROS_INFO("Map reading started!");
    ROS_INFO("Map Height: %d, Map Width: %d", new_map.info.height, new_map.info.width);

    // saving the data from the map into a vector to pass it to Map class
    std::vector<int> map_data(std::begin(new_map.data), std::end(new_map.data));
    Map map(map_data, new_map.info.width);
    updatingMap = map;

    ROS_INFO("Map reading finished!");
}