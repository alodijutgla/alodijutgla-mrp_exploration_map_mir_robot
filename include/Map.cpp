#ifndef MAP_CPP
#define MAP_CPP

#include <utility>
#include <vector>
#include <ros/ros.h>

using namespace std;

class Map {

public:
    Map() = default;

    Map(vector<int> map_data, int width)
            : map_data_(std::move(map_data)), width_(width) {
        ROS_INFO("[MRP LOG] Created map with width: %d", width_);
    }

    vector<int> *getMap() {
        return &map_data_;
    }

    // The map data, in row-major order, starting with (0,0).
    // Occupancy probabilities are in the range [0,100]. Unknown is -1.
    bool isWall(int x, int y) {
        return map_data_[x + y * width_] >= 50;
    }

    // option to check if the robot was already there
    // should be stored somehow in the map
    bool alreadyExplored(int x, int y) {
        return map_data_[x + y * width_] == 9999;
    }

private:
    vector<int> map_data_;
    int width_{};
};

#endif