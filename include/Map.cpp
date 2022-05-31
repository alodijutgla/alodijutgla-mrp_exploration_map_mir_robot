#ifndef MAP_CPP
#define MAP_CPP


#include <vector>
#include <iostream>
#include <cmath>
#include <string>

using namespace std;

class Map
{

public:
    Map(vector<int> map_data, int width)
            : map_data_(map_data), width_(width)
    {
   cout << "Created mapp with width:" + to_string(width) <<endl;
    }




    vector<int> *getMap()
    {
        return &map_data_;
    }


    bool isWall(int x, int y)
    {
        return map_data_[x + y * width_] == 1;
    }




private:

    vector<int> map_data_;
    int width_;

};

#endif