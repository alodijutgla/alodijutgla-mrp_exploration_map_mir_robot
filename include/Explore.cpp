#ifndef EXPLORE_CPP
#define EXPLORE_CPP

#include <vector>
#include <iostream>
#include <cmath>
#include "Map.cpp"

#include <ros/ros.h>
#include <ros/service_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;

class Explore
{

public:
    Explore(const Map &map, int x, int y)
        : map_(map), act_x(x), act_y(y), state(0)
    {
    }

    void start()
    {
        int direction;
        int option;
        int direction_old;
        switch (state)
        {
            // move to first wall
        case 0:

            direction = moveToWall();
            // state = 1;

            break;

            // follow wall
        case 1:

            // option 1 wall crossing
            // option 2 wall ending in free space
            option = followWall(direction);

            if (option = 1)
            {
                state = 2;
            }
            else if (option = 2)
            {
                state = 2;
            }

            break;

            // change with wall direction
        case 2:

            direction_old = direction;
            direction = newDirection(direction);
            switchRotation(direction_old, direction);
            state = 1;
            break;

        default:
            break;
        }
    }

    void rotate(int deg)
    {
        send_goal(act_x, act_y, deg);
    }

private:
    Map map_;

    int act_x;
    int act_y;

    pair<int, int> Wall;

    int state;
    int seq_goal = 0; // sequence id of goal

    // 1 positiv x
    // 2 negativ x
    // 3 positiv y
    // 4 negativ y
    // -1 faullt
    // rotate to new direction and move last square in the old direction to again close to the wall
    void switchRotation(int direction_old, int direction)
    {
        switch (direction_old)
        {
        case 1:

            if (direction == 3)
            {
                send_goal(act_x + 1, act_y, -90);
            }
            else if (direction == 4)
            {
                send_goal(act_x + 1, act_y, 90);
            }
            break;
        case 2:

            if (direction == 3)
            {
                send_goal(act_x - 1, act_y, 90);
            }
            else if (direction == 4)
            {
                send_goal(act_x - 1, act_y, -90);
            }
            break;
        case 3:

            if (direction == 1)
            {
                send_goal(act_x, act_y + 1, 90);
            }
            else if (direction == 2)
            {
                send_goal(act_x, act_y + 1, -90);
            }
            break;
        case 4:

            if (direction == 1)
            {
                send_goal(act_x, act_y - 1, -90);
            }
            else if (direction == 2)
            {
                send_goal(act_x, act_y - 1, 90);
            }
            break;

        default:
            break;
        }
    }

    // follow the wall as long she goes straigth
    // if wall is ending because a crossing wall return 1
    // if wall is ending in free space return 2
    int followWall(int direction)
    {
        // set new robot cordinates diretly one in frnt because one square is maybe not enough to turn
        pair<int, int> newRobotCord = getCordinates(direction, act_x, act_y);
        pair<int, int> newRobotCord_old;
        while (true)
        {

            Wall = getCordinates(direction, Wall.first, Wall.second);
            newRobotCord = getCordinates(direction, newRobotCord.first, newRobotCord.second);

            if (map_.isWall(newRobotCord.first, newRobotCord.second))
            {
                cout << "New Wall i crossing" << endl;
                return 1;
                break;
            }
            else if (!map_.isWall(Wall.first, Wall.second))
            {
                cout << "Wall is ending" << endl;
                return 2;
                break;
            }
            send_goal(newRobotCord_old.first, newRobotCord_old.second, 0);
            newRobotCord_old = newRobotCord;
        }
    }

    // check if in which direction the robot should turn if there is a wall in front of it
    int newDirection(int direction)
    {
        // 1 positiv x
        // 2 negativ x
        // 3 positiv y
        // 4 negativ y
        // -1 faullt
        switch (direction)
        {
        case 1:
        case 2:
            if (map_.isWall(act_x, act_y + 1))
            {
                cout << "New direction is Y negativ" << endl;
                return 4;
            }
            else if (map_.isWall(act_x, act_y - 1))
            {
                cout << "New direction is Y positiv" << endl;
                return 3;
            }
            else
            {
                cout << "Error something went wrong at finding a new direction" << endl;
            }
            break;
        case 3:
        case 4:
            if (map_.isWall(act_x + 1, act_y))
            {
                cout << "New direction is X negativ" << endl;
                return 2;
            }
            else if (map_.isWall(act_x - 1, act_y))
            {
                cout << "New direction is X positive" << endl;
                return 1;
            }
            else
            {
                cout << "Error something went wrong at finding a new direction" << endl;
            }

            break;

        default:
            break;
        }
    }

    // 1 positiv x
    // 2 negativ x
    // 3 positiv y
    // 4 negativ y
    // -1 faullt
    // get next wall cordinate
    pair<int, int> getCordinates(int direction, int x, int y)
    {
        switch (direction)
        {
        case 1:
            return make_pair(x + 1, y);
            break;
        case 2:
            return make_pair(x - 1, y);
            break;
        case 3:
            return make_pair(x, y + 1);
            break;
        case 4:
            return make_pair(x, y - 1);
            break;

        default:
            break;
        }
    }

    // move near to the next wall
    int moveToWall()
    {

        float x = act_x;
        float y = act_y;
        // rotate about 360°
        rotate(360);

        cout << "Robot startet at x: " + to_string(act_x) + " And Y: " + to_string(act_y);

        // amount of trys for find a wall
        for (int i = 0; i < 10; i++)
        {
            // if wall found move there
            if (findWall(x, y))
            {
                // 1 positiv x
                // 2 negativ x
                // 3 positiv y
                // 4 negativ y
                // -1 faullt only one wall pice

                int direction = checkWallDirection(x, y);

                cout << "Wall direction is; " + to_string(direction);

                switch (direction)
                {
                case 1:

                    // check on which side of the wall we are
                    if (x < act_x)
                    {
                        x = x + 1;
                    }
                    else
                    {
                        x = x - 1;
                    }

                    break;

                case 2:

                    x = x + 1;
                    rotate(180); // for always look in moving direction
                    break;

                case 3:

                    y = y - 1;
                    rotate(270); // for always look in moving direction
                    break;

                case 4:

                    y = y + 1;
                    rotate(90); // for always look in moving direction
                    break;

                case -1:
                    ROS_ERROR("Wall has no direction");

                    break;

                default:
                    ROS_INFO("check direction unexpected value");

                    break;
                }

                // move close to wall
                send_goal(x, y, 0);
                return direction;
            }
            // wall not found move in x positive direction
            else
            {
                act_x = act_x + 5;
                send_goal(act_x, 0, 0);
                rotate(360);
            }
        }

        return -1;
    }

    // check in which direction the wall is going
    int checkWallDirection(int x, int y)
    {

        if (map_.isWall(x + 1, y) && map_.isWall(x + 2, y))
        {
            return 1; // wall in xpositive direction
        }
        else if (map_.isWall(x - 1, y) && map_.isWall(x - 2, y))
        {
            return 2; // x negativ direction
        }
        else if (map_.isWall(x, y + 1) && map_.isWall(x, y + 2))
        {
            return 3; // y positiv direction
        }
        else if (map_.isWall(x, y - 1) && map_.isWall(x, y - 1))
        {
            return 4; // y negativ direction
        }
        else
        {
            return -1;
        }
    }

    // find the nearest wall
    bool findWall(float &x, float &y)
    {

        bool wall = false;
        int direction = 0;
        int xTargetSteps = 1;
        int yTargetSteps = 1;
        int Steps = 1;

        while (x < 5000)
        {
            switch (direction)
            {
            // x positive
            case 0:

                x = x + 1;
                Steps++;

                if (xTargetSteps == Steps)
                {
                    direction++;
                    Steps = 1;
                    xTargetSteps++;
                }
                break;

                // y negative
            case 1:
                y = y - 1;
                Steps++;

                if (yTargetSteps == Steps)
                {
                    direction++;
                    Steps = 1;
                    yTargetSteps++;
                }

                break;

                // x negative
            case 2:
                x = x - 1;
                Steps++;

                if (xTargetSteps == Steps)
                {
                    direction++;
                    Steps = 1;
                    xTargetSteps++;
                }
                break;

                // y positive
            case 3:
                y = y + 1;
                Steps++;

                if (yTargetSteps == Steps)
                {
                    direction++;
                    Steps = 1;
                    yTargetSteps++;
                }
                break;
                // error
            default:
                ROS_ERROR(" sEARCH FOR WALL FAILED BECAUSE OF WRONG INPUT");
                break;
            }

            wall = map_.isWall(x, y);
            if (wall)
            {

                Wall.first = x;
                Wall.second = y;
                cout << "First Wall founded at " + to_string(x) + " And Y: " + to_string(y);
                // wall found
                return true;
            }
        }
        //  no wall found
        return false;
    }

    // send a goal to the robot
    bool send_goal(float x, float y, float rotation)
    {
        // Move the robot with the help of an action client. Goal positions are
        // transmitted to the robot and feedback is given about the actual
        // driving state of the robot.

        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
        while (!ac.waitForServer(ros::Duration(10.0)))
            ;

        move_base_msgs::MoveBaseGoal goal_msgs;

        goal_msgs.target_pose.header.seq = seq_goal;
        goal_msgs.target_pose.header.stamp = ros::Time::now();
        goal_msgs.target_pose.header.frame_id = "map";
        goal_msgs.target_pose.pose.position.x = x;
        goal_msgs.target_pose.pose.position.y = y;
        goal_msgs.target_pose.pose.position.z = 0;
        goal_msgs.target_pose.pose.orientation.x = 0;
        goal_msgs.target_pose.pose.orientation.y = 0;
        goal_msgs.target_pose.pose.orientation.z = rotation;
        goal_msgs.target_pose.pose.orientation.w = 1;

        ++seq_goal;

        ac.sendGoal(goal_msgs);
        ac.waitForResult(ros::Duration(7.0));

        while (ac.getState() == actionlib::SimpleClientGoalState::PENDING)
            ;

        while (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
            ;

        while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
            {
                ROS_ERROR("ABORTED");
                return false;
                break;
            }
        }

        ROS_INFO("Reached point x:%.2f, y:%.2f", x, y);
        act_x = x;
        act_y = y;

        return true;
    }
};

#endif