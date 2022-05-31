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

        switch (state)
        {
            case 0:

                direction = moveToWall();
                //state = 1;

                break;

            case 1:

                followWall(direction);
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

    int state;
    int seq_goal = 0;    // sequence id of goal

    // follow the wall as long she goes straigth
    void followWall(int direction)
    {

        while (map_.isWall(act_x, act_y))
        {
        }
    }

    // 1 positiv x
    // 2 negativ x
    // 3 positiv y
    // 4 negativ y
    // -1 faullt
    pair<int, int> getCordinates(int direction)
    {
        switch (direction)
        {
            case 1:
                return make_pair(act_x + 1, act_y);
                break;
            case 2:
                return make_pair(act_x - 1, act_y);
                break;
            case 3:
                return make_pair(act_x + 1, act_y);
                break;
            case 4:
                return make_pair(act_x + 1, act_y);
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