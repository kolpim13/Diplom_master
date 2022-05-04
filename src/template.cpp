// ROS headers
#include "ros/ros.h"
#include "std_msgs/String.h"

// User headers
// ...

// User publisher and subscriber definitions
// ...

int main(int argc, char** argv)
{
    // ROS needed initialization
    ros::init(argc, argv, "name");
    ros::NodeHandle n;

    // Publishers and subscribers init
    // ...

    while (ros::ok())
    {
        // Inner logic
        // ...
    }

    return 0;
}
