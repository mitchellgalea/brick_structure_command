#include "ros/ros.h"

#include <iostream>

#include "brick_command/brick_structure.h"





int main(int argc, char **argv)
{
    chdir("/");
    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    brick_command::BrickStructure structure("/home/mitch/catkin_ws/src/brick_structure_command/blueprints/test.yaml", 0.01);




  return 0;
}
