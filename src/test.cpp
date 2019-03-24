#include "ros/ros.h"
<<<<<<< HEAD
#include "std_msgs/String.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"

#include <sstream>
=======

#include <iostream>

#include "brick_command/brick_structure.h"



>>>>>>> 38097712c81864a6607561aead80f81653afd8c4


int main(int argc, char **argv)
{
<<<<<<< HEAD

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;
=======
    chdir("/");
    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    brick_command::BrickStructure structure("/home/mitch/catkin_ws/src/brick_structure_command/blueprints/test.yaml", 0.01);



>>>>>>> 38097712c81864a6607561aead80f81653afd8c4

  return 0;
}
