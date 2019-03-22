#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"

#include <sstream>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  return 0;
}
