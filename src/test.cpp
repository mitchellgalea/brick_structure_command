#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geometry_msgs/Pose.h>

#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  tf2::Matrix3x3 rot;
  rot.setEulerYPR(1.57, 0, 0);

  tf2::Vector3 trans(1, 2, 0);

  tf2::Transform tf(rot, trans);

  geometry_msgs::Pose p;

  p.position.x = 0;
  p.position.y = 0;
  p.position.z = 0;

  geometry_msgs::Pose p2 = tf2_geometry_msgs::do_transform_pose(p, tf);



  return 0;
}
