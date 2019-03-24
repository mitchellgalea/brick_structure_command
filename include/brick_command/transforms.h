#ifndef TRANFSORMS_H
#define TRANFSORMS_H

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"

namespace brick_command {

class Transforms
{
private:
    Transforms();
public:
    static geometry_msgs::Pose pose2d2Pose(geometry_msgs::Pose2D pose_in);
    static geometry_msgs::Pose tfPose(geometry_msgs::Pose pose_in, double x, double y,
                                      double z, double yaw);
    static double pose2Yaw(geometry_msgs::Pose pose);
    static geometry_msgs::Pose relativePose(geometry_msgs::Pose pose_a, geometry_msgs::Pose pose_b);
};
}

#endif //TRANFSORMS_H
