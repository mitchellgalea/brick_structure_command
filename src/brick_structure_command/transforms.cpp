#include "brick_structure_command/transforms.h"

using namespace brick_structure_command;

Transforms::Transforms()
{}

geometry_msgs::Pose Transforms::pose2d2Pose(geometry_msgs::Pose2D pose_in)
{
    geometry_msgs::Pose pose_out;
    pose_out.position.x = pose_in.x;
    pose_out.position.y = pose_in.y;
    tf2::Matrix3x3 rot_m;
    rot_m.setEulerYPR(pose_in.theta, 0, 0);
    tf2::Quaternion rot_q;
    rot_m.getRotation(rot_q);
    pose_out.orientation = tf2::toMsg(rot_q);
    return pose_out;
}

geometry_msgs::Pose Transforms::tfPose(geometry_msgs::Pose pose_in, double x, double y,
                                  double z, double yaw)
{
    tf2::Vector3 v;
    fromMsg(pose_in.position, v);
    tf2::Quaternion r;
    fromMsg(pose_in.orientation, r);
    tf2::Transform t(r, v);

    tf2::Vector3 v_t(x, y, z);
    tf2::Matrix3x3 r_t;
    r_t.setEulerYPR(yaw, 0, 0);
    tf2::Transform t_t(r_t, v_t);

    tf2::Transform t_o = t * t_t;
    geometry_msgs::Pose pose_out;
    pose_out = tf2::toMsg(t_o, pose_out);
    return pose_out;
}

double Transforms::pose2Yaw(geometry_msgs::Pose pose)
{
    tf2::Quaternion r;
    tf2::fromMsg(pose.orientation, r);
    tf2::Matrix3x3 r_m(r);
    double yaw, roll, pitch;
    r_m.getEulerYPR(yaw, roll, pitch);
    return yaw;
}
