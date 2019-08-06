#ifndef BRICKSTRUCTUREGEOMETRY_H
#define BRICKSTRUCTUREGEOMETRY_H

#include <utility>
#include <vector>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Pose2D.h"
#include "ros/ros.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>

#define EPSILION 0.0001

namespace brick_structure {

typedef geometry_msgs::Point Point;
typedef geometry_msgs::Vector3 Vector3;
typedef geometry_msgs::Pose Pose;
typedef geometry_msgs::PoseStamped PoseStamped;
typedef geometry_msgs::Pose2D Pose2D;
typedef pcl::PointXYZ PCPoint;
typedef pcl::PointCloud<PCPoint> PointCloud;
typedef pcl::PCLPointCloud2 PointCloud2;
typedef geometry_msgs::PoseArray PoseArray;

enum class direction {
    horizontal, vertical
};

struct Line {
    Point point_1;
    Point point_2;
};

struct LineVec {
    Vector3 vec_1;
    Vector3 vec_2;
};

struct Plane {
    Pose pose;
    Vector3 normal;
    unsigned brick_index;
    bool brick_face;
};

class Geometry
{
private:
    Geometry();
public:
    static Pose zeroPose();
    static Pose pose2d2Pose(Pose2D pose_in);
    static Pose tfPose(Pose pose_in, double x, double y, double z, double yaw);
    static Pose tfPose(Pose pose_in, Pose transform);
    static double pose2Yaw(Pose pose);
    static Pose relativePose(Pose pose_a, Pose pose_b);
    static Pose relativePose(Pose pose, Point point);
    static Plane points2Plane(Pose a, Pose b, Pose c);
    static Plane points2Plane(Point a, Point b, Point c);
    static bool pointOnPlane(Point point, Plane plane);
    static bool poseOnPlane(Pose pose, Plane plane);
    static std::vector<Point> pointsOnPlane(std::vector<Point> point_cloud, Plane plane);
    static std::vector<Pose> posesOnPlane(std::vector<Pose> poses, Plane plane);
    static double pointDistance(Point point_1, Point point_2, bool x = true, bool y = true, bool z = true);
    static double pointDistance(PCPoint point_1, PCPoint point_2, bool x = true, bool y = true, bool z = true);
    static direction getPointDirection(PointCloud::Ptr points, double max_distance, unsigned index);
    static Point projectPointOnPlane(Point point, Plane plane);
    static double linePointDistance(Point point, Point l_point_1, Point l_point_2);
    static double linePointsDistanceOutput(std::vector<Point> h_points, std::vector<Point> v_points,
                                           std::vector<Line> h_lines, std::vector<Line> v_lines);
    static void shiftLines(Pose pose, std::vector<LineVec> h_linevecs, std::vector<LineVec> v_linevecs,
                            std::vector<Line> &h_lines, std::vector<Line> &v_lines);
    static std::vector<PoseStamped> posesToPoseStamped(std::vector<Pose> poses, std::string frame);

};
}

#endif //BRICKSTRUCTUREGEOMETRY_H
