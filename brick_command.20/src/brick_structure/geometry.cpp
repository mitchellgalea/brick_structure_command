#include "brick_structure/geometry.h"

using namespace brick_structure;

Geometry::Geometry()
{}

Pose Geometry::zeroPose()
{
    Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 0;
    pose.orientation.w = 1.0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    return pose;
}

Pose Geometry::pose2d2Pose(Pose2D pose_in)
{
    Pose pose_out;
    pose_out.position.x = pose_in.x;
    pose_out.position.y = pose_in.y;
    tf2::Matrix3x3 rot_m;
    rot_m.setEulerYPR(pose_in.theta, 0, 0);
    tf2::Quaternion rot_q;
    rot_m.getRotation(rot_q);
    pose_out.orientation = tf2::toMsg(rot_q);
    return pose_out;
}

Pose Geometry::tfPose(Pose pose_in, double x, double y,
                                  double z, double yaw)
{
    if(fabs(pose_in.orientation.w) < EPSILION &&
       fabs(pose_in.orientation.x) < EPSILION &&
       fabs(pose_in.orientation.y) < EPSILION &&
       fabs(pose_in.orientation.z) < EPSILION)pose_in.orientation.w = 1;
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
    Pose pose_out;
    pose_out = tf2::toMsg(t_o, pose_out);
    return pose_out;
}

Pose Geometry::tfPose(Pose pose_in, Pose transform)
{
    if(fabs(pose_in.orientation.w) < EPSILION &&
       fabs(pose_in.orientation.x) < EPSILION &&
       fabs(pose_in.orientation.y) < EPSILION &&
       fabs(pose_in.orientation.z) < EPSILION)pose_in.orientation.w = 1;
    if(fabs(transform.orientation.w) < EPSILION &&
        fabs(transform.orientation.x) < EPSILION &&
        fabs(transform.orientation.y) < EPSILION &&
        fabs(transform.orientation.z) < EPSILION)transform.orientation.w = 1;
    tf2::Vector3 v;
    fromMsg(pose_in.position, v);
    tf2::Quaternion r;
    fromMsg(pose_in.orientation, r);
    tf2::Transform t(r, v);
    tf2::Vector3 v_t;
    fromMsg(transform.position, v_t);
    tf2::Quaternion r_t;
    fromMsg(transform.orientation, r_t);
    tf2::Transform t_t(r_t, v_t);

    tf2::Transform t_o = t * t_t;
    Pose pose_out;
    pose_out = tf2::toMsg(t_o, pose_out);
    return pose_out;
}

Pose Geometry::relativePose(Pose pose_a, Pose pose_b)
{
    tf2::Vector3 v_a;
    fromMsg(pose_a.position, v_a);
    tf2::Quaternion r_a;
    fromMsg(pose_a.orientation, r_a);
    tf2::Transform t_a(r_a, v_a);

    tf2::Vector3 v_b;
    fromMsg(pose_b.position, v_b);
    tf2::Quaternion r_b;
    fromMsg(pose_b.orientation, r_b);
    tf2::Transform t_b(r_b, v_b);

    tf2::Transform t = t_a.inverseTimes(t_b);
    Pose pose_ref;
    pose_ref= tf2::toMsg(t,pose_ref);
    return pose_ref;
}

Pose Geometry::relativePose(Pose pose, Point point)
{
    tf2::Vector3 v_a;
    fromMsg(pose.position, v_a);
    tf2::Quaternion r_a;
    fromMsg(pose.orientation, r_a);
    tf2::Transform t_a(r_a, v_a);

    tf2::Vector3 v_b;
    fromMsg(point, v_b);
    tf2::Quaternion r_b;
    tf2::Transform t_b(r_b, v_b);

    tf2::Transform t = t_a.inverseTimes(t_b);
    Pose pose_ref;
    pose_ref= tf2::toMsg(t,pose_ref);
    return pose_ref;
}

double Geometry::pose2Yaw(Pose pose)
{
    tf2::Quaternion r;
    tf2::fromMsg(pose.orientation, r);
    tf2::Matrix3x3 r_m(r);
    double yaw, roll, pitch;
    r_m.getEulerYPR(yaw, roll, pitch);
    return yaw;
}

Plane Geometry::points2Plane(Pose a,Pose b,Pose c)
{
    Plane plane;
    plane.pose = a;
    tf2::Vector3 ab(b.position.x - a.position.x, b.position.y - a.position.y, b.position.z - a.position.z);
    tf2::Vector3 ac(c.position.x - a.position.x, c.position.y - a.position.y, c.position.z - a.position.z);
    tf2::Vector3 normal = ab.cross(ac);
    geometry_msgs::Vector3 normal_msg;
    normal_msg.x = normal.getX();
    normal_msg.y = normal.getY();
    normal_msg.z = normal.getZ();
    plane.normal = normal_msg;
    return plane;
}

Plane Geometry::points2Plane(Point a,Point b,Point c)
{
    Plane plane;
    tf2::Vector3 ab(b.x - a.x, b.y - a.y, b.z - a.z);
    tf2::Vector3 ac(c.x - a.x, c.y - a.y, c.z - a.z);
    tf2::Vector3 normal = ab.cross(ac);
    geometry_msgs::Vector3 normal_msg;
    normal_msg.x = normal.getX()/sqrt(pow(normal.getX(), 2) + pow(normal.getY(), 2));
    normal_msg.y = normal.getY()/sqrt(pow(normal.getX(), 2) + pow(normal.getY(), 2));
    normal_msg.z = 0;
    plane.normal = normal_msg;
    Pose2D pose_2d;
    pose_2d.x = a.x;
    pose_2d.y = a.y;
    double theta = atan2(plane.normal.y, plane.normal.x);
    pose_2d.theta = theta - (M_PI/2);
    plane.pose = pose2d2Pose(pose_2d);
    plane.pose.position.z = a.z;
    return plane;
}

bool Geometry::pointOnPlane(Point point,Plane plane)
{
    tf2::Vector3 normal(plane.normal.x, plane.normal.y, plane.normal.z);
    tf2::Vector3 v(plane.pose.position.x - point.x, plane.pose.position.y - point.y, plane.pose.position.z - point.z);
    double dot = normal.dot(v);
    if(fabs(dot) < 0.00001) return true;
    return false;
}

std::vector<Point> Geometry::pointsOnPlane(std::vector<Point> points,Plane plane)
{
    std::vector<Point> plane_points;
    for(auto point:points)
    {
        if(Geometry::pointOnPlane(point, plane)) plane_points.push_back(point);
    }
    return plane_points;
}

bool Geometry::poseOnPlane(Pose pose,Plane plane)
{
    tf2::Vector3 normal(plane.normal.x, plane.normal.y, plane.normal.z);
    tf2::Vector3 v(plane.pose.position.x - pose.position.x, plane.pose.position.y -  pose.position.y, plane.pose.position.z -  pose.position.z);
    double dot = normal.dot(v);
    if(fabs(dot) < 0.00001) return true;
    return false;
}

std::vector<Pose> Geometry::posesOnPlane(std::vector<Pose> poses,Plane plane)
{
    std::vector<Pose> plane_poses;
    for(auto pose:poses)
    {
        if(Geometry::poseOnPlane(pose, plane)) plane_poses.push_back(pose);
    }
    return plane_poses;
}

double Geometry::pointDistance(Point point_1, Point point_2, bool x, bool y, bool z)
{
    double dx = point_1.x - point_2.x;
    double dy = point_1.y - point_2.y;
    double dz = point_1.z - point_2.z;
    if(!x) dx = 0;
    if(!y) dy = 0;
    if(!z) dz = 0;
    return sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
}

double Geometry::pointDistance(PCPoint point_1, PCPoint point_2, bool x, bool y, bool z)
{
    double dx = point_1.x - point_2.x;
    double dy = point_1.y - point_2.y;
    double dz = point_1.z - point_2.z;
    if(!x) dx = 0;
    if(!y) dy = 0;
    if(!z) dz = 0;
    return sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
}

direction Geometry::getPointDirection(PointCloud::Ptr point_cloud, double max_distance, unsigned index)
{
    if(index > point_cloud->points.size())
    {
        ROS_ERROR("Get Point Direction: Index out of range");
    }
    PCPoint index_point = point_cloud->points[index];
    std::vector<double> xy_distances;
    std::vector<double> z_distances;

    for(auto point:point_cloud->points)
    {
        double distance = pointDistance(index_point, point);
        if(distance < max_distance)
        {
            double xy_distance = pointDistance(index_point, point, true, true, false);
            double z_distance = pointDistance(index_point, point, false, false, true);
            xy_distances.push_back(xy_distance);
            z_distances.push_back(z_distance);
        }
    }

    if(xy_distances.size() > 0 && z_distances.size() > 0)
    {
        double xy_mean = std::accumulate(std::begin(xy_distances), std::end(xy_distances), 0.0) / xy_distances.size();
        double z_mean = std::accumulate(std::begin(z_distances), std::end(z_distances), 0.0) / z_distances.size();
        if (xy_mean < z_mean) return direction::vertical;
        else return direction::horizontal;
    }
    else
    {
        ROS_ERROR("Get Point Direction: No Close Points");
    }
}

Point Geometry::projectPointOnPlane(Point point, Plane plane)
{
    // Compute unit vector for plane
    Vector3 n_u;
    double mag = sqrt(pow(plane.normal.x,2) + pow(plane.normal.y,2) + pow(plane.normal.z,2));
    n_u.x = plane.normal.x/mag;
    n_u.y = plane.normal.y/mag;
    n_u.z = plane.normal.z/mag;

    Vector3 v;
    v.x = point.x - plane.pose.position.x;
    v.y = point.y - plane.pose.position.y;
    v.z = point.z - plane.pose.position.z;

    double dist = v.x*n_u.x + v.y*n_u.y + v.z*n_u.z;

    Point p_point;
    p_point.x = point.x - dist*plane.normal.x;
    p_point.y = point.y - dist*plane.normal.y;
    p_point.z = point.z - dist*plane.normal.z;

    return p_point;
}

double Geometry::linePointDistance(Point point, Point l_point_1, Point l_point_2)
{
    // Calculates the perpendicular distance between the point and the line
    tf2::Vector3 a(l_point_1.x - l_point_2.x, l_point_1.y - l_point_2.y, l_point_1.z - l_point_2.z);
    tf2::Vector3 b(point.x - l_point_2.x, point.y - l_point_2.y, point.z - l_point_2.z);
    tf2::Vector3 c = a.cross(b);
    double c_mag = sqrt(pow(c.getX(),2) + pow(c.getY(),2) + pow(c.getZ(),2));
    double a_mag = sqrt(pow(a.getX(),2) + pow(a.getY(),2) + pow(a.getZ(),2));
    double dist = c_mag / a_mag;

    // Checks angles to check whether intersection point lies between line segment
    double lp1_dist = pointDistance(point, l_point_1);
    double lp2_dist = pointDistance(point, l_point_2);
    double l_dist =  pointDistance(l_point_1, l_point_2);

    double angle_1 = acos((pow(lp1_dist,2)+pow(l_dist,2)-pow(lp2_dist,2))/2*lp1_dist*l_dist);
    double angle_2 = acos((pow(lp2_dist,2)+pow(l_dist,2)-pow(lp1_dist,2))/2*lp2_dist*l_dist);

    // If one of the angles is greater than pi/2 the triangle is obtuse and the closest
    // points is the minimum lp_dist
    if(angle_1 > M_PI/2 || angle_2 > M_PI/2 ) return std::min(lp1_dist, lp2_dist);
    else return dist;
}

double Geometry::linePointsDistanceOutput(std::vector<Point> h_points, std::vector<Point> v_points,
                                                 std::vector<Line> h_lines, std::vector<Line> v_lines)
{
    double output = 0;
    for(auto point:h_points)
    {
        double min_dist = linePointDistance(point, h_lines[0].point_1, h_lines[0].point_2);
        for(auto line:h_lines)
        {
            double dist = linePointDistance(point, line.point_1, line.point_2);
            if(dist < min_dist) min_dist = dist;
        }
        output += pow(min_dist, 2);
    }

    for(auto point:v_points)
    {
        double min_dist = linePointDistance(point, v_lines[0].point_1, v_lines[0].point_2);
        for(auto line:v_lines)
        {
            double dist = linePointDistance(point, line.point_1, line.point_2);
            if(dist < min_dist) min_dist = dist;
        }
        output += pow(min_dist, 2);
    }
    return output;
}

void Geometry::shiftLines(Pose pose, std::vector<LineVec> h_linevecs, std::vector<LineVec> v_linevecs,
                        std::vector<Line> &h_lines, std::vector<Line> &v_lines)
{
    h_lines.clear();
    v_lines.clear();
    for(auto linevec:h_linevecs)
    {
        Pose p1_pose = tfPose(pose, linevec.vec_1.x, linevec.vec_1.y, linevec.vec_1.z, 0);
        Pose p2_pose = tfPose(pose, linevec.vec_2.x, linevec.vec_2.y, linevec.vec_2.z, 0);
        Line h_line;
        h_line.point_1 = p1_pose.position;
        h_line.point_2 = p2_pose.position;
        h_lines.push_back(h_line);
    }
    for(auto linevec:v_linevecs)
    {
        Pose p1_pose = tfPose(pose, linevec.vec_1.x, linevec.vec_1.y, linevec.vec_1.z, 0);
        Pose p2_pose = tfPose(pose, linevec.vec_2.x, linevec.vec_2.y, linevec.vec_2.z, 0);
        Line v_line;
        v_line.point_1 = p1_pose.position;
        v_line.point_2 = p2_pose.position;
        v_lines.push_back(v_line);
    }
}

std::vector<PoseStamped> Geometry::posesToPoseStamped(std::vector<Pose> poses, std::string frame) {
    std::vector<PoseStamped> stamped_poses;
    for(auto pose:poses)
    {
        PoseStamped stamped_pose;
        stamped_pose.pose = pose;
        stamped_pose.header.frame_id = frame;
        stamped_poses.push_back(stamped_pose);
    }
    return stamped_poses;
}


