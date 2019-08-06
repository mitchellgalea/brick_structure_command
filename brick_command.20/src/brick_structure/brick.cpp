#include "brick_structure/brick.h"

using namespace brick_structure;
//////// Brick Class Methods
Brick::Brick()
{
    z_dim_ = RedBrick::z_dim;
}

Brick::Brick(double x_dim, double y_dim, double z_dim,
             uint8_t r, uint8_t g, uint8_t b, Pose pose, std::string color)
             :x_dim_(x_dim), y_dim_(y_dim), z_dim_(z_dim),
             r_(r), g_(g), b_(b), pose_(pose), color_(color)
{}

Brick::Brick(char color, Pose pose)
    :pose_(pose)
{
    switch (color) {
        case 'R':
            color_ = RedBrick::color;
            x_dim_ = RedBrick::x_dim;
            y_dim_ = RedBrick::y_dim;
            z_dim_ = RedBrick::z_dim;
            r_ = RedBrick::r;
            g_ = RedBrick::g;
            b_ = RedBrick::b;
            break;
        case 'G':
            color_ = GreenBrick::color;
            x_dim_ = GreenBrick::x_dim;
            y_dim_ = GreenBrick::y_dim;
            z_dim_ = GreenBrick::z_dim;
            r_ = GreenBrick::r;
            g_ = GreenBrick::g;
            b_ = GreenBrick::b;
            break;
        case 'B':
            color_ = BlueBrick::color;
            x_dim_ = BlueBrick::x_dim;
            y_dim_ = BlueBrick::y_dim;
            z_dim_ = BlueBrick::z_dim;
            r_ = BlueBrick::r;
            g_ = BlueBrick::g;
            b_ = BlueBrick::b;
            break;
        case 'O':
            color_ = OrangeBrick::color;
            x_dim_ = OrangeBrick::x_dim;
            y_dim_ = OrangeBrick::y_dim;
            z_dim_ = OrangeBrick::z_dim;
            r_ = OrangeBrick::r;
            g_ = OrangeBrick::g;
            b_ = OrangeBrick::b;
            break;
    }
}

//////// Brick GETTERS
Pose Brick::getPose() const
{ return pose_; }
std::string Brick::getColor() const
{ return color_; }
double Brick::getXDim() const
{ return x_dim_; }
double Brick::getYDim() const
{ return y_dim_; }
double Brick::getZDim() const
{ return z_dim_; }
uint8_t Brick::getR() const
{ return r_; }
uint8_t Brick::getG() const
{ return g_; }
uint8_t Brick::getB() const
{ return b_; }

mrc_msgs::Brick Brick::getBrickMsg() const {
    mrc_msgs::Brick brick_msg;
    brick_msg.color = getColor();
    brick_msg.pose.pose = getPose();
    return brick_msg;
}

Pose Brick::getCentrePose()
{
    return Geometry::tfPose(pose_, x_dim_/2, y_dim_/2, z_dim_/2, 0);
}

//////// Brick SETTERS
void Brick::setPose(Pose pose)
{ pose_ = pose; }

//////// Brick Methods
std::vector<Point> Brick::getPoints(bool top, double interval)
{
    std::vector<Point> points;
    Point p1 = Geometry::tfPose
    (getPose(), 0, 0, top*getZDim(), 0).position;
    Point p2 = Geometry::tfPose
    (getPose(), getXDim(), 0, top*getZDim(), 0).position;
    double yaw = Geometry::pose2Yaw(pose_);

    points.push_back(p1);
    double distance = sqrt(pow(p1.x -p2.x, 2) + pow(p1.y - p2.y, 2));
    for(double d = interval; d < distance; d = d + interval)
    {
        Point point;
        point.x = p1.x + d*cos(yaw);
        point.z = p1.z;
        point.y = p1.y + d*sin(yaw);
        points.push_back(point);
    }

    return points;
}

std::vector<Point> Brick::getCornerPoints()
{
    std::vector<Point> points;
    // pushes back each corner pose by transforming from the brick pose by the dimensions
    points.push_back(Geometry::tfPose(pose_, 0, 0, 0, 0).position);
    points.push_back(Geometry::tfPose(pose_, 0, 0, z_dim_, 0).position);
    points.push_back(Geometry::tfPose(pose_, x_dim_, 0, z_dim_, 0).position);
    points.push_back(Geometry::tfPose(pose_, x_dim_, 0, 0, 0).position);
    points.push_back(Geometry::tfPose(pose_, 0, y_dim_, 0, 0).position);
    points.push_back(Geometry::tfPose(pose_, 0, y_dim_, z_dim_, 0).position);
    points.push_back(Geometry::tfPose(pose_, x_dim_, y_dim_, z_dim_, 0).position);
    points.push_back(Geometry::tfPose(pose_, x_dim_, y_dim_, 0, 0).position);
    return points;
}

std::vector<Pose> Brick::getCornerPoses()
{
    std::vector<Pose> poses;
    // pushes back each corner pose by transforming from the brick pose by the dimensions
    poses.push_back(Geometry::tfPose(pose_, 0, 0, 0, 0));
    poses.push_back(Geometry::tfPose(pose_, 0, 0, z_dim_, 0));
    poses.push_back(Geometry::tfPose(pose_, x_dim_, 0, z_dim_, 0));
    poses.push_back(Geometry::tfPose(pose_, x_dim_, 0, 0, 0));
    poses.push_back(Geometry::tfPose(pose_, 0, y_dim_, 0, 0));
    poses.push_back(Geometry::tfPose(pose_, 0, y_dim_, z_dim_, 0));
    poses.push_back(Geometry::tfPose(pose_, x_dim_, y_dim_, z_dim_, 0));
    poses.push_back(Geometry::tfPose(pose_, x_dim_, y_dim_, 0, 0));
    return poses;
}

bool Brick::pointInBrick(Point point)
{
    Pose temp_pose;
    temp_pose.position = point;
    Pose relative_pose = Geometry::relativePose(getCentrePose(), temp_pose);
    if(fabs(relative_pose.position.x) < getXDim()/2 + EPSILION &&
       fabs(relative_pose.position.y) < getYDim()/2 + EPSILION &&
       fabs(relative_pose.position.z) < getZDim()/2 + EPSILION ) return true;
    return false;
}

Pose Brick::getRelativeBrickPose(Brick brick)
{
    return Geometry::relativePose(getPose(), brick.getPose());
}

Pose Brick::getPoseFromCornerPose(unsigned corner_index, Pose pose)
{
    switch(corner_index) {
        case 0: return pose;
        case 1: return Geometry::tfPose(pose, 0, 0, -z_dim_, 0);
        case 2: return Geometry::tfPose(pose, -x_dim_, 0, -z_dim_, 0);
        case 3: return Geometry::tfPose(pose, -x_dim_, 0, 0, 0);
        case 4: return Geometry::tfPose(pose, 0, -y_dim_, 0, 0);
        case 5: return Geometry::tfPose(pose, 0, -y_dim_, -z_dim_, 0);
        case 6: return Geometry::tfPose(pose, -x_dim_, -y_dim_, -z_dim_, 0);
        case 7: return Geometry::tfPose(pose, -x_dim_, -y_dim_, 0, 0);
    }
}

Pose Brick::getCentreTopFacePose() {
    return Geometry::tfPose(pose_, x_dim_/2, -y_dim_/2, z_dim_, 0);
}

//////// Brick Type Variable Declarations

RedBrick::RedBrick(Pose pose)
    :Brick(x_dim, y_dim, z_dim, r, g, b, pose, color){}
const std::string RedBrick::color = mrc_msgs::Brick::RED;
const double RedBrick::x_dim = 0.3;
const double RedBrick::y_dim = 0.2;
const double RedBrick::z_dim = 0.2;
const uint8_t RedBrick::r = 255;
const uint8_t RedBrick::g = 0;
const uint8_t RedBrick::b = 0;

GreenBrick::GreenBrick(Pose pose)
    :Brick(x_dim, y_dim, z_dim, r, g, b, pose, color)  {}
const std::string GreenBrick::color = mrc_msgs::Brick::GREEN;
const double GreenBrick::x_dim = 0.6;
const double GreenBrick::y_dim = 0.2;
const double GreenBrick::z_dim = 0.2;
const uint8_t GreenBrick::r = 0;
const uint8_t GreenBrick::g = 255;
const uint8_t GreenBrick::b = 0;

BlueBrick::BlueBrick(Pose pose)
    :Brick(x_dim, y_dim, z_dim, r, g, b, pose, color)  {}
const std::string BlueBrick::color = mrc_msgs::Brick::BLUE;
const double BlueBrick::x_dim = 1.2;
const double BlueBrick::y_dim = 0.2;
const double BlueBrick::z_dim = 0.2;
const uint8_t BlueBrick::r = 0;
const uint8_t BlueBrick::g = 0;
const uint8_t BlueBrick::b = 255;

OrangeBrick::OrangeBrick(Pose pose)
    :Brick(x_dim, y_dim, z_dim, r, g, b, pose, color) {}
const std::string OrangeBrick::color = mrc_msgs::Brick::ORANGE;
const double OrangeBrick::x_dim = 1.8;
const double OrangeBrick::y_dim = 0.2;
const double OrangeBrick::z_dim = 0.2;
const uint8_t OrangeBrick::r = 255;
const uint8_t OrangeBrick::g = 165;
const uint8_t OrangeBrick::b = 0;
