#include "brick_structure_command/brick.h"

using namespace brick_structure_command;
//////// Brick Class Methods
Brick::Brick() {}

Brick::Brick(double x_dim, double y_dim, double z_dim,
             uint8_t r, uint8_t g, uint8_t b, geometry_msgs::Pose pose)
             :x_dim_(x_dim), y_dim_(y_dim), z_dim_(z_dim),
             r_(r), g_(g), b_(b), pose_(pose)
{}

Brick::Brick(char colour, geometry_msgs::Pose pose)
    :pose_(pose)
{
    switch (colour) {
        case 'R':
            x_dim_ = RedBrick::x_dim;
            y_dim_ = RedBrick::y_dim;
            z_dim_ = RedBrick::z_dim;
            r_ = RedBrick::r;
            g_ = RedBrick::g;
            b_ = RedBrick::b;
            break;
        case 'G':
            x_dim_ = GreenBrick::x_dim;
            y_dim_ = GreenBrick::y_dim;
            z_dim_ = GreenBrick::z_dim;
            r_ = GreenBrick::r;
            g_ = GreenBrick::g;
            b_ = GreenBrick::b;
            break;
        case 'B':
            x_dim_ = BlueBrick::x_dim;
            y_dim_ = BlueBrick::y_dim;
            z_dim_ = BlueBrick::z_dim;
            r_ = BlueBrick::r;
            g_ = BlueBrick::g;
            b_ = BlueBrick::b;
            break;
        case 'O':
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
geometry_msgs::Pose Brick::getPose() const
{ return pose_; }
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

//////// Brick SETTERS
void Brick::setPose(geometry_msgs::Pose pose)
{ pose_ = pose; }

//////// Brick Methods
std::vector<geometry_msgs::Point> Brick::getPoints(bool top, double interval)
{
    std::vector<geometry_msgs::Point> points;
    int i = (1 * top) + (-1 * !top);
    geometry_msgs::Point p1 = Transforms::tfPose
    (getPose(), -getXDim()/2, 0, i*getZDim()/2, 0).position;
    geometry_msgs::Point p2 = Transforms::tfPose
    (getPose(), getXDim()/2, 0, i*getZDim()/2, 0).position;
    double x_min = std::min(p1.x, p2.x);
    double x_max = std::max(p1.x, p2.x);
    for(double x = x_min; x < x_max; x = x + interval)
    {
        geometry_msgs::Point point;
        point.x = x;
        point.z = p1.z;
        point.y = p1.y + (((x - p1.x) * (p2.y - p1.y)) /(p2.x - p1.x));
        points.push_back(point);
    }
    return points;
}

//////// Brick Type Variable Declarations

RedBrick::RedBrick(geometry_msgs::Pose pose)
    :Brick(x_dim, y_dim, z_dim, r, g, b, pose){}
const double RedBrick::x_dim = 0.3;
const double RedBrick::y_dim = 0.2;
const double RedBrick::z_dim = 0.2;
const uint8_t RedBrick::r = 255;
const uint8_t RedBrick::g = 0;
const uint8_t RedBrick::b = 0;

GreenBrick::GreenBrick(geometry_msgs::Pose pose)
    :Brick(x_dim, y_dim, z_dim, r, g, b, pose)  {}
const double GreenBrick::x_dim = 0.6;
const double GreenBrick::y_dim = 0.2;
const double GreenBrick::z_dim = 0.2;
const uint8_t GreenBrick::r = 0;
const uint8_t GreenBrick::g = 255;
const uint8_t GreenBrick::b = 0;

BlueBrick::BlueBrick(geometry_msgs::Pose pose)
    :Brick(x_dim, y_dim, z_dim, r, g, b, pose)  {}
const double BlueBrick::x_dim = 1.2;
const double BlueBrick::y_dim = 0.2;
const double BlueBrick::z_dim = 0.2;
const uint8_t BlueBrick::r = 0;
const uint8_t BlueBrick::g = 0;
const uint8_t BlueBrick::b = 255;

OrangeBrick::OrangeBrick(geometry_msgs::Pose pose)
    :Brick(x_dim, y_dim, z_dim, r, g, b, pose) {}
const double OrangeBrick::x_dim = 1.8;
const double OrangeBrick::y_dim = 0.2;
const double OrangeBrick::z_dim = 0.2;
const uint8_t OrangeBrick::r = 255;
const uint8_t OrangeBrick::g = 165;
const uint8_t OrangeBrick::b = 0;
