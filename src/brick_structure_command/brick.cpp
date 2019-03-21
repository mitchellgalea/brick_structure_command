#include "brick_structure_command/brick.h"

using namespace brick_structure_command;
//////// Brick Class Methods

Brick::Brick(double x_dim, double y_dim, double z_dim,
             uint8_t r, uint8_t g, uint8_t b)
             :x_dim_(x_dim), y_dim_(y_dim), z_dim_(z_dim),
             r_(r), g_(g), b_(b)
{}
Brick::~Brick() {}

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

//////// Brick Type Variable Declarations

RedBrick::RedBrick()
    :Brick(x_dim, y_dim, z_dim, r, g, b) {}
const double RedBrick::x_dim = 0.3;
const double RedBrick::y_dim = 0.2;
const double RedBrick::z_dim = 0.2;
const uint8_t RedBrick::r = 255;
const uint8_t RedBrick::g = 0;
const uint8_t RedBrick::b = 0;

GreenBrick::GreenBrick()
    :Brick(x_dim, y_dim, z_dim, r, g, b) {}
const double GreenBrick::x_dim = 0.6;
const double GreenBrick::y_dim = 0.2;
const double GreenBrick::z_dim = 0.2;
const uint8_t GreenBrick::r = 0;
const uint8_t GreenBrick::g = 255;
const uint8_t GreenBrick::b = 0;

BlueBrick::BlueBrick()
    :Brick(x_dim, y_dim, z_dim, r, g, b) {}
const double BlueBrick::x_dim = 1.2;
const double BlueBrick::y_dim = 0.2;
const double BlueBrick::z_dim = 0.2;
const uint8_t BlueBrick::r = 0;
const uint8_t BlueBrick::g = 0;
const uint8_t BlueBrick::b = 255;

OrangeBrick::OrangeBrick()
    :Brick(x_dim, y_dim, z_dim, r, g, b) {}
const double OrangeBrick::x_dim = 1.8;
const double OrangeBrick::y_dim = 0.2;
const double OrangeBrick::z_dim = 0.2;
const uint8_t OrangeBrick::r = 255;
const uint8_t OrangeBrick::g = 165;
const uint8_t OrangeBrick::b = 0;
