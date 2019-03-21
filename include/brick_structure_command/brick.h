#ifndef BRICK_H
#define  BRICK_H

#include <stdint.h>
#include <string>
#include <geometry_msgs/Pose.h>

namespace brick_structure_command {

class Brick {
protected:
    geometry_msgs::Pose pose_;
    double x_dim_;
    double y_dim_;

    double z_dim_;
    uint8_t r_;
    uint8_t g_;
    uint8_t b_;
public:
    Brick(double x_dim, double y_dim, double z_dim, uint8_t r, uint8_t g, uint8_t b);
    virtual ~Brick() = 0;

    geometry_msgs::Pose getPose() const;
    double getXDim() const;
    double getYDim() const;
    double getZDim() const;
    uint8_t getR() const;
    uint8_t getG() const;
    uint8_t getB() const;
};

class RedBrick : public Brick
{
public:
    RedBrick();
private:
    static const double x_dim;
    static const double y_dim;
    static const double z_dim;
    static const uint8_t r;
    static const uint8_t g;
    static const uint8_t b;
};

class GreenBrick : public Brick
{
public:
    GreenBrick();
private:
    static const double x_dim;
    static const double y_dim;
    static const double z_dim;
    static const uint8_t r;
    static const uint8_t g;
    static const uint8_t b;
};

class BlueBrick : public Brick
{
public:
    BlueBrick();
private:
    static const double x_dim;
    static const double y_dim;
    static const double z_dim;
    static const uint8_t r;
    static const uint8_t g;
    static const uint8_t b;
};

class OrangeBrick : public Brick
{
public:
    OrangeBrick();
private:
    static const double x_dim;
    static const double y_dim;
    static const double z_dim;
    static const uint8_t r;
    static const uint8_t g;
    static const uint8_t b;
};

}

#endif //BRICK_H

// C Flags
// W -Wall -Wextra
// C++ Flags
// -pedantic -std=c++11 -c -fsyntax-only /dev/null
// GCC Include Paths
// /opt/ros/kinetic/include,/opt/ros/kinetic/lib,/home/mitchell_galea/catkin_ws/devel/include,./include
// cmake
// alias cm='cd ~/catkin_ws && catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1'
