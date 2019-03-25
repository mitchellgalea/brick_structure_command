#ifndef BRICK_H
#define  BRICK_H

#include <stdint.h>
#include <string>
#include <vector>
#include <math.h>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "ros/ros.h"

#include "brick_command/transforms.h"

namespace brick_command {

//!  Brick Class contains Pose and Colour of Brick.
/*!
  Brick Class contains Dimensions, Pose and Colour of Bricks. Used by Brick Structure
*/

class Brick {
protected:
    //////// PRIVATE MEMBERS
    geometry_msgs::Pose pose_;  /*!< Pose of Brick */
    std::string colour_;        /*!< Colour of brick, single letter - ie red = R */
    double x_dim_;              /*!< X dimension of brick in m */
    double y_dim_;              /*!< Y dimension of brick in m. */
    double z_dim_;              /*!< Z dimension of brick in m */
    uint8_t r_;                 /*!< Red RGB Value */
    uint8_t g_;                 /*!< Green RGB Value */
    uint8_t b_;                 /*!< Blue RGB Value */
public:
    //////// CONSTRUCTORS

    //! Default Brick Constructor
    /*!
      Brick Constructor that creates a brick object
    */
    Brick();

    //! Parameter Brick Constructor
    /*!
      Brick Constructor that takes as arguments all members and sets them
      \param x_dim X dimension of brick in m
      \param y_dim Y dimension of brick in m
      \param z_dim Z dimension of brick in m
      \param r Red RGB pixel value
      \param g Green RGB pixel value
      \param b Blue RGB pixel value
      \param pose Pose of BRICK
      \param colour Single Letter Colour Code ie red = R, blue = B
    */
    Brick(double x_dim, double y_dim, double z_dim, uint8_t r, uint8_t g, uint8_t b,
          geometry_msgs::Pose pose, std::string colour);

    //! Colour Brick Constructor
    /*!
      Brick Constructor that takes as a Colour as argument and sets variables based on static values
      \param colour Single Letter Colour Code ie red = R, blue = B
      \param pose Pose of BRICK
    */
    Brick(char colour, geometry_msgs::Pose pose = geometry_msgs::Pose());

    //////// GETTERS

    //! Getter for Pose
    geometry_msgs::Pose getPose() const;

    //! Getter for pose_
    std::string getColour() const;

    //! Getter for x_dim_
    double getXDim() const;

    //! Getter for y_dim_
    double getYDim() const;

    //! Getter for z_dim_
    double getZDim() const;

    //! Getter for r_
    uint8_t getR() const;

    //! Getter for g_
    uint8_t getG() const;

    //! Getter for b_
    uint8_t getB() const;

    //////// SETTERS
    void setPose(geometry_msgs::Pose pose);

    //////// METHODS

    //! Gets Points along a Surface of Brick
    /*!
      Function will get the 3D point along the centre y axis of the brick of either the top or bottom face at an interval
      \param top True if top, false if bottom
      \param interval distance in between points in m
    */
    std::vector<geometry_msgs::Point> getPoints(bool top, double interval);

    //! Gets relative Pose of another brick
    /*!
      Calculates the relative pose from this brick to another brick
      \param brick Brick in which pose is being found
    */
    geometry_msgs::Pose getRelativeBrickPose(Brick brick);

};

class RedBrick : public Brick
{
public:
    RedBrick(geometry_msgs::Pose pose = geometry_msgs::Pose());
    static const std::string colour;
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
    GreenBrick(geometry_msgs::Pose pose  = geometry_msgs::Pose());
    static const std::string colour;
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
    BlueBrick(geometry_msgs::Pose pose  = geometry_msgs::Pose());
    static const std::string colour;
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
    OrangeBrick(geometry_msgs::Pose pose  = geometry_msgs::Pose());
    static const std::string colour;
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
