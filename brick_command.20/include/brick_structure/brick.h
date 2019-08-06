#ifndef BRICK_H
#define  BRICK_H

#include <stdint.h>
#include <string>
#include <vector>
#include <math.h>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "mrc_msgs/Brick.h"
#include "ros/ros.h"

#include "brick_structure/geometry.h"
#define EPSILION 0.0001

namespace brick_structure {

//!  Brick Class contains Pose and color of Brick.
/*!
  Brick Class contains Dimensions, Pose and color of Bricks. Used by Brick Structure
*/

class Brick {
protected:
    //////// PRIVATE MEMBERS
    Pose pose_;  /*!< Pose of bottom left vertice of Brick */
    std::string color_;        /*!< color of brick, single letter - ie red = R */
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
      \param color Single Letter color Code ie red = R, blue = B
    */
    Brick(double x_dim, double y_dim, double z_dim, uint8_t r, uint8_t g, uint8_t b,
          Pose pose, std::string color);

    //! color Brick Constructor
    /*!
      Brick Constructor that takes as a color as argument and sets variables based on static values
      \param color Single Letter color Code ie red = R, blue = B
      \param pose Pose of BRICK
    */
    Brick(char color, Pose pose = Pose());

    //////// GETTERS

    //! Getter for Pose
    Pose getPose() const;

    //! Getter for Pose
    Pose getCentrePose();

    //! Getter for pose_
    std::string getColor() const;

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

    //! Gets the Brick as a ros Brick
    mrc_msgs::Brick getBrickMsg() const;

    //////// SETTERS
    void setPose(Pose pose);

    //////// METHODS

    //! Gets Points along a Surface of Brick to check construction
    /*!
      Function will get the 3D point along the centre y axis of the brick of either the top or bottom face at an interval
      \param top True if top, false if bottom
      \param interval distance in between points in m
    */
    std::vector<Point> getPoints(bool top, double interval);

    //! Gets the Corner points of the Brick
    std::vector<Point> getCornerPoints();

    //! Gets the Corner poses of the Brick
    std::vector<Pose> getCornerPoses();

    //! Function to Check whether a point is within the brick
    bool pointInBrick(Point point);

    //! Gets relative Pose of another brick
    /*!
      Calculates the relative pose from this brick to another brick
      \param brick Brick in which pose is being found
    */
    Pose getRelativeBrickPose(Brick brick);

    //! Gets the relative Brick Pose of an inputted corner and pose
    /*!
      \param corner_index the corner of the pose
      \param pose the pose
    */
    Pose getPoseFromCornerPose(unsigned corner_index, Pose pose);

    //! Gets the relative Brick Pose of an inputted corner and pose
    /*!
      \param corner_index the corner of the pose
      \param pose the pose
    */
    Pose getCentreTopFacePose();


};

//!  Red Brick is a class that contains static const dimensions and colors of Red Bricks
class RedBrick : public Brick
{
public:
    RedBrick(Pose pose = Pose());
    static const std::string color;
    static const double x_dim;
    static const double y_dim;
    static const double z_dim;
    static const uint8_t r;
    static const uint8_t g;
    static const uint8_t b;
};

//!  Green Brick is a class that contains static const dimensions and colors of Green Bricks
class GreenBrick : public Brick
{
public:
    GreenBrick(Pose pose  = Pose());
    static const std::string color;
    static const double x_dim;
    static const double y_dim;
    static const double z_dim;
    static const uint8_t r;
    static const uint8_t g;
    static const uint8_t b;
};

//!  Blue Brick is a class that contains static const dimensions and colors of Blue Bricks
class BlueBrick : public Brick
{
public:
    BlueBrick(Pose pose  = Pose());
    static const std::string color;
    static const double x_dim;
    static const double y_dim;
    static const double z_dim;
    static const uint8_t r;
    static const uint8_t g;
    static const uint8_t b;
};

//!  Orange Brick is a class that contains static const dimensions and colors of Orange Bricks
class OrangeBrick : public Brick
{
public:
    OrangeBrick(Pose pose  = Pose());
    static const std::string color;
    static const double x_dim;
    static const double y_dim;
    static const double z_dim;
    static const uint8_t r;
    static const uint8_t g;
    static const uint8_t b;
};

}

#endif //BRICK_H
