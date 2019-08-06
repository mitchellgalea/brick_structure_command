#ifndef BRICKSTRUCTURE_H
#define BRICKSTRUCTURE_H

#include <vector>
#include <memory>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <math.h>
#include <iostream>
#include <utility>
#include <fstream>
#include <chrono>
#include <random>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"

#include "ros/ros.h"
#include "brick_structure/brick.h"
#include "mrc_msgs/Brick.h"
#include "mrc_msgs/BrickCommand.h"
#include "brick_command/BrickStructureLines.h"
#include "brick_structure/geometry.h"
#include "mrc_msgs/BrickPile.h"



namespace brick_structure {

//!  Brick Structure Class contains a structure of bricks
/*!
  Brick Class contains a vector of bricks which are ordered in building order
  This is so that the sequence of structure construction can be kept
*/
class BrickStructure {
private:
    //////// PRIVATE MEMBERS
    std::vector<Brick> bricks_; /*!< vector containing the bricks */
    unsigned c_brick_count_;    /*!< current count of bricks built */
    double layer_height_;       /*!< layer height of bricks (m)*/
    unsigned layers_;           /*!< layer count of structure */
    double point_interval_;     /*!< member used for calculations */
    double brick_spacing_;      /*!< member used for brick spacings(m)*/
    std::vector<Plane> planes_; /*!< vector for containing the different brick planes*/

    //////// BOOLEANS
    bool initialized_;          /*!< bool for initialization */

    //////// PRIVATE METHODS

    //! Method for parsing a yaml file
      /*!
        will set up the brick structure from the blueprint
        \param path of the blueprint file
      */
    bool parseYAML(std::string path);

    //! Method to check whether all bricks are the same height
    bool checkBrickHeights();

    //! Methods that checks the constructions
      /*!
        Checks whether the construction is possible, for instance a brick
        must be supported by the floor or another brick to be
        constructed
      */
    bool checkConstruction();

    //! Checks whether a brick can be placed in the position
    bool checkBrickPlacement(Brick brick, int brick_count);

    //! Methods that gets the layer of a brick
      /*!
        Returns the layer number of a brick
        \param brick
      */
    unsigned getLayer(Brick brick);

    //! Method that returns points of all bricks in a layer to check for construction
    std::vector<geometry_msgs::Point> getLayerPoints(unsigned layer, int brick_count = -1);

    //! Methods that returns gets the bricks within a layer
      /*!
        Returns the bricks within a layer
        \param layer number
        \param brick_count c_brick_count_ in which the layer would have the bricks in
      */
    std::vector<Brick> getBricksinLayer(unsigned layer, int brick_count = -1);

    std::vector<unsigned> getBrickIndexsinLayer(unsigned layer, int brick_count = -1);

    //! Methods that returns gets the bricks within a layer
    /*!
      Returns the bricks within a layer
      \param layer number
      \param bricks gets the bricks from this vector that are in the layer
    */
    std::vector<Brick> getBricksinLayer(unsigned layer, std::vector<Brick> bricks);

    //! Methods that returns gets the bricks within a Plane
    /*!
      Returns the bricks within a plane
      \param layer number
      \param brick_count c_brick_count_ in which the layer would have the bricks in
    */
    std::vector<Brick> getBricksinPlane(unsigned plane, int brick_count = -1);

    std::vector<unsigned> getBrickIndexsinPlane(unsigned plane, std::vector<unsigned> brick_indexs);

    //! Methods that returns gets the bricks within a plane
    /*!
      Returns the bricks within a Plane
      \param layer number
      \param bricks gets the bricks from this vector that are in the plane
    */
    std::vector<Brick> getBricksinPlane(unsigned plane, std::vector<Brick> bricks);

    //! Methods that returns the plane index for a certain brick
    /*!
      Returns the plane index of a brick
      \param brick_index of the brick
    */
    unsigned getBrickPlaneIndex(bool front, int brick_index = -1);

    //! Method for internal use
    double pointDistance(geometry_msgs::Point p1, geometry_msgs::Point p2);

    //! Methods that returns the brick index of an adjacent brick
      /*!
        \param brick input
      */
    int adjacentBrickIndex(double max_dist, bool front, std::vector<unsigned> &control_corner_idxs, unsigned brick_index, unsigned brick_count);

    //! Methods that returns Bricks that are adjacent to a brick
    /*!
      \param brick input
    */
    int lowerLineBrickIndex(double max_dist, bool front, std::vector<unsigned> &control_corner_idxs, unsigned brick_index, unsigned brick_count);

    //! Methods that returns Bricks that are below a brick
      /*!
        \param brick input
      */
     std::vector<Brick> downBricks(unsigned brick_index, unsigned brick_count);

    bool pointInBrickStructure(geometry_msgs::Point point, unsigned brick_count = -1);

    //! Sets Brick Control Scenario
    /*!

     */
    void setBrickCommandControl(double max_dist, mrc_msgs::BrickCommand &brick_command, Pose placement_offset,
                           std::vector<mrc_msgs::BrickPile> brick_piles, int brick_index = -1);

    Pose getBrickPlacementPose(int brick_index, Pose placement_offset, Pose pile_pose);

    Pose getBrickPlacementPose(int control_brick_index, int brick_index, Pose placement_offset, Pose pile_pose);

    Pose getBrickPlacementPose(int &index, std::vector<int> &control_brick_indexs,
                               std::vector<std::vector<unsigned>> control_corner_idxs,
                               int brick_index, Pose placement_offset, Pose pile_pose);
public:
    //////// CONSTRUCTORS
    //! Default BrickStructure Constructor
    /*!
      Brick Constructor that creates a brick object
    */
    BrickStructure();

    //! BrickStructure Constructor that uses a blueprint file
    /*!
      Brick Constructor that creates a brick object
    */
    BrickStructure(std::string path, double point_interval, int brick_count = 0);

    //////// GETTERS

    //! Getter for c_brick_count_
    unsigned getCBrickCount() const;

    //! Getter for bricks_ size
    unsigned getBricksSize() const;

    //! Getter for initialized_
    bool initialized() const;

    //! Gets a brick from an index
    Brick getBrick(unsigned brick_index);

    //////// METHODS

    //! Method to increment c_brick_count_
    void incCBrickCount();

    //! method to check if the structure is complete
    bool structureComplete();

    //! Method to get the next brick command
    /*!
      returns a BrickCommand ROS msg
      \param increment if true c_brick_count_ will be incremented
    */
    mrc_msgs::BrickCommand getCBrickCommand(double max_dist, Pose placement_offset,
                                                 std::vector<mrc_msgs::BrickPile> brick_piles, bool increment = true);

    //! Method to get the next brick command
    /*!
      returns a BrickCommand ROS msg
      \param increment if true c_brick_count_ will be incremented
      \param brick_poses a vector of the brick poses in a certain frame
    */
    mrc_msgs::BrickCommand getCBrickCommand(double max_dist, Pose placement_offset, std::vector<PoseStamped> brick_poses,
                                                 std::vector<mrc_msgs::BrickPile> brick_piles, bool increment = true);

    //! Method to output Brick Structure as a Gazebo World
    /*!
      \param myfile will add the gazebo world to that
      \param brick_count the structure will be built to that level
    */
    void gazeboLaunchOut(std::ofstream &myfile, unsigned brick_count);
    void gazeboLaunchOut(std::ofstream &myfile, unsigned brick_count, double x, double y, double z, double yaw);

    //! Method to output all the corner points of every brick in the structure for perception
    /*!
      \param brick_count the structure will be built to that level
    */
    std::vector<geometry_msgs::Point> getBrickCornerPoints(int brick_count = -1, int plane = -1);

    //! Method to output all the corner points of every brick in the structure for perception
    /*!
      \param brick_count the structure will be built to that level
    */
    std::vector<geometry_msgs::Pose> getBrickCornerPoses(int brick_count = -1, int plane = -1);

    //! Method to output all the corner points of every brick in the structure for perception
    /*!
      \param brick_count the structure will be built to that level
    */
    std::vector<geometry_msgs::Point> getStructureCornerPoints(int brick_count = -1, int plane = -1);

    //! Method to output all the brick lines of the structure for perception
    /*!
      \param brick_count the structure will be built to that level
      \param plane the plane in which the lines should be extracted from
    */

    std::vector<geometry_msgs::Pose> getStructureCornerPoses(int brick_count = -1, int plane = -1);

    //! Method to output all the brick lines of the structure for perception
    /*!
      \param brick_count the structure will be built to that level
      \param plane the plane in which the lines should be extracted from
    */
    std::vector<Line> getStructureLines(int plane, int brick_count = -1);

    //! Method to output all the brick lines of the structure in the format of msg
    /*!
      \param brick_count the structure will be built to that level
      \param plane the plane in which the lines should be extracted from
    */
    brick_command::BrickStructureLines getStructureLinesMsg(int plane, int brick_count = -1);

    //! Method to transform all brick Poses to match a new pose of a brick
    /*!
      \param brick_index is the index of the brick pose which is inputted
      \param corner_index is the index of the corner of the pose
      \param pose is the pose to be used to transform
    */
    void updatePoses(unsigned brick_index, unsigned corner_index, Pose pose);

    //! Method to get all the poses in a different frame
    /*!
      \param brick_index is the index of the brick pose which is inputted
      \param corner_index is the index of the corner of the pose
      \param pose is the pose to be used to transform
    */
    std::vector<Pose> getRelativePosesTopFaceCentre(unsigned brick_index, unsigned corner_index, Pose pose);
    //! Method to get all the poses in a different frame of the top centre face
    /*!
      \param brick_index is the index of the brick pose which is inputted
      \param corner_index is the index of the corner of the pose
      \param pose is the pose to be used to transform
    */
    std::vector<Pose> getRelativePoses(unsigned brick_index, unsigned corner_index, Pose pose);


    //! Method to get all the poses of the bricks in a pose array
    PoseArray getBrickPoses(int brick_count = -1);

    void addBrickNoise(double noise);

    //! Print Function
    void print();

};

}


#endif // BRICKSTRUCTURE_H
