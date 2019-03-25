#ifndef BRICKSTRUCTURE_H
#define BRICKSTRUCTURE_H

#include <vector>
#include <memory>
#include <yaml-cpp/yaml.h>
#include <math.h>
#include <iostream>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

#include "ros/ros.h"
#include "brick_command/brick.h"
#include "brick_command/BrickMsg.h"
#include "brick_command/BrickCommand.h"
#include "brick_command/transforms.h"


#define EPSILION 0.0001

namespace brick_command {

class BrickStructure {
private:
    //////// PRIVATE MEMBERS
    std::vector<Brick> bricks_;
    unsigned c_brick_count_;
    double layer_height_;
    unsigned layers_;
    double point_interval_;
    //////// BOOLEANS
    bool initialized_;

    //////// PRIVATE METHODS
    bool parseYAML(std::string path);
    bool checkBrickHeights();
    bool checkConstruction();
    bool checkBrickPlacement(Brick brick, int brick_count);
    unsigned getLayer(Brick brick);
    std::vector<geometry_msgs::Point> getLayerPoints(unsigned layer, int brick_count = -1);
    std::vector<Brick> getBricksinLayer(unsigned layer, int brick_count = -1);
    double pointDistance(geometry_msgs::Point p1, geometry_msgs::Point p2);
    void adjacentBricks(Brick brick, std::vector<BrickMsg> &brick_msgs, unsigned brick_count);
    void downBricks(Brick brick, std::vector<BrickMsg> &brick_msgs, unsigned brick_count);
public:
    //////// CONSTRUCTORS
    BrickStructure();
    BrickStructure(std::string path, double point_interval);

    //////// GETTERS
    unsigned getCBrickCount() const;
    bool initialized() const;

    //////// METHODS
    void incCBrickCount();
    bool structureComplete();
    BrickCommand getCBrickCommand(bool increment = false);
    void print();
    void print2Count();
};

}


#endif // BRICKSTRUCTURE_H
