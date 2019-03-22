#ifndef BRICKSTRUCTURE_H
#define BRICKSTRUCTURE_H

#include <vector>
#include <memory>
#include <yaml-cpp/yaml.h>
#include <math.h>
//#include <eigen3/Eigen/Sparse>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"

#include "brick_structure_command/brick.h"
#include "brick_structure_command/BrickCommand.h"


namespace brick_structure_command {

class BrickStructure {
private:
    //////// PRIVATE MEMBERS
    std::vector<std::shared_ptr<Brick>> bricks_;
    unsigned c_brick_count_;

    //////// PRIVATE METHODS
    bool checkBrickHeights();
    bool checkConstruction();
    bool checkPosition();
public:
    //////// CONSTRUCTORS
    BrickStructure(std::string path);

    //////// GETTERS
    unsigned getCBrickCount() const;

    //////// METHODS
    void incCBrickCount();
    brick_structure_command::BrickCommand brickCommand();
};

}


#endif // BRICKSTRUCTURE_H
