#ifndef BRICKSTRUCTURE_H
#define BRICKSTRUCTURE_H

#include <vector>
#include <memory>

#include "brick_structure_command/brick.h"
#include "brick_structure_command/BrickCommand.h"

class BrickStructure {
private:
    //////// PRIVATE MEMBERS
    std::vector<shared_ptr<B>> bricks_;
    unsigned c_brick_count_;
    
    //////// PRIVATE METHODS
    bool checkBrickHeights();
    bool checkConstruct();
    bool checkPosition();
public:
    //////// CONSTRUCTORS
    BrickStructure();
    
    //////// GETTERS
    unsigned getCBrickCount() const;
    
    //////// METHODS
    void incCBrickCount();
    BrickCommand brickCommand();
    
    
};


#endif // BRICKSTRUCTURE_H
