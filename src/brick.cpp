#include "brick_structure_command/brick.h"

//////// Brick Class Methods 

template<class BrickType>
Brick<BrickType>::Brick() {}

template<class BrickType>
double Brick<BrickType>::getXDim() const { return BrickType::x_dim; }

template<class BrickType>
double Brick<BrickType>::getYDim() const { return BrickType::y_dim; }

template<class BrickType>
double Brick<BrickType>::getZDim() const { return BrickType::z_dim; }

template<class BrickType>
uint8_t Brick<BrickType>::getR() const { return BrickType::r; }

template<class BrickType>
uint8_t Brick<BrickType>::getG() const { return BrickType::g; }

template<class BrickType>
uint8_t Brick<BrickType>::getB() const { return BrickType::b; }

//////// Brick Type Variable Declarations

const double RedBrick::x_dim = 0.3;
const double RedBrick::y_dim = 0.2;
const double RedBrick::z_dim = 0.2;
const uint8_t RedBrick::r = 255;
const uint8_t RedBrick::g = 0;
const uint8_t RedBrick::b = 0;

const double GreenBrick::x_dim = 0.6;
const double GreenBrick::y_dim = 0.2;
const double GreenBrick::z_dim = 0.2;
const uint8_t GreenBrick::r = 0;
const uint8_t GreenBrick::g = 255;
const uint8_t GreenBrick::b = 0;

const double BlueBrick::x_dim = 1.2;
const double BlueBrick::y_dim = 0.2;
const double BlueBrick::z_dim = 0.2;
const uint8_t BlueBrick::r = 0;
const uint8_t BlueBrick::g = 0;
const uint8_t BlueBrick::b = 255;

const double OrangeBrick::x_dim = 1.8;
const double OrangeBrick::y_dim = 0.2;
const double OrangeBrick::z_dim = 0.2;
const uint8_t OrangeBrick::r = 255;
const uint8_t OrangeBrick::g = 165;
const uint8_t OrangeBrick::b = 0;
