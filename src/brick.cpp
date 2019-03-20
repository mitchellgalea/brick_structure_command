#include "brick_structure_command/brick.h"

Brick::Brick() {}

Brick::getXDim() const { return BrickType::x_dim; }
Brick::getYDim() const { return BrickType::y_dim; }
Brick::getZDim() const { return BrickType::z_dim; }
Brick::getR() const { return BrickType::r; }
Brick::getG() const { return BrickType::g; }
Brick::getB() const { return BrickType::b; }

constexpr double RedBrick::x_dim = 0.3;
constexpr double RedBrick::y_dim = 0.2;
constexpr double RedBrick::z_dim = 0.2;
const uint8_t RedBrick::r = 255;
const uint8_t RedBrick::g = 0;
const uint8_t RedBrick::b = 0;

constexpr double GreenBrick::x_dim = 0.6;
constexpr double GreenBrick::y_dim = 0.2;
constexpr double GreenBrick::z_dim = 0.2;
const uint8_t GreenBrick::r = 0;
const uint8_t GreenBrick::g = 255;
const uint8_t GreenBrick::b = 0;

constexpr double BlueBrick::x_dim = 1.2;
constexpr double BlueBrick::y_dim = 0.2;
constexpr double BlueBrick::z_dim = 0.2;
const uint8_t BlueBrick::r = 0;
const uint8_t BlueBrick::g = 0;
const uint8_t BlueBrick::b = 255;

constexpr double OrangeBrick::x_dim = 1.8;
constexpr double OrangeBrick::y_dim = 0.2;
constexpr double OrangeBrick::z_dim = 0.2;
const uint8_t OrangeBrick::r = ;
const uint8_t OrangeBrick::g = 255;
const uint8_t OrangeBrick::b = 0;
