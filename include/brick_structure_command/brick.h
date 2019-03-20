#ifndef BRICK_H
#define  BRICK_H

#include <stdint.h>

class B{};

template<class BrickType>
class Brick : public B
{
public:
    Brick();
    double getXDim() const;
    double getYDim() const;
    double getZDim() const;
    uint8_t getR() const;
    uint8_t getG() const;
    uint8_t getB() const;
};

class RedBrick {
public:
    static const double x_dim;
    static const double y_dim;
    static const double z_dim;
    static const uint8_t r;
    static const uint8_t g;
    static const uint8_t b;
};

class GreenBrick {
public:
    static const double x_dim;
    static const double y_dim;
    static const double z_dim;
    static const uint8_t r;
    static const uint8_t g;
    static const uint8_t b;
};

class BlueBrick {
public:
    static const double x_dim;
    static const double y_dim;
    static const double z_dim;
    static const uint8_t r;
    static const uint8_t g;
    static const uint8_t b;
};

class OrangeBrick {
public:
    static const double x_dim;
    static const double y_dim;
    static const double z_dim;
    static const uint8_t r;
    static const uint8_t g;
    static const uint8_t b;
};

template class Brick<RedBrick>;
template class Brick<GreenBrick>;
template class Brick<BlueBrick>;
template class Brick<OrangeBrick>;

#endif //BRICK_H
