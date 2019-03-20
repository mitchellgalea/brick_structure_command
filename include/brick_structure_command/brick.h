#ifndef BRICK_H
#define  BRICK_H

#include <stdint.h>
#include <string>

class Brick {
protected:
    double x_dim_;
    double y_dim_;
    double z_dim_;
    uint8_t r_;
    uint8_t g_;
    uint8_t b_;
public:
    Brick(double x_dim, double y_dim, double z_dim, uint8_t r, uint8_t g, uint8_t b);
    virtual ~Brick() = 0;

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

#endif //BRICK_H
