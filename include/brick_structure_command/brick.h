#ifndef BRICK_H
#define  BRICK_H

template<struct BrickType>
class Brick {
public:
    Brick()
    double getXDim() const;
    double getYDim() const;
    double getZDim() const;
    uint8_t getR() const;
    uint8_t getG() const;
    uint8_t getB() const;
};

struct RedBrick {
    static const double x_dim;
    static const double y_dim;
    static const double z_dim;
    static const uint8_t r;
    static const uint8_t g;
    static const uint8_t b;
};

struct GreenBrick {
    static const double x_dim_;
    static const double y_dim_;
    static const double z_dim_;
    static const uint8_t r;
    static const uint8_t g;
    static const uint8_t b;
};

struct BlueBrick {
    static const double x_dim_;
    static const double y_dim_;
    static const double z_dim_;
    static const uint8_t r;
    static const uint8_t g;
    static const uint8_t b;
};

struct OrangeBrick {
    static const double x_dim_;
    static const double y_dim_;
    static const double z_dim_;
    static const uint8_t r;
    static const uint8_t g;
    static const uint8_t b;
};

#endif //BRICK_H
