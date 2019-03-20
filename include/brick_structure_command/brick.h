#ifndef BRICK_H
#define  BRICK_H

class Brick {
private:
    double x_, y_, z_;
public:
    Brick();

    double getX() const;
    double getY() const;
    double getZ() const;

};

#endif //BRICK_H
