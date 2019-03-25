#include "ros/ros.h"

#include <sstream>
#include <iostream>
#include <string>

#include "brick_command/brick.h"

using namespace brick_command;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "brick_param_node");

  ros::NodeHandle n;
  //Red Brick parameters
  n.setParam("red_x_dim", std::to_string(RedBrick::x_dim));
  n.setParam("red_y_dim", std::to_string(RedBrick::y_dim));
  n.setParam("red_z_dim", std::to_string(RedBrick::z_dim));
  n.setParam("red_colour_code", RedBrick::colour);
  //Green Brick parameters
  n.setParam("green_x_dim", std::to_string(GreenBrick::x_dim));
  n.setParam("green_y_dim", std::to_string(GreenBrick::y_dim));
  n.setParam("green_z_dim", std::to_string(GreenBrick::z_dim));
  n.setParam("green_colour_code", GreenBrick::colour);
  //Blue Brick parameters
  n.setParam("blue_x_dim", std::to_string(BlueBrick::x_dim));
  n.setParam("blue_y_dim", std::to_string(BlueBrick::y_dim));
  n.setParam("blue_z_dim", std::to_string(BlueBrick::z_dim));
  n.setParam("blue_colour_code", BlueBrick::colour);
  //Orange Brick parameters
  n.setParam("orange_x_dim", std::to_string(OrangeBrick::x_dim));
  n.setParam("orange_y_dim", std::to_string(OrangeBrick::y_dim));
  n.setParam("orange_z_dim", std::to_string(OrangeBrick::z_dim));
  n.setParam("orange_colour_code", OrangeBrick::colour);

  return 0;
}
