#include <iostream>
#include <string>

#include "ros/ros.h"

#include "brick_command/brick_structure.h"
#include "brick_command/RequestBrickCommand.h"
#include "brick_command/BrickCommand.h"

class BrickCommandNode
{
private:
    ros::NodeHandle nh_;
    ros::ServiceServer req_brick_command_;

    brick_command::BrickStructure brick_structure_;
public:
    BrickCommandNode(ros::NodeHandle nh)
        :nh_(nh)
    {
        req_brick_command_ = nh_.advertiseService("request_brick_command",
                                &BrickCommandNode::requestBrickCommand, this);

        std::string blueprint_path;
        double point_interval;
        ros::NodeHandle pn("~");
        pn.param<std::string>("blueprint_path", blueprint_path, "");
        pn.param<double>("point_interval", point_interval, 0.001);

        brick_structure_ = brick_command::BrickStructure(blueprint_path, point_interval);

    }
    bool requestBrickCommand(brick_command::RequestBrickCommand::Request &req,
                             brick_command::RequestBrickCommand::Response &res)
    {
        brick_command::BrickCommand brick_command = brick_structure_.getCBrickCommand( req.increment);
        res.brick_command = brick_command;
        return true;
    }
};

int main(int argc, char **argv)
{
    chdir("/");
    ros::init(argc, argv, "brick_command_node");

    ros::NodeHandle n;

    BrickCommandNode brick_command_node(n);

    ros::spin();

  return 0;
}
