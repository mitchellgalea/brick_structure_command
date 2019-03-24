#include <iostream>

#include "ros/ros.h"

#include "brick_command/brick_structure.h"

class BrickCommandNode
{
private:
    ros::NodeHandle nh_;
    ros::ServiceServer req_brick_command_;

    //brick_structure_command::BrickStructure brick_structure_;
public:
    BrickCommandNode(ros::NodeHandle nh)
        :nh_(nh)
    {

    }
//     bool RequestBrickCommand()
//
//     bool requestMapDistance(pipe_sim::RequestMapDistance::Request &req,
//                         pipe_sim::RequestMapDistance::Response &res)
// {
//     if(req.req == true && pipe_map_ptr_->mapSet())
//     {
//         ROS_INFO("Map Distance Requested");
//         res.ack = true;
//         res.distance = pipe_map_ptr_->getMapDistance();
//         ROS_INFO("Sending Map Distance");
//     }
//     else
//     {
//         res.ack = false;
//     }
//     return true;
// }

};



int main(int argc, char **argv)
{
    chdir("/");
    ros::init(argc, argv, "brick_command_node");

    ros::NodeHandle n;



  return 0;
}
