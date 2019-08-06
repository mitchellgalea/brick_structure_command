#include <iostream>
#include <string>
#include <fstream>
#include <boost/filesystem.hpp>
#include <iomanip>
#include <thread>

#include "ros/ros.h"
#include "ros/package.h"
#include "geometry_msgs/PoseArray.h"

#include "brick_structure/geometry.h"
#include "brick_structure/brick_structure.h"
#include "brick_structure/geometry.h"
#include "brick_command/BrickStructureLines.h"
#include "brick_command/StructurePoseEst.h"

using namespace brick_structure;

class BrickCommandNode
{
private:
    ros::NodeHandle nh_;
public:
    BrickCommandNode(ros::NodeHandle nh)
        :nh_(nh)
    {
        //input arguments
        std::string blueprint_name;
        std::string yaml_name;
        double point_interval;
        int brick_count;
        int plane;
        bool robot, brick_noise;
        double x, y, z, yaw, noise_std_dev;
        ros::NodeHandle pn("~");
        pn.param<std::string>("blueprint_name", blueprint_name, "test");
        pn.param<double>("point_interval", point_interval, 0.001);
        pn.param<int>("brick_count", brick_count, 0);
        pn.param<bool>("robot", robot, false);
        pn.param<bool>("brick_noise", brick_noise, false);
        pn.param<double>("x", x, 0.0);
        pn.param<double>("y", y, 0.0);
        pn.param<double>("z", z, 0.0);
        pn.param<double>("yaw", yaw, 1.5708);
        pn.param<double>("noise_std_dev", noise_std_dev, 0.01);

        //gets paths to launch and blueprints
        std::string launch_path = ros::package::getPath("brick_command");
        std::string blueprint_path = launch_path;
        blueprint_path.append("/blueprints/");
        blueprint_path.append(blueprint_name);
        blueprint_path.append(".yaml");
        launch_path.append("/launch/");
        launch_path.append(blueprint_name);
        launch_path.append(".launch");

        //initialises brick_structure
        BrickStructure brick_structure = brick_structure::BrickStructure(blueprint_path, point_interval);
        ROS_INFO("Brick Structure Initialized");
        if(brick_noise){
            brick_structure.addBrickNoise(noise_std_dev);
            ROS_INFO("Brick Noise Added");
        }

        std::ofstream myfile;
        myfile.open(launch_path);
        if(robot)brick_structure.gazeboLaunchOut(myfile, brick_count, x, y, z, yaw);
        else brick_structure.gazeboLaunchOut(myfile, brick_count);
        myfile.close();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "brick_structure_gazebo_launch_gen");

    ros::NodeHandle n;

    BrickCommandNode brick_command_node(n);

    ros::shutdown();


  return 0;
}
