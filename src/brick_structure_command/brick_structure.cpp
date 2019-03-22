#include "brick_structure_command/brick_structure.h"

using namespace brick_structure_command;

BrickStructure::BrickStructure(std::string path)
{
    YAML::Node node = YAML::LoadFile(path);
    if(node["ReferencePose"] && node["Layers"])
    {
        YAML::Node pose_node = node["ReferencePose"];
        geometry_msgs::Pose2D reference_pose_2D;
        reference_pose_2D.x = pose_node["x"].as<double>();
        reference_pose_2D.y  = pose_node["y"].as<double>();
        reference_pose_2D.theta = pose_node["yaw"].as<double>() * (M_PI/180);

        //// TODO: add yaw to quaternion

        double brick_spacing = 0.0;
        if(node["BrickSpacing"])
        {
            brick_spacing = node["BrickSpacing"].as<double>();
        }


    }
    else
    {
        //// TODO: add exception
    }
}

bool BrickStructure::checkBrickHeights()
{

}

bool BrickStructure::checkConstruction()
{

}

bool BrickStructure::checkPosition()
{

}
unsigned BrickStructure::getCBrickCount() const
{

}

void BrickStructure::incCBrickCount()
{

}

BrickCommand BrickStructure::brickCommand()
{

}
