#include "brick_structure_command/brick_structure.h"

using namespace brick_structure_command;

BrickStructure::BrickStructure(std::string path)
{
    YAML::Node node = YAML::LoadFile(path);
    if(node["ReferencePose"] && node["Layers"])
    {
        YAML::Node pose_node = node["ReferencePose"];
        geometry_msgs::Pose reference_pose;
        reference_pose.position.x = pose_node["x"].as<double>();
        reference_pose.position.y = pose_node["y"].as<double>();
        reference_pose.position.z = pose_node["z"].as<double>();

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
