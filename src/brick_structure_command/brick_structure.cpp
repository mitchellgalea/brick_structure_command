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
        geometry_msgs::Pose reference_pose = Transforms::pose2d2Pose(reference_pose_2D);

        double brick_spacing = 0.0;
        if(node["BrickSpacing"]) brick_spacing = node["BrickSpacing"].as<double>();

        YAML::Node layers_node = node["Layers"];
        if(layers_node.size() > 0)
        {
            YAML::Node first_layer_node = layers_node[0];
            double reference_offset = 0;
            double brick_z = 0;
            for(auto brick_node:first_layer_node["Bricks"])
            {
                Brick brick(brick_node["Colour"].as<char>());
                if(brick_node["Sequence"].as<unsigned>() != 1)
                {
                    reference_offset += brick.getXDim() + brick_spacing;
                }
                else
                {
                    brick_z = brick.getZDim();
                    reference_offset += brick.getXDim() / 2;
                    break;
                }
            }
            reference_pose = Transforms::tfPose(reference_pose, reference_offset, 0, 0, 0, 0);
            for(auto layer_node:layers_node)
            {
                double offset = layer_node["Offset"].as<double>();
                reference_pose = Transforms::tfPose(reference_pose, offset, 0, 0, 0, 0);
                geometry_msgs::Pose current_pose = reference_pose;
                for(auto brick_node:layer_node["Bricks"])
                {
                    Brick brick(brick_node["Colour"].as<char>());
                    brick.setPose()Transforms::tfPose(current_pose, brick.getXDim()/2, 0, brick.getZDim()/2, 0));
                    unsigned seq = brick_node["Sequence"].as<unsigned>();
                    if(bricks_.size() < seq) bricks_.resize(seq);
                    bricks_.at(seq-1) = brick;
                    current_pose = Transforms::tfPose(current_pose, brick.getXDim() + brick_spacing, 0, 0, 0);
                }
                reference_pose = Transforms::tfPose(reference_pose, 0, 0, brick_z, 0);
            }
        }
        else
        {
          //// TODO: add exception
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
