#include "brick_command/brick_structure.h"

using namespace brick_command;

//////// CONSTRUCTORS
BrickStructure::BrickStructure()
    :initialized_(false) {}
BrickStructure::BrickStructure(std::string path, double point_interval)
    :c_brick_count_(0), point_interval_(point_interval), initialized_(false)
{
    parseYAML(path);
    if(!checkBrickHeights())
    {
        std::cout << "BRICK HEIGHT FAIL" << std::endl;
        //// TODO: exception
    }
    if(!checkConstruction())
    {
        std::cout << "BRICK CONSTRUCTION FAIL" << std::endl;
        //// TODO: exception
    }
    initialized_ = true;
    print();
}

//////// PRIVATE METHODS
bool BrickStructure::parseYAML(std::string path)
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
            layers_ = node["Layers"].size();
            YAML::Node first_layer_node = layers_node[0];
            double reference_offset = 0;
            for(auto brick_node:first_layer_node["Bricks"])
            {
                Brick brick(brick_node["Colour"].as<char>());
                if(brick_node["Sequence"].as<unsigned>() != 1)
                {
                    reference_offset += brick.getXDim() + brick_spacing;
                }
                else
                {
                    layer_height_ = brick.getZDim();
                    reference_offset += brick.getXDim() / 2;
                    break;
                }
            }
            reference_pose = Transforms::tfPose(reference_pose, -reference_offset, 0, 0, 0);
            for(auto layer_node:layers_node)
            {
                double offset = layer_node["Offset"].as<double>();
                reference_pose = Transforms::tfPose(reference_pose, offset, 0, 0, 0);
                geometry_msgs::Pose current_pose = reference_pose;
                for(auto brick_node:layer_node["Bricks"])
                {
                    Brick brick(brick_node["Colour"].as<char>());
                    brick.setPose(Transforms::tfPose(current_pose, brick.getXDim()/2, 0, brick.getZDim()/2, 0));
                    unsigned seq = brick_node["Sequence"].as<unsigned>();
                    if(bricks_.size() < seq) bricks_.resize(seq);
                    bricks_.at(seq-1) = brick;
                    current_pose = Transforms::tfPose(current_pose, brick.getXDim() + brick_spacing, 0, 0, 0);
                }
                reference_pose = Transforms::tfPose(reference_pose, 0, 0, layer_height_, 0);
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
    for(auto brick:bricks_)
    {
        if(fabs(brick.getZDim() - layer_height_) >= 0.01) return false;
    }
    return true;
}

bool BrickStructure::checkBrickPlacement(Brick brick, int brick_count)
{
    unsigned layer = getLayer(brick);
    bool s1 = false;
    bool s2 = false;
    if(layer != 0)
    {
        std::vector<geometry_msgs::Point> points = getLayerPoints(layer - 1, brick_count);
        std::vector<geometry_msgs::Point> brick_points = brick.getPoints(false, point_interval_);
        for(unsigned i = 0; i < brick_points.size()/2; i++)
        {
            for(auto point:points)
            {
                if(pointDistance(point, brick_points[i]) < point_interval_)
                {
                    s1 = true;
                    break;
                }
                if(s1) break;
            }
        }
        for(unsigned i = brick_points.size(); i > brick_points.size()/2; i--)
        {
            for(auto point:points)
            {
                if(pointDistance(point, brick_points[i]) < point_interval_)
                {
                    s2 = true;
                    break;
                }
                if(s2) break;
            }
        }
        if(s1 && s2)return true;
        std::cout << "OTHER LAYER FAIL" << std::endl;
        return false;
    }
    else
    {
        if(fabs(brick.getPose().position.z - brick.getZDim()/2) < EPSILION) return true;
        std::cout << "FIRST LAYER FAIL" <<std::endl;
        return false;
    }
}

bool BrickStructure::checkConstruction()
{
    for(unsigned i = 0; i < bricks_.size(); i++)
    {
        if(!checkBrickPlacement(bricks_[i], i))return false;
    }
    return true;
}

unsigned BrickStructure::getLayer(Brick brick)
{
    double z = brick.getPose().position.z - brick.getZDim()/2;
    for(unsigned i = 0; i < layers_; i++)
    {
        if(fabs(z - i*layer_height_) < EPSILION) return i;
    }
    //// TODO exception
}

std::vector<Brick> BrickStructure::getBricksinLayer(unsigned layer, int brick_count)
{
    if(brick_count == -1)brick_count = bricks_.size();
    if(layer >= layers_); //// TODO exception
    std::vector<Brick> bricks;
    for(unsigned i = 0; i < brick_count; i++)
    {
        if(getLayer(bricks_[i]) == layer)bricks.push_back(bricks_[i]);
    }
    return bricks;
}
std::vector<geometry_msgs::Point> BrickStructure::getLayerPoints(unsigned layer, int brick_count)
{
    if(brick_count == -1)brick_count = bricks_.size();
    if(layer >= layers_); //// TODO exception

    std::vector<geometry_msgs::Point> points;
    std::vector<Brick> bricks = getBricksinLayer(layer, brick_count);
    for(auto brick:bricks)
    {
        std::vector<geometry_msgs::Point> brick_points = brick.getPoints(true, point_interval_);
        points.insert(std::end(points), std::begin(brick_points), std::end(brick_points));
    }
    return points;
}

double BrickStructure::pointDistance(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2) + pow(p2.z - p1.z, 2));
}

BrickMsg BrickStructure::adjacentBrick(Brick brick, unsigned brick_count, bool direction)
{
    std::vector<Brick> layer_bricks = getBricksinLayer(getLayer(brick), brick_count);
    Brick adjacentBrick;
    bool set = false;
    double distance = 100 * (!direction*-1 + direction*1);
    std::cout << distance << std::endl;
    for(auto layer_brick:layer_bricks)
    {
        double temp_distance = brick.getRelativeBrickPose(layer_brick).position.x;
        std::cout << temp_distance << std::endl;
        if(direction && temp_distance < distance && temp_distance > 0)
        {
            set = true;
            distance = temp_distance;
            adjacentBrick = layer_brick;
        }
        if(!direction && temp_distance > distance && temp_distance < 0)
        {
            set = true;
            distance = temp_distance;
            adjacentBrick = layer_brick;
        }
    }
    BrickMsg brick_msg;
    if(set)
    {
        brick_msg.colour = adjacentBrick.getColour();
        brick_msg.pose = adjacentBrick.getPose();
        brick_msg.active = true;
    }
    else brick_msg.active = false;
    return brick_msg;
}

std::vector<BrickMsg> BrickStructure::lowerBricks(Brick brick, unsigned brick_count)
{
    std::vector<Brick> layer_bricks = getBricksinLayer(getLayer(brick) - 1, brick_count);
    std::vector<Brick> lower_bricks;
    for(auto layer_brick:layer_bricks)
    {
        double distance = brick.getRelativeBrickPose(layer_brick).position.x;
        if(fabs(distance) < (brick.getXDim()/2 + layer_brick.getXDim()))
        {
            lower_bricks.push_back(layer_brick);
        }
    }
    std::vector<BrickMsg> brick_msgs;
    for(auto lower_brick:lower_bricks)
    {
        BrickMsg brick_msg;
        brick_msg.colour = lower_brick.getColour();
        brick_msg.pose = lower_brick.getPose();
        brick_msg.active = true;
        brick_msgs.push_back(brick_msg);
    }
    return brick_msgs;
}

//////// GETTERS
unsigned BrickStructure::getCBrickCount() const
{ return c_brick_count_; }

//////// METHODS
void BrickStructure::incCBrickCount()
{
    c_brick_count_ ++;
}

BrickCommand BrickStructure::getCBrickCommand(bool increment)
{
    BrickCommand brick_command;
    brick_command.brick.colour = bricks_[c_brick_count_].getColour();
    brick_command.brick.pose = bricks_[c_brick_count_].getPose();
    brick_command.right_brick = adjacentBrick(bricks_[c_brick_count_], c_brick_count_, true);
    brick_command.left_brick = adjacentBrick(bricks_[c_brick_count_], c_brick_count_, false);
    brick_command.lower_bricks = lowerBricks(bricks_[c_brick_count_], c_brick_count_);
    if(increment) incCBrickCount();
    return brick_command;
}

bool BrickStructure::structureComplete()
{
    if(c_brick_count_ >= bricks_.size()) return true;
    return false;
}

void BrickStructure::print()
{
    int count = 1;
    for(auto brick:bricks_)
    {
        std::cout << "Brick " << count << ", Colour: " << brick.getColour();
        std::cout << ", POSE(x: " << brick.getPose().position.x << ", y: " << brick.getPose().position.y << ", z: " << brick.getPose().position.z << ")" << std::endl;
        count ++;
    }
}

void BrickStructure::print2Count()
{

}
