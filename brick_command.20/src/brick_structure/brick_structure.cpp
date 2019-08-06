#include "brick_structure/brick_structure.h"

using namespace brick_structure;

//////// CONSTRUCTORS
BrickStructure::BrickStructure()
        : initialized_(false) {}

BrickStructure::BrickStructure(std::string path, double point_interval, int brick_count)
        : c_brick_count_(brick_count), point_interval_(point_interval), initialized_(false), brick_spacing_(0.0) {
    //sets layer height
    Brick layer_height_brick;
    layer_height_ = layer_height_brick.getZDim();

    parseYAML(path);
    if (!checkBrickHeights()) {
        ROS_ERROR("BRICK HEIGHT FAIL");
    }
    if (!checkConstruction()) {
        ROS_ERROR("BRICK CONSTRUCTION FAIL");
    }
    initialized_ = true;
    //print();
}

//////// PRIVATE METHODS

bool BrickStructure::parseYAML(std::string path) {
    YAML::Node node = YAML::LoadFile(path);
    //Sets Brick Spacing
    if (node["BrickSpacing"]) brick_spacing_ = node["BrickSpacing"].as<double>();

    if (node["Sections"]) {
        YAML::Node sections_node = node["Sections"];
        for (auto section_node:sections_node) {
            if (section_node["ReferencePose"] && section_node["Layers"]) {
                //Takes 2D Pose input and Transforms it to 3D Pose
                YAML::Node pose_node = section_node["ReferencePose"];
                Pose2D reference_pose_2D;
                reference_pose_2D.x = pose_node["x"].as<double>();
                reference_pose_2D.y = pose_node["y"].as<double>();
                reference_pose_2D.theta = pose_node["yaw"].as<double>() * (M_PI / 180);
                Pose reference_pose = Geometry::pose2d2Pose(reference_pose_2D);

                YAML::Node layers_node = section_node["Layers"];
                if (layers_node.size() > 0) {
                    layers_ = section_node["Layers"].size();
                    YAML::Node first_layer_node = layers_node[0];
                    double reference_offset = 0;
                    //Iterates through first layer bricks to find the reference pose
                    unsigned min_seq = 1000;
                    double min_seq_reference_offset;
                    bool space = false;
                    for (auto brick_node:first_layer_node["Bricks"]) {
                        if (brick_node["Sequence"] && brick_node["Color"]) {
                            unsigned sequence = brick_node["Sequence"].as<unsigned>();
                            if(sequence < min_seq)
                            {
                                min_seq = sequence;
                                min_seq_reference_offset = reference_offset;
                            }
                            Brick brick(brick_node["Color"].as<char>());
                            if (space)reference_offset += brick.getXDim() + brick_spacing_;
                            else {
                                reference_offset += brick.getXDim();
                                space = true;
                            }
                        } else if (brick_node["Space"]) {
                            double empty_space = brick_node["Space"].as<double>();
                            reference_offset += empty_space;
                            space = false;
                        } else
                            ROS_ERROR("BRICK NODE NEEDS TO BE EITHER SPACE OR BRICK");
                    }
                    reference_pose = Geometry::tfPose(reference_pose, -min_seq_reference_offset, 0, 0, 0);
                    //Iterates through all the layers and calculates brick poses
                    bool planes_set = false;    // member to set planes
                    for (auto layer_node:layers_node) {
                        double offset = layer_node["Offset"].as<double>();
                        reference_pose = Geometry::tfPose(reference_pose, offset, 0, 0, 0);
                        Pose current_pose = reference_pose;
                        space = false;
                        for (auto brick_node:layer_node["Bricks"]) {
                            if (brick_node["Sequence"] && brick_node["Color"]) {
                                unsigned sequence = brick_node["Sequence"].as<unsigned>();
                                if (space)current_pose = Geometry::tfPose(current_pose, brick_spacing_, 0, 0, 0);
                                else space = true;
                                //calculates Pose of Brick
                                Brick brick(brick_node["Color"].as<char>());
                                brick.setPose(current_pose);
                                if (bricks_.size() < sequence) bricks_.resize(sequence);
                                bricks_.at(sequence - 1) = brick;
                                current_pose = Geometry::tfPose(current_pose, brick.getXDim(), 0, 0, 0);
                                //Extracts the Planes for the first brick in each section
                                if (!planes_set) {
                                    //std::vector<Point> brick_points = brick.getCornerPoints();
                                    std::vector<Pose> brick_poses = brick.getCornerPoses();
                                    planes_.push_back(
                                            Geometry::points2Plane(brick_poses[0], brick_poses[1], brick_poses[2]));
                                    planes_.back().brick_index = sequence - 1;
                                    planes_.back().brick_face = true;
                                    planes_.push_back(
                                            Geometry::points2Plane(brick_poses[4], brick_poses[5], brick_poses[6]));
                                    planes_.back().brick_index = sequence - 1;
                                    planes_.back().brick_face = false;
                                    planes_set = true;
                                }
                            } else if (brick_node["Space"]) {
                                double empty_space = brick_node["Space"].as<double>();
                                current_pose = Geometry::tfPose(current_pose, empty_space, 0, 0, 0);
                                space = false;
                            } else
                                ROS_ERROR("BRICK NODE NEEDS TO BE EITHER SPACE OR BRICK");
                        }
                        reference_pose = Geometry::tfPose(reference_pose, 0, 0, layer_height_, 0);
                    }
                } else
                    ROS_ERROR("BLUEPRINT ERROR: LAYERS MUST BE GREATER THAN 0");
            } else
                ROS_ERROR("BLUEPRINT ERROR: SECTION MUST CONTAIN REFERENCE POSE AND LAYERS");
        }
    }
}

bool BrickStructure::checkBrickHeights() {
    for (auto brick:bricks_) {
        if (fabs(brick.getZDim() - layer_height_) >= 0.01) return false;
    }
    return true;
}

bool BrickStructure::checkBrickPlacement(Brick brick, int brick_count) {
    unsigned layer = getLayer(brick);
    bool s1 = false;
    bool s2 = false;
    if (layer != 0) {
        //gets to points in the layer below the brick and the points in the bottom of the brick
        std::vector<Point> points = getLayerPoints(layer - 1, brick_count);
        std::vector<Point> brick_points = brick.getPoints(false, point_interval_);

        //for brick to be able to be placed a point must be in both halves of the brick
        for (unsigned i = 0; i < brick_points.size() / 2; i++) {
            for (auto point:points) {
                if (pointDistance(point, brick_points[i]) < point_interval_) {
                    s1 = true;
                    break;
                }
                if (s1) break;
            }
        }
        for (unsigned i = brick_points.size(); i > brick_points.size() / 2; i--) {
            for (auto point:points) {
                if (pointDistance(point, brick_points[i]) < point_interval_) {
                    s2 = true;
                    break;
                }
                if (s2) break;
            }
        }
        if (s1 && s2)return true;
        return false;
    } else {
        if (fabs(brick.getPose().position.z) < EPSILION) return true;
        return false;
    }
}

bool BrickStructure::checkConstruction() {
    for (unsigned i = 0; i < bricks_.size(); i++) {
        if (!checkBrickPlacement(bricks_[i], i))return false;
    }
    return true;
}

unsigned BrickStructure::getLayer(Brick brick) {
    double z = brick.getPose().position.z;
    for (unsigned i = 0; i < layers_; i++) {
        if (fabs(z - i * layer_height_) < EPSILION) return i;
    }
    ROS_ERROR("GET LAYER ERROR: BRICK NOT IN ANY LAYERS");
}

std::vector<Brick> BrickStructure::getBricksinLayer(unsigned layer, int brick_count) {
    if (brick_count == -1)brick_count = bricks_.size();
    if (layer >= layers_) ROS_ERROR("BRICKS IN LAYER: INPUT LAYER GREATER THAN LAYER COUNT");
    std::vector<Brick> layer_bricks;
    for (unsigned i = 0; i < brick_count; i++) {
        if (getLayer(bricks_[i]) == layer)layer_bricks.push_back(bricks_[i]);
    }
    return layer_bricks;
}

std::vector<unsigned> BrickStructure::getBrickIndexsinLayer(unsigned layer, int brick_count) {
    if (brick_count == -1)brick_count = bricks_.size();
    if (layer >= layers_) ROS_ERROR("BRICKS IN LAYER: INPUT LAYER GREATER THAN LAYER COUNT");
    std::vector<unsigned> indexs;
    for (unsigned i = 0; i < brick_count; i++) {
        if (getLayer(bricks_[i]) == layer)indexs.push_back(i);
    }
    return indexs;
}


std::vector<Brick> BrickStructure::getBricksinLayer(unsigned layer, std::vector<Brick> bricks) {
    if (layer >= layers_) ROS_ERROR("BRICKS IN LAYER: INPUT LAYER GREATER THAN LAYER COUNT");
    std::vector<Brick> layer_bricks;
    for (auto brick:bricks) {
        if (getLayer(brick) == layer)layer_bricks.push_back(brick);
    }
    return layer_bricks;
}

std::vector<Brick> BrickStructure::getBricksinPlane(unsigned plane, int brick_count) {
    if (brick_count == -1)brick_count = bricks_.size();
    if (plane >= planes_.size()) ROS_ERROR("BRICKS IN LAYER: INPUT LAYER GREATER THAN LAYER COUNT");
    std::vector<Brick> plane_bricks_;
    for (unsigned i = 0; i < brick_count; i++) {
        std::vector<Point> points = bricks_[i].getCornerPoints();
        if (Geometry::pointOnPlane(points[0], planes_[plane])
            && Geometry::pointOnPlane(points[1], planes_[plane])
            && Geometry::pointOnPlane(points[2], planes_[plane])
            && Geometry::pointOnPlane(points[3], planes_[plane])) {
            plane_bricks_.push_back(bricks_[i]);
        }
        if (Geometry::pointOnPlane(points[4], planes_[plane])
            && Geometry::pointOnPlane(points[5], planes_[plane])
            && Geometry::pointOnPlane(points[6], planes_[plane])
            && Geometry::pointOnPlane(points[7], planes_[plane])) {
            plane_bricks_.push_back(bricks_[i]);
        }

    }
    return plane_bricks_;
}

std::vector<Brick> BrickStructure::getBricksinPlane(unsigned plane, std::vector<Brick> bricks) {
    if (plane >= planes_.size()) ROS_ERROR("BRICKS IN LAYER: INPUT LAYER GREATER THAN LAYER COUNT");
    std::vector<Brick> plane_bricks_;
    for (auto brick:bricks) {
        std::vector<Point> points = brick.getCornerPoints();
        if (Geometry::pointOnPlane(points[0], planes_[plane])
            && Geometry::pointOnPlane(points[1], planes_[plane])
            && Geometry::pointOnPlane(points[2], planes_[plane])
            && Geometry::pointOnPlane(points[3], planes_[plane])) {
            plane_bricks_.push_back(brick);
        }
        if (Geometry::pointOnPlane(points[4], planes_[plane])
            && Geometry::pointOnPlane(points[5], planes_[plane])
            && Geometry::pointOnPlane(points[6], planes_[plane])
            && Geometry::pointOnPlane(points[7], planes_[plane])) {
            plane_bricks_.push_back(brick);
        }
    }
    return plane_bricks_;
}

std::vector<unsigned> BrickStructure::getBrickIndexsinPlane(unsigned plane, std::vector<unsigned> brick_indexs) {
    if (plane >= planes_.size()) ROS_ERROR("BRICKS IN LAYER: INPUT LAYER GREATER THAN LAYER COUNT");
    std::vector<unsigned> indexs;
    for (auto index:brick_indexs) {
        std::vector<Point> points = bricks_[index].getCornerPoints();
        if (Geometry::pointOnPlane(points[0], planes_[plane])
            && Geometry::pointOnPlane(points[1], planes_[plane])
            && Geometry::pointOnPlane(points[2], planes_[plane])
            && Geometry::pointOnPlane(points[3], planes_[plane])) {
            indexs.push_back(index);
        }
        if (Geometry::pointOnPlane(points[4], planes_[plane])
            && Geometry::pointOnPlane(points[5], planes_[plane])
            && Geometry::pointOnPlane(points[6], planes_[plane])
            && Geometry::pointOnPlane(points[7], planes_[plane])) {
            indexs.push_back(index);
        }
    }
    return indexs;
}

unsigned BrickStructure::getBrickPlaneIndex(bool front, int brick_index) {
    if (brick_index == -1) brick_index = c_brick_count_;
    std::vector<Point> points = bricks_[brick_index].getCornerPoints();
    for (unsigned i = 0; i < planes_.size(); i++) {
        if (Geometry::pointOnPlane(points[0], planes_[i])
            && Geometry::pointOnPlane(points[1], planes_[i])
            && Geometry::pointOnPlane(points[2], planes_[i])
            && Geometry::pointOnPlane(points[3], planes_[i]) && front) {
            return i;
        }
        if (Geometry::pointOnPlane(points[4], planes_[i])
            && Geometry::pointOnPlane(points[5], planes_[i])
            && Geometry::pointOnPlane(points[6], planes_[i])
            && Geometry::pointOnPlane(points[7], planes_[i]) && !front) {
            return i;
        }
    }
    ROS_ERROR("Get Brick Plane Index Failure: No Matching Planes Found");
}

std::vector<Point> BrickStructure::getLayerPoints(unsigned layer, int brick_count) {
    if (brick_count == -1)brick_count = bricks_.size();
    if (layer >= layers_) ROS_ERROR("LAYER POINTS: INPUT LAYER GREATER THAN LAYER COUNT");
    std::vector<Point> points;
    std::vector<Brick> bricks = getBricksinLayer(layer, brick_count);
    for (auto brick:bricks) {
        std::vector<Point> brick_points = brick.getPoints(true, point_interval_);
        points.insert(std::end(points), std::begin(brick_points), std::end(brick_points));
    }
    return points;
}

double BrickStructure::pointDistance(Point p1, Point p2) {
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2) + pow(p2.z - p1.z, 2));
}

int BrickStructure::adjacentBrickIndex(double max_dist, bool front, std::vector<unsigned> &control_corner_idxs, unsigned brick_index, unsigned brick_count) {
    // Gets the plane lines either front or back
    unsigned plane_idx = getBrickPlaneIndex(front, brick_index);
    std::vector<Line> plane_lines = getStructureLines(plane_idx, brick_count);
    // Gets vertical lines
    std::vector<Line> vert_lines;
    for (auto line:plane_lines) {
        if (fabs(Geometry::pointDistance(line.point_1, line.point_2, true, true, false)) < EPSILION)
            vert_lines.push_back(line);
    }
    // Gets the brick points
    std::vector<Point> brick_points = bricks_[brick_index].getCornerPoints();
    std::vector<Point> plane_brick_points;
    if (front) plane_brick_points = {brick_points[0], brick_points[1], brick_points[2], brick_points[3]};
    else plane_brick_points = {brick_points[4], brick_points[5], brick_points[6], brick_points[7]};
    // Iterates through the vertical lines to find a line adjacent to the brick
    Line adjacent_line;
    bool set = false;
    double min_dist = max_dist;
    for (auto line:vert_lines) {
        // Gets the high point of the line to check if the layers match
        Point high_point, low_point;
        if (line.point_1.z > line.point_2.z) {
            high_point = line.point_1;
            low_point = line.point_2;
        } else {
            low_point = line.point_1;
            high_point = line.point_2;
        }
        // Checks if layers match
        if (fabs(low_point.z - plane_brick_points[0].z) < EPSILION) {
            // Gets distance between the line and both sides of the brick
            double point_dist_1 = Geometry::pointDistance(low_point, plane_brick_points[0]);
            double point_dist_2 = Geometry::pointDistance(low_point, plane_brick_points[3]);
            // Checks if the min point distance is less than min distance and sets if so
            if (std::min(point_dist_1, point_dist_2) < min_dist) {
                set = true;
                min_dist = std::min(point_dist_1, point_dist_2);
                adjacent_line = line;
            }
        }
    }
    if (!set) return -1;
    // if line is set goes through the bricks to find which brick the line is in
    for (unsigned i = 0; i < brick_count; i++) {
        std::vector<Point> brick_points = bricks_[i].getCornerPoints();
        // Iterates through points twice to find brick with matching points to line
        for (unsigned m = 0; m < brick_points.size(); m++) {
            for (unsigned n = 0; n < brick_points.size(); n++) {
                if (fabs(pointDistance(adjacent_line.point_1, brick_points[m])) < EPSILION
                    && fabs(pointDistance(adjacent_line.point_2, brick_points[n])) < EPSILION){
                    control_corner_idxs = {m,n};
                    return i;
                }
            }
        }
    }
}

int BrickStructure::lowerLineBrickIndex(double max_dist, bool front, std::vector<unsigned> &control_corner_idxs, unsigned brick_index, unsigned brick_count){
    // Gets the plane lines either front or back
    unsigned plane_idx = getBrickPlaneIndex(front, brick_index);
    std::vector<Line> plane_lines = getStructureLines(plane_idx, brick_count);
    // Gets vertical lines
    std::vector<Line> vert_lines;
    for (auto line:plane_lines) {
        if (fabs(Geometry::pointDistance(line.point_1, line.point_2, true, true, false)) < EPSILION)
            vert_lines.push_back(line);
    }
    // Gets the brick points
    std::vector<Point> brick_points = bricks_[brick_index].getCornerPoints();
    std::vector<Point> plane_brick_points;
    if (front) plane_brick_points = {brick_points[0], brick_points[1], brick_points[2], brick_points[3]};
    else plane_brick_points = {brick_points[4], brick_points[5], brick_points[6], brick_points[7]};
    // Iterates through the vertical lines to find a line edge lower to the brick
    Line lower_line;
    bool set = false;
    double min_dist = max_dist;
    for (auto line:vert_lines) {
        // Gets the high point of the line to check if the layers match
        Point high_point, low_point;
        if (line.point_1.z > line.point_2.z) {
            high_point = line.point_1;
            low_point = line.point_2;
        } else {
            low_point = line.point_1;
            high_point = line.point_2;
        }
        // Checks if layers match
        if (fabs(high_point.z - plane_brick_points[0].z) < EPSILION) {
            // Gets distance between the line and both sides of the brick
            double point_dist_1 = Geometry::pointDistance(high_point, plane_brick_points[0]);
            double point_dist_2 = Geometry::pointDistance(high_point, plane_brick_points[3]);
            // Checks if the min point distance is less than min distance and sets if so
            if (std::min(point_dist_1, point_dist_2) < min_dist) {
                set = true;
                min_dist = std::min(point_dist_1, point_dist_2);
                lower_line = line;
            }
        }
    }
    if (!set) return -1;
    // if line is set goes through the bricks to find which brick the line is in
    for (unsigned i = 0; i < brick_count; i++) {
        std::vector<Point> brick_points = bricks_[i].getCornerPoints();
        // Iterates through points twice to find brick with matching points to line
        for (unsigned m = 0; m < brick_points.size(); m++) {
            for (unsigned n = 0; n < brick_points.size(); n++) {
                if (fabs(pointDistance(lower_line.point_1, brick_points[m]) ) < EPSILION
                    && fabs(pointDistance(lower_line.point_2, brick_points[n])) < EPSILION )
                    {
                        control_corner_idxs = {m,n};
                        return i;
                    }
            }
        }
    }
}

std::vector<Brick> BrickStructure::downBricks(unsigned brick_index, unsigned brick_count) {
    std::vector<Brick> layer_bricks = getBricksinLayer(getLayer(bricks_[brick_index]) - 1, brick_count);
    std::vector<Brick> lower_bricks;
    for (auto layer_brick:layer_bricks) {
        double distance = bricks_[brick_index].getRelativeBrickPose(layer_brick).position.x;
        if (distance >= layer_brick.getXDim() && distance <= bricks_[brick_index].getXDim()) {
            lower_bricks.push_back(layer_brick);
        }
    }
    return lower_bricks;
}
void BrickStructure::setBrickCommandControl(double max_dist, mrc_msgs::BrickCommand &brick_command,
                                            Pose placement_offset, std::vector<mrc_msgs::BrickPile> brick_piles,
                                            int brick_index) {
    Pose pile_pose;
    std::string color = brick_command.brick.color;
    for(auto brick_pile:brick_piles)
    {
        if(color.compare(brick_pile.color) == 0){
            pile_pose.position.x = brick_pile.centre.x;
            pile_pose.position.y = brick_pile.centre.y;
            break;
        }
    }
    brick_command.control_brick_index = -1;
    int control_brick_index;
    if (brick_index == -1)brick_index = c_brick_count_;
    if (c_brick_count_ == 0) {
        brick_command.control_type = 1;
        brick_command.robot_place_pose.pose = getBrickPlacementPose(brick_index, placement_offset, pile_pose);
        return;
    }
    // Check for Adjacent Bricks
    std::vector<unsigned> control_corner_idxs;
    control_brick_index = adjacentBrickIndex(max_dist, true, control_corner_idxs, brick_index, brick_index);
    if (control_brick_index != -1) {
        brick_command.control_type = 2;
        brick_command.control_brick = bricks_[control_brick_index].getBrickMsg();
        brick_command.control_brick_index = control_brick_index;
        //brick_command.control_corner_idxs = control_corner_idxs;
        brick_command.robot_place_pose.pose = getBrickPlacementPose(control_brick_index, brick_index,
                                                               placement_offset, pile_pose);
        return;
    }
    // Check for Lower Bricks
    if (getLayer(bricks_[brick_index]) == 0) {
        brick_command.control_type = 1;
        brick_command.robot_place_pose.pose = getBrickPlacementPose(brick_index, placement_offset, pile_pose);
        return;
    }
    // Since Front and Back lower edge can be different it checks both the front and back for lower edge scenario
    std::vector<int> control_brick_indexs_3;
    std::vector<std::vector<unsigned>> control_corner_idxs_3;
    int control_brick_index_f = lowerLineBrickIndex(max_dist, true, control_corner_idxs, brick_index, brick_index);
    if (control_brick_index_f != -1) {
        control_brick_indexs_3.push_back(control_brick_index_f);
        control_corner_idxs_3.push_back(control_corner_idxs);
    }
    int control_brick_index_b = lowerLineBrickIndex(max_dist, false, control_corner_idxs, brick_index, brick_index);
    if (control_brick_index_b != -1) {
        control_brick_indexs_3.push_back(control_brick_index_b);
        control_corner_idxs_3.push_back(control_corner_idxs);
    }
    if (!control_brick_indexs_3.empty()) {
        brick_command.control_type = 3;
        int index = -1;
        brick_command.robot_place_pose.pose = getBrickPlacementPose(index, control_brick_indexs_3, control_corner_idxs_3,
                                                               brick_index, placement_offset, pile_pose);
        if(index!=-1)
        {
            brick_command.control_brick = bricks_[control_brick_indexs_3[index]].getBrickMsg();
            brick_command.control_brick_index = control_brick_indexs_3[index];
            //brick_command.control_corner_idxs = control_corner_idxs_3[index];
        }
        return;
    } else {
        brick_command.control_type = 4;
        brick_command.robot_place_pose.pose = getBrickPlacementPose(brick_index, placement_offset, pile_pose);
        return;
    }
}

Pose BrickStructure::getBrickPlacementPose(int brick_index,Pose placement_offset, Pose pile_pose) {
    std::vector<Pose> poses;
    Pose brick_centre_pose = bricks_[brick_index].getCentrePose();
    Pose temp_pose;
    // Gets Poses of either side of the brick
    temp_pose = Geometry::tfPose(brick_centre_pose, 0, 0.5 * bricks_[brick_index].getYDim(), 0, -M_PI/2);
    temp_pose = Geometry::tfPose(temp_pose, placement_offset);
    temp_pose.position.z = 0;
    poses.push_back(temp_pose);
    temp_pose = Geometry::tfPose(brick_centre_pose, 0, -0.5 * bricks_[brick_index].getYDim(), 0, M_PI/2);
    temp_pose = Geometry::tfPose(temp_pose, placement_offset);
    temp_pose.position.z = 0;
    poses.push_back(temp_pose);
    // Checks poses for collisions

    // Gets the pose closest to the correct color pile as placement pose
    Pose placement_pose = poses[0];
    double min_distance = Geometry::pointDistance(poses[0].position, pile_pose.position);
    for(auto pose:poses)
    {
        double distance = Geometry::pointDistance(pose.position, pile_pose.position);
        if(distance < min_distance)
        {
            min_distance = distance;
            placement_pose = pose;
        }
    }
    return placement_pose;

}

Pose BrickStructure::getBrickPlacementPose(int control_brick_index, int brick_index,
                                           Pose placement_offset, Pose pile_pose) {
    std::vector<Pose> poses;
    Pose brick_centre_pose = bricks_[brick_index].getCentrePose();
    Pose temp_pose;

    // Find adjacent side
    Pose relative_pose = bricks_[brick_index].getRelativeBrickPose(bricks_[control_brick_index]);
    Pose brick_edge_pose;
    // Checks which side the adjacent brick is
    if (relative_pose.position.x > 0)
        brick_edge_pose = Geometry::tfPose(brick_centre_pose, bricks_[brick_index].getXDim()/2, 0, 0, 0);
    else brick_edge_pose = Geometry::tfPose(brick_centre_pose, -1 * bricks_[brick_index].getXDim()/2, 0, 0, 0);
    temp_pose = Geometry::tfPose(brick_edge_pose, 0, 0.5 * bricks_[brick_index].getYDim(), 0, -M_PI/2);
    temp_pose = Geometry::tfPose(temp_pose, placement_offset);
    temp_pose.position.z = 0;
    poses.push_back(temp_pose);
    temp_pose = Geometry::tfPose(brick_edge_pose, 0, -0.5*bricks_[brick_index].getYDim(), 0, M_PI/2);
    temp_pose = Geometry::tfPose(temp_pose, placement_offset);
    temp_pose.position.z = 0;
    poses.push_back(temp_pose);
    // Checks poses for collisions

    // Gets the pose closest to the correct color pile as placement pose
    Pose placement_pose = poses[0];
    double min_distance = Geometry::pointDistance(poses[0].position, pile_pose.position);
    for(auto pose:poses)
    {
        double distance = Geometry::pointDistance(pose.position, pile_pose.position);
        if(distance < min_distance)
        {
            min_distance = distance;
            placement_pose = pose;
        }
    }
    return placement_pose;
}
Pose BrickStructure::getBrickPlacementPose( int &index,
                                           std::vector<int> &control_brick_indexs,
                                           std::vector<std::vector<unsigned>> control_corner_idxs, int brick_index,
                                           Pose placement_offset, Pose pile_pose) {
    std::vector<Pose> poses;
    Pose brick_centre_pose = bricks_[brick_index].getCentrePose();
    Pose temp_pose;
    // Iterates through the different possible control_brick_indexs
    for (unsigned i = 0; i < control_brick_indexs.size(); i++) {
        // Gets relative pose from brick centre to control edge to determine which side of brick to use
        Pose relative_pose = Geometry::relativePose(bricks_[brick_index].getCentrePose(),
                                                    bricks_[control_brick_indexs[i]].getCornerPoses()[control_corner_idxs[i][0]]);
        Pose brick_edge_pose;
        if (relative_pose.position.x > 0)
            brick_edge_pose = Geometry::tfPose(brick_centre_pose, bricks_[brick_index].getXDim() / 2, 0, 0, 0);
        else brick_edge_pose = Geometry::tfPose(brick_centre_pose, -1 * bricks_[brick_index].getXDim() / 2, 0, 0, 0);
        if (control_corner_idxs[i][0] >= 0 && control_corner_idxs[i][0] < 4) {
            temp_pose = Geometry::tfPose(brick_edge_pose, 0, -0.5 * bricks_[brick_index].getYDim(), 0, M_PI/2);
            temp_pose = Geometry::tfPose(temp_pose, placement_offset);
            temp_pose.position.z = 0;
            poses.push_back(temp_pose);
        } else if (control_corner_idxs[i][0] >= 4 && control_corner_idxs[i][0] < 8) {
            temp_pose = Geometry::tfPose(brick_edge_pose, 0, 0.5 * bricks_[brick_index].getYDim(), 0, -M_PI/2);
            temp_pose = Geometry::tfPose(temp_pose, placement_offset);
            temp_pose.position.z = 0;
            poses.push_back(temp_pose);
        } else ROS_ERROR("Get Brick Placement Pose Error: Lower Edge, control corner idxs invalid");
    }
    // Checks poses for collisions

    // Gets the pose closest to the correct color pile as placement pose
    Pose placement_pose = poses[0];
    double min_distance = Geometry::pointDistance(poses[0].position, pile_pose.position);
    index = control_brick_indexs[0];
    for(unsigned i = 0; i < poses.size(); i++)
    {
        double distance = Geometry::pointDistance(poses[i].position, pile_pose.position);
        if(distance < min_distance)
        {
            placement_pose = poses[i];
            min_distance = distance;
            index = i;
        }
    }
    return placement_pose;
}
//////// GETTERS
unsigned BrickStructure::getCBrickCount() const { return c_brick_count_; }

unsigned BrickStructure::getBricksSize() const {
    return bricks_.size();
}

bool BrickStructure::initialized() const { return initialized_; }

Brick BrickStructure::getBrick(unsigned brick_index) {
    if (brick_index > bricks_.size()) ROS_ERROR("Get Brick: Brick out of index");
    else return bricks_[brick_index];
}

//////// METHODS
void BrickStructure::incCBrickCount() {
    c_brick_count_++;
}

mrc_msgs::BrickCommand BrickStructure::getCBrickCommand(double max_dist, Pose placement_offset,
        std::vector<mrc_msgs::BrickPile> brick_piles, bool increment) {
    mrc_msgs::BrickCommand brick_command;
    brick_command.brick = bricks_[c_brick_count_].getBrickMsg();
    brick_command.brick.pose.pose = bricks_[c_brick_count_].getCentreTopFacePose();
    setBrickCommandControl(max_dist, brick_command, placement_offset, brick_piles, c_brick_count_);
    if (increment) incCBrickCount();
    return brick_command;
}

mrc_msgs::BrickCommand BrickStructure::getCBrickCommand(double max_dist, Pose placement_offset, std::vector<PoseStamped> brick_poses,
                                                             std::vector<mrc_msgs::BrickPile> brick_piles, bool increment) {
    mrc_msgs::BrickCommand brick_command;
    brick_command.brick = bricks_[c_brick_count_].getBrickMsg();
    brick_command.brick.pose = brick_poses[c_brick_count_];
    setBrickCommandControl(max_dist, brick_command, placement_offset, brick_piles, c_brick_count_);
    if(brick_command.control_brick_index != -1) brick_command.control_brick.pose = brick_poses[brick_command.control_brick_index];
    if (increment) incCBrickCount();
    return brick_command;
}

bool BrickStructure::structureComplete() {
    if (c_brick_count_ >= bricks_.size()) return true;
    return false;
}

void BrickStructure::gazeboLaunchOut(std::ofstream &myfile, unsigned brick_count) {
    std::string newline = "\n";
    myfile << R"(<?xml version="1.0"?>)" << newline;
    myfile << R"(<launch>)" << newline;
    myfile << R"(    <!-- Launch Empty Gazebo World -->)" << newline;
    myfile << R"(    <include file="$(find gazebo_ros)/launch/empty_world.launch">)" << newline;
    myfile << R"(      <arg name="gui" value="true"/>)" << newline;
    myfile << R"(      <arg name="paused" value="true"/>)" << newline;
    myfile << R"(    </include>)" << newline;

    if (brick_count == 0) brick_count = bricks_.size();
    myfile << R"(    <!-- Spawn each brick -->)" << newline;
    for (unsigned i = 0; i < brick_count; i++) {
        myfile << R"(    <node name="spawn_brick_)" << i << R"(" pkg="gazebo_ros" type="spawn_model")" << newline;
        myfile << R"(          args="-file $(find arm_gazebo)/models/)";
        std::string color = bricks_[i].getColor();
        if (color == RedBrick::color) myfile << R"(mbzirc_red_brick/mbzirc_red_brick.sdf)";
        else if (color == GreenBrick::color) myfile << R"(mbzirc_green_brick/mbzirc_green_brick.sdf)";
        else if (color == BlueBrick::color) myfile << R"(mbzirc_blue_brick/mbzirc_blue_brick.sdf)";
        else if (color == OrangeBrick::color) myfile << R"(mbzirc_orange_brick/mbzirc_orange_brick.sdf)";
        myfile << R"( -sdf)";
        Pose centre_pose = bricks_[i].getCentrePose();
        myfile << R"( -x )" << centre_pose.position.x;
        myfile << R"( -y )" << centre_pose.position.y;
        myfile << R"( -z )" << centre_pose.position.z;
        myfile << R"( -Y )" << Geometry::pose2Yaw(centre_pose);
        myfile << R"( -model brick_)" << i << R"(" />)" << newline;
    }
    myfile << R"(</launch>)" << newline;
}

void BrickStructure::gazeboLaunchOut(std::ofstream &myfile, unsigned brick_count,
                                    double x, double y, double z, double yaw) {
    std::string newline = "\n";
    myfile << R"(<?xml version="1.0"?>)" << newline;
    myfile << R"(<launch>)" << newline;
    myfile << R"(    <!-- Robot Arguments -->)" << newline;
    myfile << R"(    <arg name="model" default="waffle" doc="model type [burger, waffle, waffle_pi]"/>)" << newline;
    myfile << R"(    <arg name="x_pos" default=")" << x << R"("/>)" << newline;
    myfile << R"(    <arg name="y_pos" default=")" << y << R"("/>)" << newline;
    myfile << R"(    <arg name="z_pos" default=")" << z << R"("/>)" << newline;
    myfile << R"(    <arg name="Y" default=")" << yaw << R"("/>)" << newline;
    myfile << R"(    <!-- Launch Empty Gazebo World -->)" << newline;
    myfile << R"(    <include file="$(find gazebo_ros)/launch/empty_world.launch">)" << newline;
    myfile << R"(      <arg name="gui" value="true"/>)" << newline;
    myfile << R"(      <arg name="paused" value="true"/>)" << newline;
    myfile << R"(    </include>)" << newline;

    if (brick_count == 0) brick_count = bricks_.size();
    myfile << R"(    <!-- Spawn each brick -->)" << newline;
    for (unsigned i = 0; i < brick_count; i++) {
        myfile << R"(    <node name="spawn_brick_)" << i << R"(" pkg="gazebo_ros" type="spawn_model")" << newline;
        myfile << R"(          args="-file $(find arm_gazebo)/models/)";
        std::string color = bricks_[i].getColor();
        if (color == RedBrick::color) myfile << R"(mbzirc_red_brick/mbzirc_red_brick.sdf)";
        else if (color == GreenBrick::color) myfile << R"(mbzirc_green_brick/mbzirc_green_brick.sdf)";
        else if (color == BlueBrick::color) myfile << R"(mbzirc_blue_brick/mbzirc_blue_brick.sdf)";
        else if (color == OrangeBrick::color) myfile << R"(mbzirc_orange_brick/mbzirc_orange_brick.sdf)";
        myfile << R"( -sdf)";
        Pose centre_pose = bricks_[i].getCentrePose();
        myfile << R"( -x )" << centre_pose.position.x;
        myfile << R"( -y )" << centre_pose.position.y;
        myfile << R"( -z )" << centre_pose.position.z;
        myfile << R"( -Y )" << Geometry::pose2Yaw(centre_pose);
        myfile << R"( -model brick_)" << i << R"(" />)" << newline;
    }
    myfile << R"(    <!-- Robot Nodes -->)" << newline;
    myfile << R"(    <param name="robot_description" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro" />)" << newline;
    myfile << R"(    <node pkg="gazebo_ros" type="spawn_model" name="spawn_urdf")" << newline;
    myfile << R"(          args="-urdf -model turtlebot3_$(arg model) -x $(arg x_pos) -y $(arg y_pos) -z $(arg z_pos) -Y $(arg Y) -param robot_description" />)" << newline;
    myfile << R"(    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher">)" << newline;
    myfile << R"(    <param name="publish_frequency" type="double" value="50.0" />)" << newline;
    myfile << R"(    </node>)" << newline;
    myfile << R"(</launch>)" << newline;
}

std::vector<Point> BrickStructure::getBrickCornerPoints(int brick_count, int plane) {
    if (brick_count == -1)brick_count = bricks_.size();
    std::vector<Point> points;
    //iterates through each brick and adds to points
    for (unsigned i = 0; i < brick_count; i++) {
        std::vector<Point> brick_points = bricks_[i].getCornerPoints();
        points.insert(std::end(points), std::begin(brick_points), std::end(brick_points));
    }
    if (plane == -1) return points;
    if (plane < planes_.size()) return Geometry::pointsOnPlane(points, planes_[plane]);
    else {
        ROS_ERROR("PLANES OUT OF RANGE");
        return points;
    }
}

std::vector<Pose> BrickStructure::getBrickCornerPoses(int brick_count, int plane) {
    if (brick_count == -1)brick_count = bricks_.size();
    std::vector<Pose> poses;
    //iterates through each brick and adds to points
    for (unsigned i = 0; i < brick_count; i++) {
        std::vector<Pose> brick_poses = bricks_[i].getCornerPoses();
        poses.insert(std::end(poses), std::begin(brick_poses), std::end(brick_poses));
    }
    if (plane == -1) return poses;
    if (plane < planes_.size()) return Geometry::posesOnPlane(poses, planes_[plane]);
    else {
        ROS_ERROR("PLANES OUT OF RANGE");
        return poses;
    }
}

std::vector<Point> BrickStructure::getStructureCornerPoints(int brick_count, int plane) {
    if (brick_count == -1)brick_count = bricks_.size();
    std::vector<Point> points;
    //goes through each layer to get the corner points
    for (unsigned i = 0; i < layers_; i++) {
        std::vector<Point> layer_points;
        std::vector<Brick> layer_bricks = getBricksinLayer(i, brick_count);
        //iterates through each brick in the layer and gets the points
        for (auto brick:layer_bricks) {
            std::vector<Point> brick_points = brick.getCornerPoints();
            layer_points.insert(std::end(layer_points), std::begin(brick_points), std::end(brick_points));
        }
        //iterates through all the layer corner points and checks whether there is another
        //point close to it, if so it is not a corener
        for (unsigned a = 0; a < layer_points.size(); a++) {
            bool close = false;
            for (unsigned b = 0; b < layer_points.size(); b++) {
                if (a != b) {
                    double dx = layer_points[a].x - layer_points[b].x;
                    double dy = layer_points[a].y - layer_points[b].y;
                    double dz = layer_points[a].z - layer_points[b].z;
                    double distance = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
                    if (distance <= brick_spacing_ + 0.001) {
                        close = true;
                        break;
                    }
                }
            }
            if (!close) points.push_back(layer_points[a]);
        }
    }
    //adds all points that do not overlap into final points
    std::vector<Point> checked_points;
    checked_points.push_back(points[0]);
    for (unsigned a = 0; a < points.size(); a++) {
        bool close = false;
        for (unsigned b = 0; b < checked_points.size(); b++) {
            double distance = Geometry::pointDistance(points[a], checked_points[b]);
            if (distance <= brick_spacing_ + EPSILION) {
                close = true;
                break;
            }
        }
        if (!close) checked_points.push_back(points[a]);
    }

    if (plane == -1) return checked_points;
    if (plane < planes_.size()) return Geometry::pointsOnPlane(checked_points, planes_[plane]);
    else {
        ROS_ERROR("PLANES OUT OF RANGE");
        return checked_points;
    }
}

std::vector<Pose> BrickStructure::getStructureCornerPoses(int brick_count, int plane) {
    if (brick_count == -1)brick_count = bricks_.size();
    std::vector<Pose> poses;
    //goes through each layer to get the corner points
    for (unsigned i = 0; i < layers_; i++) {
        std::vector<Pose> layer_poses;
        std::vector<Brick> layer_bricks = getBricksinLayer(i, brick_count);
        //iterates through each brick in the layer and gets the points
        for (auto brick:layer_bricks) {
            std::vector<Pose> brick_poses = brick.getCornerPoses();
            layer_poses.insert(std::end(layer_poses), std::begin(brick_poses), std::end(brick_poses));
        }
        //iterates through all the layer corner points and checks whether there is another
        //point close to it, if so it is not a corener
        for (unsigned a = 0; a < layer_poses.size(); a++) {
            bool close = false;
            for (unsigned b = 0; b < layer_poses.size(); b++) {
                if (a != b) {
                    double distance = Geometry::pointDistance(layer_poses[a].position, layer_poses[b].position);
                    if (distance <= brick_spacing_ + 0.001) {
                        close = true;
                        break;
                    }
                }
            }
            if (!close) poses.push_back(layer_poses[a]);
        }
    }
    //adds all points that do not overlap into final points
    std::vector<Pose> checked_poses;
    checked_poses.push_back(poses[0]);
    for (unsigned a = 0; a < poses.size(); a++) {
        bool close = false;
        for (unsigned b = 0; b < checked_poses.size(); b++) {
            double distance = Geometry::pointDistance(poses[a].position, checked_poses[b].position);
            if (distance <= brick_spacing_ + EPSILION) {
                close = true;
                break;
            }
        }
        if (!close) checked_poses.push_back(poses[a]);
    }

    if (plane == -1) return checked_poses;
    if (plane < planes_.size()) return Geometry::posesOnPlane(checked_poses, planes_[plane]);
    else {
        ROS_ERROR("PLANES OUT OF RANGE");
        return checked_poses;
    }
}

std::vector<Line> BrickStructure::getStructureLines(int plane, int brick_count) {
    if (brick_count == -1)brick_count = bricks_.size();

    // Gets both the Corner Points of the combined structure and all the separate
    // Brick corner points
    std::vector<Pose> corner_poses = getStructureCornerPoses(brick_count, plane);
    int poses_size = corner_poses.size();

    // Picks a random point from corner_points as the first and current points
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_int_distribution<int> distribution(0, corner_poses.size() - 1);
    int current_pose_idx = distribution(generator);
    int previous_pose_idx = -1;
    Pose current_pose = corner_poses[current_pose_idx];
    std::vector<unsigned> corners_found;

    // Initializes a vector of lines
    std::vector<Line> lines;
    // Loop generates lines until the size of lines is equal to the size of points
    while (lines.size() != poses_size) {
        std::vector<unsigned> vertical_pose_indexs;
        std::vector<unsigned> horizontal_pose_indexs;
        // Iterates through all plane_poses and adds all the horizontal
        // and vertical poses to the current_pose
        for (unsigned i = 0; i < corner_poses.size(); i++) {
            // Gets the distance between the poses in the z then in the xy plane
            double z_distance = Geometry::pointDistance(corner_poses[i].position, current_pose.position, false, false,
                                                        true);
            double xy_distance = Geometry::pointDistance(corner_poses[i].position, current_pose.position, true, true,
                                                         false);
            // Using distances determines whether the pose is horizontal or/and vertical
            // Ensures that the pose is not the previous or current pose
            if (i == current_pose_idx);
            else {
                if (fabs(z_distance) < EPSILION) horizontal_pose_indexs.push_back(i);
                if (fabs(xy_distance) < EPSILION) vertical_pose_indexs.push_back(i);
            }
        }
        // Get Adjacent horizontal_pose_indexs
        double left_index = -1, right_index = -1, min_left = -1000, min_right = 1000;
        for (auto horizontal_pose_index: horizontal_pose_indexs) {
            Pose temp_corner_plane_pose;
            temp_corner_plane_pose.position = current_pose.position;
            temp_corner_plane_pose.orientation = planes_[plane].pose.orientation;
            double delta = Geometry::relativePose(temp_corner_plane_pose,
                                                  corner_poses[horizontal_pose_index]).position.x;
            if (delta > 0 && delta < min_right) {
                right_index = horizontal_pose_index;
                min_right = delta;
            }
            if (delta < 0 && delta > min_left) {
                left_index = horizontal_pose_index;
                min_left = delta;
            }
        }
        // Get Adjacent vertical_pose_indexs
        double down_index = -1, up_index = -1, min_down = -1000, min_up = 1000;
        for (auto vertical_pose_index: vertical_pose_indexs) {
            double delta = Geometry::relativePose(current_pose, corner_poses[vertical_pose_index]).position.z;

            if (delta > 0 && delta < min_up) {
                up_index = vertical_pose_index;
                min_up = delta;
            }
            if (delta < 0 && delta > min_down) {
                down_index = vertical_pose_index;
                min_down = delta;
            }
        }
        // Checks 4 quadrants of the pose to see if a brick is there
        // Adds brick_spacing and epsilion
        double small_value = 0.01;

        Pose quad_1_pose = Geometry::tfPose(current_pose, brick_spacing_ + small_value, 0, small_value, 0);
        Pose quad_2_pose = Geometry::tfPose(current_pose, -(brick_spacing_ + small_value), 0, small_value, 0);
        Pose quad_3_pose = Geometry::tfPose(current_pose, -(brick_spacing_ + small_value), 0, -small_value, 0);
        Pose quad_4_pose = Geometry::tfPose(current_pose, brick_spacing_ + small_value, 0, -small_value, 0);

        std::vector<unsigned> pose_indexs;
        // Checks each quandrant and if empty adds appropriate edges
        if (pointInBrickStructure(quad_1_pose.position, brick_count) !=
            pointInBrickStructure(quad_4_pose.position, brick_count)) {
            if (right_index != -1 && std::find(pose_indexs.begin(), pose_indexs.end(), right_index) == pose_indexs.end()
                && right_index != previous_pose_idx && right_index != current_pose_idx)
                pose_indexs.push_back(right_index);
        }
        if (pointInBrickStructure(quad_2_pose.position, brick_count) !=
            pointInBrickStructure(quad_3_pose.position, brick_count)) {
            if (left_index != -1 && std::find(pose_indexs.begin(), pose_indexs.end(), left_index) == pose_indexs.end()
                && left_index != previous_pose_idx && left_index != current_pose_idx)
                pose_indexs.push_back(left_index);
        }
        if (pointInBrickStructure(quad_1_pose.position, brick_count) !=
            pointInBrickStructure(quad_2_pose.position, brick_count)) {
            if (up_index != -1 && std::find(pose_indexs.begin(), pose_indexs.end(), up_index) == pose_indexs.end()
                && up_index != previous_pose_idx && up_index != current_pose_idx)
                pose_indexs.push_back(up_index);
        }
        if (pointInBrickStructure(quad_3_pose.position, brick_count) !=
            pointInBrickStructure(quad_4_pose.position, brick_count)) {
            if (down_index != -1 && std::find(pose_indexs.begin(), pose_indexs.end(), down_index) == pose_indexs.end()
                && down_index != previous_pose_idx && down_index != current_pose_idx)
                pose_indexs.push_back(down_index);
        }
        if (pose_indexs.size() < 1) {
            ROS_ERROR("POSE INDEX SIZE NOT 1");
            break;
        } else {
            unsigned line_pose_idx = pose_indexs[0];
            // Creates line with poses and pushes back
            Line line;
            line.point_1 = current_pose.position;
            line.point_2 = corner_poses[line_pose_idx].position;
            lines.push_back(line);
            corners_found.push_back(current_pose_idx);
            current_pose = corner_poses[line_pose_idx];
            previous_pose_idx = current_pose_idx;
            current_pose_idx = line_pose_idx;

            if (std::find(corners_found.begin(), corners_found.end(), current_pose_idx) != corners_found.end() &&
                lines.size() != corner_poses.size()) {
                current_pose_idx = distribution(generator);
                bool found = true;
                while (found) {
                    if (std::find(corners_found.begin(), corners_found.end(), current_pose_idx) !=
                        corners_found.end())
                        current_pose_idx = distribution(generator);
                    else found = false;
                }
                current_pose = corner_poses[current_pose_idx];
            }
        }
    }
    return lines;
}

bool BrickStructure::pointInBrickStructure(Point point, unsigned brick_count) {
    if (brick_count == -1)brick_count = bricks_.size();
    for (unsigned i = 0; i < brick_count; i++) {
        if (bricks_[i].pointInBrick(point)) return true;
    }
    return false;
}

brick_command::BrickStructureLines BrickStructure::getStructureLinesMsg(int plane, int brick_count) {
    std::vector<Line> lines = getStructureLines(plane, brick_count);
    Pose reference_pose;
    unsigned brick_index;
    unsigned corner_index = 0;
    bool pose_found = false;
    // Iterates through all the lines to find a point that lies on z = 0 and on the plane
    // To set as reference;

    for (brick_index = 0; brick_index < brick_count; brick_index++) {
        if (getLayer(bricks_[brick_index]) == 0) {
            std::vector<Pose> brick_poses = bricks_[brick_index].getCornerPoses();
            if (Geometry::poseOnPlane(brick_poses[0], planes_[plane])) {
                corner_index = 0;
                reference_pose = brick_poses[corner_index];
                pose_found = true;
                break;
            } else if (Geometry::poseOnPlane(brick_poses[4], planes_[plane])) {
                corner_index = 4;
                reference_pose = brick_poses[corner_index];
                pose_found = true;
                break;
            }
        }
    }

    if (!pose_found) ROS_ERROR("Get Structure Lines Msg: Reference Pose Not Found");

    // Declares msg
    brick_command::BrickStructureLines msg;
    msg.brick_index = brick_index;
    msg.corner_index = corner_index;
    msg.pose = reference_pose;
    // Iterates through all the lines to get transform from reference pose
    std::vector<geometry_msgs::Vector3> h_vecs_1;
    std::vector<geometry_msgs::Vector3> h_vecs_2;
    std::vector<geometry_msgs::Vector3> v_vecs_1;
    std::vector<geometry_msgs::Vector3> v_vecs_2;
    for (auto line:lines) {
        // Computes point 1 vector
        Pose temp_pose;
        temp_pose.position = line.point_1;
        Pose relative_pose = Geometry::relativePose(reference_pose, temp_pose);
        geometry_msgs::Vector3 point_1_vec;
        point_1_vec.x = relative_pose.position.x;
        point_1_vec.y = relative_pose.position.y;
        point_1_vec.z = relative_pose.position.z;

        // Computes point 2 vector
        temp_pose.position = line.point_2;
        relative_pose = Geometry::relativePose(reference_pose, temp_pose);
        geometry_msgs::Vector3 point_2_vec;
        point_2_vec.x = relative_pose.position.x;
        point_2_vec.y = relative_pose.position.y;
        point_2_vec.z = relative_pose.position.z;

        double xyDistance = Geometry::pointDistance(line.point_1, line.point_2, true, true, false);
        double zDistance = Geometry::pointDistance(line.point_1, line.point_2, false, false, true);

        if (zDistance > xyDistance) {
            // Adds vectors to vectors
            v_vecs_1.push_back(point_1_vec);
            v_vecs_2.push_back(point_2_vec);
        } else {
            // Adds vectors to vectors
            h_vecs_1.push_back(point_1_vec);
            h_vecs_2.push_back(point_2_vec);
        }
    }
    msg.h_vecs_1 = h_vecs_1;
    msg.h_vecs_2 = h_vecs_2;
    msg.v_vecs_1 = v_vecs_1;
    msg.v_vecs_2 = v_vecs_2;
    return msg;
}

void BrickStructure::updatePoses(unsigned brick_index, unsigned corner_index, Pose pose) {
    Pose brick_pose = bricks_[brick_index].getPoseFromCornerPose(corner_index, pose);
    Pose old_brick_pose = bricks_[brick_index].getPose();
    for (unsigned i = 0; i < bricks_.size(); i++) {
        Pose relative_pose = Geometry::relativePose(old_brick_pose, bricks_[i].getPose());
        bricks_[i].setPose(Geometry::tfPose(brick_pose, relative_pose));
    }
    for (unsigned i = 0; i < planes_.size(); i++) {
        unsigned brick_index_hold = planes_[i].brick_index;
        std::vector<Pose> brick_poses = bricks_[planes_[i].brick_index].getCornerPoses();
        if (planes_[i].brick_face) {
            planes_[i] = Geometry::points2Plane(brick_poses[0], brick_poses[1], brick_poses[2]);
            planes_[i].brick_face = true;
        } else {
            planes_[i] = Geometry::points2Plane(brick_poses[4], brick_poses[5], brick_poses[6]);
            planes_[i].brick_face = false;
        }
        planes_[i].brick_index = brick_index_hold;
    }
}

std::vector<Pose> BrickStructure::getRelativePoses(unsigned brick_index, unsigned corner_index, Pose pose) {
    Pose brick_pose = bricks_[brick_index].getPoseFromCornerPose(corner_index, pose);
    Pose old_brick_pose = bricks_[brick_index].getPose();
    std::vector<Pose> poses;
    for (unsigned i = 0; i < bricks_.size(); i++) {
        Pose relative_pose = Geometry::relativePose(old_brick_pose, bricks_[i].getPose());
        poses.push_back(Geometry::tfPose(brick_pose, relative_pose));
    }
    return poses;
}

std::vector<Pose> BrickStructure::getRelativePosesTopFaceCentre(unsigned brick_index, unsigned corner_index, Pose pose) {
    Pose brick_pose = bricks_[brick_index].getPoseFromCornerPose(corner_index, pose);
    Pose old_brick_pose = bricks_[brick_index].getPose();
    std::vector<Pose> poses;
    for (unsigned i = 0; i < bricks_.size(); i++) {
        Pose relative_pose = Geometry::relativePose(old_brick_pose, bricks_[i].getPose());
        Brick temp_brick = bricks_[i];
        temp_brick.setPose(Geometry::tfPose(brick_pose, relative_pose));
        poses.push_back(temp_brick.getCentreTopFacePose());
    }
    return poses;
}

PoseArray BrickStructure::getBrickPoses(int brick_count) {
    if (brick_count == -1) brick_count = bricks_.size();

    PoseArray pose_array;
    for (unsigned i = 0; i < brick_count; i++) {
        pose_array.poses.push_back(bricks_[i].getPose());
    }
    return pose_array;
}

void BrickStructure::addBrickNoise(double noise) {
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::normal_distribution<double> distribution(0, noise);
    for(unsigned i = 0; i < bricks_.size(); i++) {
        bricks_[i].setPose(Geometry::tfPose(bricks_[i].getPose(), 0, distribution(generator), 0, 0));
    }
}


void BrickStructure::print() {
    int count = 1;
    for (auto brick:bricks_) {
        std::cout << "Brick " << count << ", color: " << brick.getColor();
        std::cout << ", POSE(x: " << brick.getPose().position.x << ", y: " << brick.getPose().position.y << ", z: "
                  << brick.getPose().position.z << ", yaw: " << Geometry::pose2Yaw(brick.getPose()) << ")" << std::endl;
        count++;
    }
    count = 1;
    for (auto plane:planes_) {
        std::cout << "Plane " << count;
        std::cout << ", POINT(x: " << plane.pose.position.x << ", y: " << plane.pose.position.y << ", z: "
                  << plane.pose.position.z << ")";
        std::cout << ", VECTOR(x: " << plane.normal.x << ", y: " << plane.normal.y << ", z: " << plane.normal.z << ")"
                  << std::endl;
        count++;
    }
}





