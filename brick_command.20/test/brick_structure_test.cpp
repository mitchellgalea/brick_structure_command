#include <gtest/gtest.h>
#include <climits>

#include <ros/package.h> //This tool allows to identify the path of the package on your system
#include "brick_structure/brick_structure.h"
#include "brick_structure/geometry.h"

const double NEAR = 0.001;

using namespace brick_structure;

/* THE FOLLOWING TESTS WILL TEST THE GEOMETRY FUNCTIONS */

TEST(Geometry, pose2d2Pose){

    Pose2D input_pose_2D;
    input_pose_2D.theta = M_PI/4;
    Pose input_pose = Geometry::pose2d2Pose(input_pose_2D);

    ASSERT_NEAR(0, input_pose.position.x, NEAR);
    ASSERT_NEAR(0, input_pose.position.y, NEAR);
    ASSERT_NEAR(0, input_pose.position.z, NEAR);
    ASSERT_NEAR(0.92388, input_pose.orientation.w, NEAR);
    ASSERT_NEAR(0.0, input_pose.orientation.x, NEAR);
    ASSERT_NEAR(0.0, input_pose.orientation.y, NEAR);
    ASSERT_NEAR(0.38268, input_pose.orientation.z, NEAR);

    Pose2D tf_pose_2D;
    tf_pose_2D.theta = M_PI/4;
    tf_pose_2D.x = 1;
    tf_pose_2D.y = 1.5;
    Pose tf_pose = Geometry::pose2d2Pose(tf_pose_2D);
    tf_pose.position.z = 2;

    ASSERT_NEAR(1, tf_pose.position.x, NEAR);
    ASSERT_NEAR(1.5, tf_pose.position.y, NEAR);
    ASSERT_NEAR(2, tf_pose.position.z, NEAR);
    ASSERT_NEAR(0.92388, tf_pose.orientation.w, NEAR);
    ASSERT_NEAR(0.0, tf_pose.orientation.x, NEAR);
    ASSERT_NEAR(0.0, tf_pose.orientation.y, NEAR);
    ASSERT_NEAR(0.38268, tf_pose.orientation.z, NEAR);
}
TEST(Geometry, tfPose){

    Pose pose = Geometry::zeroPose();
    Pose pose_tf_1 = Geometry::zeroPose();
    pose_tf_1.position.x = 1.0;
    pose_tf_1.position.y = 1.2;
    pose_tf_1.position.z = 1.8;
    Pose tf_pose_1 = Geometry::tfPose(pose, pose_tf_1);
    ASSERT_NEAR(1.0, tf_pose_1.position.x, NEAR);
    ASSERT_NEAR(1.2, tf_pose_1.position.y, NEAR);
    ASSERT_NEAR(1.8, tf_pose_1.position.z, NEAR);
    ASSERT_NEAR(1.0, tf_pose_1.orientation.w, NEAR);
    ASSERT_NEAR(0.0, tf_pose_1.orientation.x, NEAR);
    ASSERT_NEAR(0.0, tf_pose_1.orientation.y, NEAR);
    ASSERT_NEAR(0.0, tf_pose_1.orientation.z, NEAR);


    Pose2D input_pose_2D;
    input_pose_2D.theta = M_PI/4;
    Pose input_pose = Geometry::pose2d2Pose(input_pose_2D);

    Pose2D tf_pose_2D;
    tf_pose_2D.theta = M_PI/4;
    tf_pose_2D.x = 1;
    tf_pose_2D.y = 1.5;
    Pose tf_pose = Geometry::pose2d2Pose(tf_pose_2D);
    tf_pose.position.z = 2;

    Pose new_pose_1 = Geometry::tfPose(input_pose, 1, 1.5, 2, M_PI/4);
    Pose new_pose_2 = Geometry::tfPose(input_pose, tf_pose);

    ASSERT_NEAR(-0.3536, new_pose_1.position.x, NEAR);
    ASSERT_NEAR(1.7678, new_pose_1.position.y, NEAR);
    ASSERT_NEAR(2, new_pose_1.position.z, NEAR);
    ASSERT_NEAR(0.70711, new_pose_1.orientation.w, NEAR);
    ASSERT_NEAR(0.0, new_pose_1.orientation.x, NEAR);
    ASSERT_NEAR(0.0, new_pose_1.orientation.y, NEAR);
    ASSERT_NEAR(0.70711, new_pose_1.orientation.z, NEAR);

    ASSERT_NEAR(-0.3536, new_pose_2.position.x, NEAR);
    ASSERT_NEAR(1.7678, new_pose_2.position.y, NEAR);
    ASSERT_NEAR(2, new_pose_2.position.z, NEAR);
    ASSERT_NEAR(0.70711, new_pose_2.orientation.w, NEAR);
    ASSERT_NEAR(0.0, new_pose_2.orientation.x, NEAR);
    ASSERT_NEAR(0.0, new_pose_2.orientation.y, NEAR);
    ASSERT_NEAR(0.70711, new_pose_2.orientation.z, NEAR);

}
TEST(Geometry, pose2Yaw){
    Pose2D input_pose_2D;
    input_pose_2D.theta = M_PI/4;
    Pose input_pose = Geometry::pose2d2Pose(input_pose_2D);
    double yaw = Geometry::pose2Yaw(input_pose);
    ASSERT_NEAR(M_PI/4, yaw, NEAR);
}
TEST(Geometry, relativePose){
    Pose2D input_pose_2D;
    input_pose_2D.theta = M_PI/4;
    Pose input_pose = Geometry::pose2d2Pose(input_pose_2D);

    Pose2D tf_pose_2D;
    tf_pose_2D.theta = M_PI/4;
    tf_pose_2D.x = 1;
    tf_pose_2D.y = 1.5;
    Pose tf_pose = Geometry::pose2d2Pose(tf_pose_2D);
    tf_pose.position.z = 2;

    Pose new_pose = Geometry::tfPose(input_pose, tf_pose);

    Pose relative_pose = Geometry::relativePose(input_pose, new_pose);
    Pose relative_pose_point = Geometry::relativePose(input_pose, new_pose.position);

    ASSERT_NEAR(1, relative_pose.position.x, NEAR);
    ASSERT_NEAR(1.5, relative_pose.position.y, NEAR);
    ASSERT_NEAR(2, relative_pose.position.z, NEAR);
    ASSERT_NEAR(0.92388, relative_pose.orientation.w, NEAR);
    ASSERT_NEAR(0.0, relative_pose.orientation.x, NEAR);
    ASSERT_NEAR(0.0, relative_pose.orientation.y, NEAR);
    ASSERT_NEAR(0.38268, relative_pose.orientation.z, NEAR);

    ASSERT_NEAR(1, relative_pose_point.position.x, NEAR);
    ASSERT_NEAR(1.5, relative_pose_point.position.y, NEAR);
    ASSERT_NEAR(2, relative_pose_point.position.z, NEAR);
}
TEST(Geometry, points2Plane){
    // Generate 3 points
    Point point_1;
    Point point_2;
    Point point_3;

    point_2.x = 2.0;
    point_2.y = 2.0;

    point_3.x = 2.0;
    point_3.y = 2.0;
    point_3.z = 1.0;

    Plane plane = Geometry::points2Plane(point_1, point_2, point_3);
    ASSERT_NEAR(point_1.x, plane.pose.position.x, NEAR);
    ASSERT_NEAR(point_1.y, plane.pose.position.y, NEAR);
    ASSERT_NEAR(point_1.z, plane.pose.position.z, NEAR);
    ASSERT_NEAR(0.707107, plane.normal.x, NEAR);
    ASSERT_NEAR(-0.707107, plane.normal.y, NEAR);
    ASSERT_NEAR(0.0, plane.normal.z, NEAR);

}
TEST(Geometry, pointOnPlane){

}
TEST(Geometry, pointDistance){

}

/* THE FOLLOWING TESTS WILL TEST THE BRICK FUNCTIONS */
TEST(Brick, constructorBrickDimensions){
    Pose pose;
    Brick red_brick = Brick('R', pose);
    Brick green_brick = Brick('G', pose);
    Brick blue_brick = Brick('B', pose);
    Brick orange_brick = Brick('O', pose);

    ASSERT_NEAR(0.3, red_brick.getXDim(), NEAR);
    ASSERT_NEAR(0.2, red_brick.getYDim(), NEAR);
    ASSERT_NEAR(0.2, red_brick.getZDim(), NEAR);

    ASSERT_NEAR(0.6, green_brick.getXDim(), NEAR);
    ASSERT_NEAR(0.2, green_brick.getYDim(), NEAR);
    ASSERT_NEAR(0.2, green_brick.getZDim(), NEAR);

    ASSERT_NEAR(1.2, blue_brick.getXDim(), NEAR);
    ASSERT_NEAR(0.2, blue_brick.getYDim(), NEAR);
    ASSERT_NEAR(0.2, blue_brick.getZDim(), NEAR);

    ASSERT_NEAR(1.8, orange_brick.getXDim(), NEAR);
    ASSERT_NEAR(0.2, orange_brick.getYDim(), NEAR);
    ASSERT_NEAR(0.2, orange_brick.getZDim(), NEAR);
}

TEST(Brick, getPose){
    Pose2D pose_2d;
    pose_2d.theta = M_PI/4;

    Pose pose = Geometry::pose2d2Pose(pose_2d);
    Brick red_brick = Brick('R', pose);
    Pose red_pose = red_brick.getPose();

    ASSERT_NEAR(0.0, red_pose.position.x, NEAR);
    ASSERT_NEAR(0.0, red_pose.position.y, NEAR);
    ASSERT_NEAR(0.0, red_pose.position.z, NEAR);
    ASSERT_NEAR(0.92388, red_pose.orientation.w, NEAR);
    ASSERT_NEAR(0.0, red_pose.orientation.x, NEAR);
    ASSERT_NEAR(0.0, red_pose.orientation.y, NEAR);
    ASSERT_NEAR(0.38268, red_pose.orientation.z, NEAR);
}
TEST(Brick, getCentrePose){
    Pose2D pose_2d;
    pose_2d.theta = M_PI/4;

    Pose pose = Geometry::pose2d2Pose(pose_2d);
    Brick red_brick = Brick('R', pose);
    Pose red_centre_pose = red_brick.getCentrePose();

    ASSERT_NEAR(0.0354, red_centre_pose.position.x, NEAR);
    ASSERT_NEAR(0.1768, red_centre_pose.position.y, NEAR);
    ASSERT_NEAR(0.1, red_centre_pose.position.z, NEAR);
    ASSERT_NEAR(0.92388, red_centre_pose.orientation.w, NEAR);
    ASSERT_NEAR(0.0, red_centre_pose.orientation.x, NEAR);
    ASSERT_NEAR(0.0, red_centre_pose.orientation.y, NEAR);
    ASSERT_NEAR(0.38268, red_centre_pose.orientation.z, NEAR);
}
TEST(Brick, getCornerPoints){
    Pose2D pose_2d;
    pose_2d.theta = M_PI/4;

    Pose pose = Geometry::pose2d2Pose(pose_2d);
    Brick red_brick = Brick('R', pose);
    std::vector<Point> brick_points = red_brick.getCornerPoints();
    std::vector<double> x_points = {0.0, 0.0, 0.2121, 0.2121, -0.1414, -0.1414, 0.0707, 0.0707};
    std::vector<double> y_points = {0.0, 0.0, 0.2121, 0.2121, 0.1414, 0.1414, 0.3536, 0.3536};
    std::vector<double> z_points = {0.0, 0.2, 0.2, 0.0, 0.0, 0.2, 0.2, 0.0};

    ASSERT_EQ(8, brick_points.size());
    for(unsigned i = 0; i < brick_points.size(); i++)
    {
        ASSERT_NEAR(x_points[i], brick_points[i].x, NEAR);
        ASSERT_NEAR(y_points[i], brick_points[i].y, NEAR);
        ASSERT_NEAR(z_points[i], brick_points[i].z, NEAR);
    }
}
TEST(Brick, getPoseFromCornerPose){
    Pose pose = Geometry::zeroPose();
    Brick red_brick = Brick('R', pose);
    std::vector<Pose> brick_poses = red_brick.getCornerPoses();
    for(unsigned i = 0; i < 8; i++) {
        Pose temp_pose = red_brick.getPoseFromCornerPose(i, brick_poses[i]);
        ASSERT_NEAR(pose.position.x, temp_pose.position.x, NEAR);
        ASSERT_NEAR(pose.position.y, temp_pose.position.y, NEAR);
        ASSERT_NEAR(pose.position.z, temp_pose.position.z, NEAR);
        ASSERT_NEAR(pose.orientation.w, temp_pose.orientation.w, NEAR);
        ASSERT_NEAR(pose.orientation.x, temp_pose.orientation.x, NEAR);
        ASSERT_NEAR(pose.orientation.y, temp_pose.orientation.y, NEAR);
        ASSERT_NEAR(pose.orientation.z, temp_pose.orientation.z, NEAR);
    }
}

/* THE FOLLOWING TESTS WILL TEST THE BRICK STRUCTURE FUNCTIONS */
TEST(BrickStructure, constructor)
{
    //gets brick structure blueprint path
    std::string blueprint_path = ros::package::getPath("brick_command");
    blueprint_path.append("/test/blueprints-test/test.yaml");

    //initialises brick_structure_
    BrickStructure brick_structure = brick_structure::BrickStructure(blueprint_path, 0.01);
    brick_structure::PoseArray pose_array = brick_structure.getBrickPoses();

    std::vector<double> x_positions = {1.0, 1.2685, 2.8360, 1.0866, 1.6149, 2.6628, 1.2598, 2.3077};
    std::vector<double> y_positions = {2.0, 2.1550, 3.0600, 2.0500, 2.3550, 2.9600, 2.1500, 2.7550};
    std::vector<double> z_positions = {0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.4, 0.4};
    double orient_w = 0.96593;
    double orient_x = 0.0;
    double orient_y = 0.0;
    double orient_z = 0.25882;

    for(unsigned i = 0; i < pose_array.poses.size(); i++)
    {
        ASSERT_NEAR(x_positions[i], pose_array.poses[i].position.x, NEAR);
        ASSERT_NEAR(y_positions[i], pose_array.poses[i].position.y, NEAR);
        ASSERT_NEAR(z_positions[i], pose_array.poses[i].position.z, NEAR);
        ASSERT_NEAR(orient_w, pose_array.poses[i].orientation.w, NEAR);
        ASSERT_NEAR(orient_x, pose_array.poses[i].orientation.x, NEAR);
        ASSERT_NEAR(orient_y, pose_array.poses[i].orientation.y, NEAR);
        ASSERT_NEAR(orient_z, pose_array.poses[i].orientation.z, NEAR);
    }
}
TEST(BrickStructure, updatePoses)
{

}
TEST(BrickStructure, getRelativePoses)
{

}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "brick_structure_test");
    return RUN_ALL_TESTS();
}
