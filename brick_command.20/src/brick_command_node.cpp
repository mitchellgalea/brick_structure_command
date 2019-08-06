#include <iostream>
#include <string>

#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/Pose.h"
#include "ros/package.h"

#include "brick_structure/brick_structure.h"
#include "mrc_msgs/RequestBrickCommand.h"
#include "mrc_msgs/BrickCommand.h"
#include "brick_structure/geometry.h"
#include "brick_command/StructurePoseEst.h"

class BrickCommandNode
{
private:
    ros::NodeHandle nh_;
    ros::ServiceServer req_brick_command_;
    ros::ServiceClient pose_est_req_;

    brick_structure::BrickStructure brick_structure_;
    double control_max_dist_;
    double control_placement_offset_;
    std::string brick_pose_frame_;

public:
    BrickCommandNode(ros::NodeHandle nh)
        :nh_(nh)
    {
        req_brick_command_ = nh_.advertiseService("request_brick_command",&BrickCommandNode::requestBrickCommand, this);
        pose_est_req_ = nh_.serviceClient<brick_command::StructurePoseEst>("structure_pose_est");
        //input arguments
        std::string blueprint_name;
        int brick_count;
        double point_interval;
        ros::NodeHandle pn("~");
        pn.param<std::string>("brick_pose_frame", brick_pose_frame_, "");
        pn.param<std::string>("blueprint_name", blueprint_name, "test");
        pn.param<int>("brick_count", brick_count, 0);
        pn.param<double>("point_interval", point_interval, 0.001);
        pn.param<double>("control_max_dist", control_max_dist_, 0.2);
        pn.param<double>("control_placement_offset", control_placement_offset_, 0.5);

        //gets brick structure blueprint path
        std::string blueprint_path = ros::package::getPath("brick_command");
        blueprint_path.append("/blueprints/");
        blueprint_path.append(blueprint_name);
        blueprint_path.append(".yaml");

        //initialises brick_structure_
        brick_structure_ = brick_structure::BrickStructure(blueprint_path, point_interval, brick_count);
        brick_structure_.print();
    }
    bool requestBrickCommand(mrc_msgs::RequestBrickCommand::Request &req,
                             mrc_msgs::RequestBrickCommand::Response &res)
    {
        // declaration of variable in case of use
        std::vector<brick_structure::PoseStamped> stamped_poses;
        // Gets the pile poses and determines the closest placement side closest to the correct brick pile
        if(req.update_pose)
        {
            brick_command::StructurePoseEst pose_est_serv;
            brick_command::BrickStructureLines lines = brick_structure_.getStructureLinesMsg(0, -1);
            pose_est_serv.request.lines = lines;
            pose_est_serv.request.frame_id = brick_pose_frame_;
            ROS_INFO("Calling Structure Pose Estimation Service");
            if(pose_est_req_.call(pose_est_serv))
            {
                brick_structure::Pose pose = pose_est_serv.response.pose.pose;
                ROS_DEBUG_STREAM("Brick Command: Estimated Pose: Brick Index: " << lines.brick_index << "\tCorner Index: " << lines.corner_index);
                ROS_DEBUG_STREAM("Brick Command: Estimated Pose: Brick Pose: [x:" << pose.position.x << "\ty: " << pose.position.y << "\tz: " << pose.position.z << "]");
                std::vector<brick_structure::Pose> poses = brick_structure_.getRelativePosesTopFaceCentre(lines.brick_index,
                        lines.corner_index, pose);
                int count = 0;
                for(auto p:poses){
                    ROS_DEBUG_STREAM("BRICK " << count << " Pose: [x:" << p.position.x << "\ty: " << p.position.y << "\tz: " << p.position.z << "]");
                    count++;
                }
                stamped_poses = brick_structure::Geometry::posesToPoseStamped(poses, brick_pose_frame_);

                //brick_structure_.updatePoses(lines.brick_index, lines.corner_index, pose);
                //brick_structure_.print();
            }
            else
            {
                ROS_ERROR("Pose Estimation Service Failed");
            }
        }
        if(brick_structure_.structureComplete())
        {
            res.complete = true;
            return true;
        }

        brick_structure::Pose placement_offset = brick_structure::Geometry::zeroPose();
        placement_offset.position.x = -control_placement_offset_;
        mrc_msgs::BrickCommand brick_command;
        if (!req.update_pose)
            brick_command = brick_structure_.getCBrickCommand(control_max_dist_, placement_offset,
                                                                req.brick_piles, req.increment);
        else
            brick_command = brick_structure_.getCBrickCommand(control_max_dist_, placement_offset, stamped_poses,
                                                               req.brick_piles, req.increment);
        ROS_INFO_STREAM("Brick Command: Brick Command Sent - Color: " << brick_command.brick.color << "\tPosition [x:" << brick_command.brick.pose.pose.position.x << ", y:" << brick_command.brick.pose.pose.position.y << ", z:" << brick_command.brick.pose.pose.position.z << "]");
        res.complete = false;
        res.brick_command = brick_command;
        return true;
    }
};

int main(int argc, char **argv)
{
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    ros::init(argc, argv, "brick_command_node");

    ros::NodeHandle n;

    BrickCommandNode brick_command_node(n);

    ros::spin();

  return 0;
}
