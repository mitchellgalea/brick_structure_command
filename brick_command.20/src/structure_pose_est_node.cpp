#include <iostream>
#include <string>
#include <fstream>
#include <boost/filesystem.hpp>
#include <iomanip>
#include <mutex>

#include "tf/transform_listener.h"
#include <tf2_ros/transform_listener.h>
#include "tf_conversions/tf_eigen.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include <Eigen/Geometry>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include "brick_command/StructurePoseEst.h"
#include "brick_command/BrickStructureLines.h"
#include "brick_structure/geometry.h"
#include "brick_structure/pose_estimation.h"

using namespace brick_structure;

class StructurePoseEst {
private:
    ros::NodeHandle nh_;
    ros::ServiceServer structure_pose_est_serv_;
    ros::Subscriber pc_sub_;
    ros::Publisher pc_pub_;
    tf::TransformListener tf_listener_;
    tf::Transform transform_;
    tf2_ros::Buffer tf2Buffer_;
    tf2_ros::TransformListener tf2Listener_;

    std::mutex mtx_;
    sensor_msgs::PointCloud2ConstPtr pc_msg_;
    PointCloud::Ptr pc_processed_;

    // point cloud processing variables
    double leaf_size_;
    double seg_dist_thresh_;
    double seg_eps_angle_;
    double hull_alpha_;
    double h_FOV_;
    double v_FOV_;
    double near_plane_dist_;
    double far_plane_dist_;
    double radius_;
    int rad_n_;

    int seg_max_it_;
    std::string file_;

    // pose estimation variables
    double grad_alpha_;
    int grad_iterations_;
    double structure_base_z_;
    double pose_dir_max_d_;
    std::string frame_id_;
    std::string pc_frame_id_;
    // booleans
    bool sub_pub_;


    bool fetchTransform(tf::Transform &transform) {
        geometry_msgs::TransformStamped local_transformStamped;
        try {
            ROS_DEBUG_STREAM(
                    "Structure Pose Est: pc_frame: " << pc_frame_id_ << "\tframe_id: " << frame_id_);
            local_transformStamped = tf2Buffer_.lookupTransform(frame_id_, pc_frame_id_,
                                                                ros::Time(0));
            tf::transformMsgToTF(local_transformStamped.transform, transform);
            return true;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return false;
        }
    }

    // This function processes the class pc_msg_ member and finds the largest
    // plane parallel to the camera y axis, then projects all the points
    // back onto that plane, then computes the hull
    bool processPointCloud(bool publish, std::string frame) {
        // Declaration of pcl pointcloud pointers that will be used
        PointCloud2::Ptr cloud(new PointCloud2);
        PointCloud::Ptr output_cloud(new PointCloud), temp_cloud(new PointCloud);

        // Convert ros msg to pcl object
        //pcl_conversions::toPCL(*pc_msg_, *cloud);
        pcl::fromROSMsg(*pc_msg_, *temp_cloud);
        if (!PoseEstimation::pointCloudHull(temp_cloud, output_cloud, leaf_size_, seg_max_it_, seg_dist_thresh_,
                                            seg_eps_angle_, hull_alpha_, v_FOV_, h_FOV_, near_plane_dist_, far_plane_dist_,
                                            radius_, rad_n_))
            return false;
        ROS_INFO_STREAM("Point Cloud Hull Output Cloud Size: " << output_cloud->points.size());
        // Transforms the Brick Structure Hull from Camera Frame to World Frame
        if (!output_cloud->points.empty()) {
            try {
                if (fetchTransform(transform_)) {
                    pcl_ros::transformPointCloud(*output_cloud, *pc_processed_, transform_);
                    pc_processed_->header.frame_id = frame_id_;
                    ROS_INFO_STREAM("TF SUCCESSFUL, SIZE: " << pc_processed_->points.size());
                    if (publish && !pc_processed_->points.empty()) {
                        pc_pub_.publish(*pc_processed_);
                        ROS_INFO_STREAM("PUBLISHING");
                    }
                    return true;
                } else {
                    ROS_ERROR("Fetching transform from tf2 buffer failed");
                }
            }
            catch (tf::TransformException &ex) {
                ROS_WARN("%s", ex.what());
            };

        }
        return false;
    }

public:
    StructurePoseEst(ros::NodeHandle nh)
            : nh_(nh), tf2Listener_(tf2Buffer_), pc_processed_(new PointCloud) {
        std::string pc_topic;

        ros::NodeHandle pn("~");
        pn.param<std::string>("pc_topic", pc_topic, "/sensor_d435_front/depth/points");
        pn.param<std::string>("frame_id", frame_id_, "base_footprint");
        pn.param<double>("h_FOV", h_FOV_, 70);
        pn.param<double>("v_FOV", v_FOV_, 40);
        pn.param<double>("near_plane_dist", near_plane_dist_, 0.3);
        pn.param<double>("far_plane_dist", far_plane_dist_, 1.0);
        pn.param<double>("leaf_size", leaf_size_, 0.015);
        pn.param<double>("seg_dist_thresh", seg_dist_thresh_, 0.015);
        pn.param<double>("seg_eps_angle", seg_eps_angle_, 5.0);
        pn.param<double>("hull_alpha", hull_alpha_, 0.03);
        pn.param<int>("seg_max_it", seg_max_it_, 100);
        pn.param<bool>("sub_pub", sub_pub_, false);
        pn.param<double>("grad_alpha", grad_alpha_, 8.0);
        pn.param<int>("grad_iterations", grad_iterations_, 100);
        pn.param<double>("structure_base_z_", structure_base_z_, 0.0);
        pn.param<double>("pose_dir_max_d", pose_dir_max_d_, 0.1);
        pn.param<double>("radius", radius_, 0.1);
        pn.param<int>("rad_n", rad_n_, 20);

        ROS_DEBUG_STREAM("h_FOV: " << h_FOV_ << "\tv_FOV: " << v_FOV_);

        pc_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(pc_topic, 1, &StructurePoseEst::pcCallback, this);
        pc_pub_ = nh_.advertise<PointCloud>("/brick_structure_hull", 1);
        structure_pose_est_serv_ = nh_.advertiseService("structure_pose_est",
                                                        &StructurePoseEst::structurePoseEst, this);
    }

    void pcCallback(const sensor_msgs::PointCloud2ConstPtr &pc_msg) {
        // Locks the mutex and sets pc_msg_
        std::unique_lock<std::mutex> lck(mtx_);
        pc_frame_id_ = pc_msg->header.frame_id;
        pc_msg_ = pc_msg;
        if (sub_pub_) processPointCloud(true, frame_id_);
    }

    bool structurePoseEst(brick_command::StructurePoseEst::Request &req,
                          brick_command::StructurePoseEst::Response &res) {
        std::unique_lock<std::mutex> lck(mtx_);
        ROS_INFO_STREAM("Service Called");
        processPointCloud(true, req.frame_id);

        std::string path = ros::package::getPath("brick_command");
        path.append("/processed.pcd");
        pcl::io::savePCDFileASCII(path, *pc_processed_);
        ROS_INFO_STREAM("POINT CLOUD PROCESSING SUCCESSFUL");
        res.pose.pose = PoseEstimation::estimatePose(req.lines, pc_processed_, grad_alpha_,
                                                     grad_iterations_, structure_base_z_, pose_dir_max_d_);
        ROS_INFO_STREAM("POSE ESTIMATION SUCCESSFUL");
        res.pose.header.frame_id = req.frame_id;
        res.pose.header.stamp = ros::Time::now();
        return true;
    }


};

int main(int argc, char **argv) {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    ros::init(argc, argv, "structure_pose_est_node");
    ros::NodeHandle nh;
    StructurePoseEst structure_pose_est_node(nh);
    ros::spin();
    return 0;
}
