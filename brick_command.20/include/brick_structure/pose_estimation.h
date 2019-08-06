#ifndef POSEESTIMATION_H
#define POSEESTIMATION_H

#include <utility>
#include <vector>
#include <random>
#include <chrono>
#include <iostream>
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/model_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/Geometry>
#include "tf_conversions/tf_eigen.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"

#include "tf2/LinearMath/Vector3.h"
#include "brick_structure/geometry.h"
#include "brick_command/BrickStructureLines.h"

namespace brick_structure {

class PoseEstimation
{
private:
    PoseEstimation();

    static Pose gradientDescentPoseMatch(std::vector<Point> h_ps, std::vector<Point> v_ps,
                                         std::vector<LineVec> h_lvs, std::vector<LineVec> v_lvs,
                                         Pose p1, Pose p2, double alpha, unsigned iterations);
public:
    static Pose estimatePose(brick_command::BrickStructureLines lines, PointCloud::Ptr pointcloud,
                             double alpha, unsigned iterations, double structure_base_z, double p_direction_max_dist);
    static bool pointCloudHull(PointCloud::Ptr  input_cloud, PointCloud::Ptr output_cloud, double leaf_size,
                               int seg_max_it, double seg_dist_thresh, double seg_eps_angle, double hull_alpha, double v_FOV,double h_FOV,
                               double near_plane_dist, double far_plane_dist, double radius, int rad_n);
};

}

#endif //POSEESTIMATION_H
