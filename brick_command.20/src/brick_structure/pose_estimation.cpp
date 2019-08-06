#include "brick_structure/pose_estimation.h"

using namespace brick_structure;

PoseEstimation::PoseEstimation()
{}

Pose PoseEstimation::estimatePose(brick_command::BrickStructureLines lines, PointCloud::Ptr pointcloud,
                                  double alpha, unsigned iterations, double structure_base_z, double p_direction_max_dist)
{
    ROS_INFO_STREAM("lines.h_vecs_1.size() " << lines.h_vecs_1.size() );
    ROS_INFO_STREAM("lines.v_vecs_1.size() " << lines.v_vecs_1.size() );

    // Picks 3 random points to determine the plane
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_int_distribution<int> distribution(0,pointcloud->points.size()-1);

    ROS_INFO_STREAM("estimatePose: BEFORE distribution(generator)");

    PCPoint pc_1 = pointcloud->points[distribution(generator)];
    PCPoint pc_2 = pointcloud->points[distribution(generator)];
    PCPoint pc_3 = pointcloud->points[distribution(generator)];

    Point p_1, p_2, p_3;
    p_1.x = pc_1.x;
    p_1.y = pc_1.y;
    p_1.z = pc_1.z;
    p_2.x = pc_2.x;
    p_2.y = pc_2.y;
    p_2.z = pc_2.z;
    p_3.x = pc_3.x;
    p_3.y = pc_3.y;
    p_3.z = pc_3.z;
    Plane plane = Geometry::points2Plane(p_1, p_2, p_3);

    // Processes Horizontal and Vertical LineVecs
    std::vector<LineVec> h_linevecs;
    std::vector<LineVec> v_linevecs;
    for(unsigned i = 0; i < lines.h_vecs_1.size(); i++)
    {
        LineVec linevec;
        linevec.vec_1 = lines.h_vecs_1[i];
        linevec.vec_2 = lines.h_vecs_2[i];
        h_linevecs.push_back(linevec);
    }
    for(unsigned i = 0; i < lines.v_vecs_1.size(); i++)
    {
        LineVec linevec;
        linevec.vec_1 = lines.v_vecs_1[i];
        linevec.vec_2 = lines.v_vecs_2[i];
        v_linevecs.push_back(linevec);
    }

    // Gets the Horizonal and Vertical Points from Point cloud
    std::vector<Point> h_points;
    std::vector<Point> v_points;
    for(unsigned i = 0; i < pointcloud->points.size(); i++)
    {
        Point point;
        point.x = pointcloud->points[i].x;
        point.y = pointcloud->points[i].y;
        point.z = pointcloud->points[i].z;
        direction dir = Geometry::getPointDirection(pointcloud, p_direction_max_dist, i);
        if(dir == direction::horizontal) h_points.push_back(point);
        else v_points.push_back(point);
    }

    ROS_INFO_STREAM("h_points.size() " << h_points.size() << ", v_points.size() " << v_points.size() );
//
//    // Checks each horizontal point to ensure its not outside the vertical extrema
//    double min = Geometry::relativePose(plane.pose, v_points[0]).position.x;
//    double max = min;
//    for( auto point : v_points )
//    {
//        double relative_dist = Geometry::relativePose(plane.pose, point).position.x;
//        if (relative_dist < min) min = relative_dist;
//        if (relative_dist > max) max = relative_dist;
//    }
//    ROS_INFO_STREAM("for auto v_points" );
//
//    std::vector<Point> checked_h_points;
//    for(auto point : h_points)
//    {
//        double relative_dist = Geometry::relativePose(plane.pose, point).position.x;
//        if(relative_dist > min && relative_dist < max) checked_h_points.push_back(point);
//    }
//    ROS_INFO_STREAM("for auto h_points" );
//    h_points = checked_h_points;

    // Gets the 2 input poses by projecting onto plane
    Point point = Geometry::projectPointOnPlane(lines.pose.position, plane);
    point.z = structure_base_z;
    Pose pose_1 = plane.pose;
    Pose pose_2 = Geometry::tfPose(pose_1, 0, 0, 0, M_PI);

    ROS_INFO_STREAM("BEFORE gradientDescentPoseMatch");

    return gradientDescentPoseMatch(h_points, v_points, h_linevecs, v_linevecs,
                                         pose_1, pose_2, alpha, iterations);
}

Pose PoseEstimation::gradientDescentPoseMatch(std::vector<Point> h_ps, std::vector<Point> v_ps,
                                  std::vector<LineVec> h_lvs, std::vector<LineVec> v_lvs,
                                  Pose p1, Pose p2, double alpha, unsigned iterations)
{
    std::vector<Pose> poses = {p1,p2};
    std::vector<double> p_costs;
    // Calculates for input poses
    for(unsigned i = 0; i < poses.size(); i++)
    {
        std::vector<Line> h_ls;
        std::vector<Line> v_ls;

        for(unsigned it = 0; it < iterations; it++)
        {
            // Calculates Current Gradient by transforming Pose to left and right by
            // Small amount
            Pose p_p = Geometry::tfPose(poses[i], 0.0001, 0, 0, 0);
            Pose p_n = Geometry::tfPose(poses[i], -0.0001, 0, 0, 0);

            Geometry::shiftLines(p_p, h_lvs, v_lvs, h_ls, v_ls);
            double cost_p = Geometry::linePointsDistanceOutput(h_ps, v_ps, h_ls, v_ls);
            Geometry::shiftLines(p_n, h_lvs, v_lvs, h_ls, v_ls);
            double cost_n = Geometry::linePointsDistanceOutput(h_ps, v_ps, h_ls, v_ls);

            double gradient = cost_n - cost_p;
            poses[i] = Geometry::tfPose(poses[i], alpha*gradient, 0, 0, 0);
        }
        Geometry::shiftLines(poses[i], h_lvs, v_lvs, h_ls, v_ls);
        p_costs.push_back(Geometry::linePointsDistanceOutput(h_ps, v_ps, h_ls, v_ls));
    }
    ROS_INFO_STREAM("gradientDescentPoseMatch: after 1st for loop");

    Pose output_pose = poses[0];
    double min_cost = p_costs[0];
    for(unsigned i = 0; i < poses.size(); i++)
    {
        std::cout << "Cost: " << p_costs[i] << std::endl;
        if(p_costs[i] < min_cost)
        {
            output_pose = poses[i];
            min_cost = p_costs[i];
        }
    }
    ROS_INFO_STREAM("gradientDescentPoseMatch: after 2nd for loop");
    return output_pose;
}

bool PoseEstimation::pointCloudHull(PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, double leaf_size,
                                    int seg_max_it, double seg_dist_thresh, double seg_eps_angle, double hull_alpha, double v_FOV,double h_FOV,
                                    double near_plane_dist, double far_plane_dist, double radius, int rad_n) {

    // Declaration of pcl pointcloud pointers that will be used
    PointCloud2::Ptr voxel_cloud_2 (new PointCloud2);
    PointCloud::Ptr voxel_cloud (new PointCloud), hull_cloud (new PointCloud), tf_cloud (new PointCloud), rad_cloud (new PointCloud),
            plane_cloud (new PointCloud), projected_cloud (new PointCloud), z_pass_cloud (new PointCloud);

    ROS_INFO_STREAM("Pose Estimation: Point Cloud Hull Started");
    //Voxel Grid filter
    pcl::ApproximateVoxelGrid<PCPoint> voxel_grid;
    voxel_grid.setInputCloud(input_cloud);
    voxel_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
    voxel_grid.filter (*voxel_cloud);
    ROS_INFO_STREAM("Pose Estimation: Voxel Grid Started, Input Size: " << input_cloud->points.size()
    << "\t Output Size: " << voxel_cloud->points.size());

    // Z PassThroughfilter to increase likelihood of correct plane
    pcl::PassThrough<PCPoint> pass;
    pass.setInputCloud (voxel_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, far_plane_dist);
    pass.filter (*z_pass_cloud);
    if(z_pass_cloud->points.empty())return false;
    ROS_INFO_STREAM("Pose Estimation: Z Pass Started, Input Size: " << voxel_cloud->points.size()
    << "\t Output Size: " << z_pass_cloud->points.size());

    // Coefficents and Inliers for Plane Segmentation
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object to filter for planes parallel to the y axis
    pcl::SACSegmentation<PCPoint> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (seg_max_it);
    seg.setDistanceThreshold (seg_dist_thresh);
    seg.setAxis(Eigen::Vector3f(0,1,0));
    seg.setEpsAngle (seg_eps_angle * (M_PI/180));
    seg.setInputCloud (z_pass_cloud);
    seg.segment (*inliers, *coefficients);

    // Create the filtering object
    pcl::ExtractIndices<PCPoint> extract;
    extract.setInputCloud (z_pass_cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*plane_cloud);
    ROS_INFO_STREAM("Pose Estimation: Plane Segmentation Started, Input Size: " << z_pass_cloud->points.size()
    << "\t Output Size: " << plane_cloud->points.size());
    if(plane_cloud->points.empty())return false;

    // Project all the points onto the plane of best fit
    pcl::ProjectInliers<PCPoint> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (plane_cloud);
    proj.setModelCoefficients (coefficients);
    proj.filter (*projected_cloud);
    ROS_INFO_STREAM("Pose Estimation: Plane Projection Started, Input Size: " << plane_cloud->points.size()
    << "\t Output Size: " << projected_cloud->points.size());

    // PCL Radius Outlier Removal
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(projected_cloud);
    outrem.setRadiusSearch(radius);
    outrem.setMinNeighborsInRadius (rad_n);
    // apply filter
    outrem.filter (*rad_cloud);
    ROS_INFO_STREAM("Pose Estimation: Radius Outlier Removal, Input Size: " << projected_cloud->points.size()
                                                                          << "\t Output Size: " << rad_cloud->points.size());

    // Concave Hull filtering
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud (rad_cloud);
    chull.setAlpha(hull_alpha);
    chull.reconstruct (*hull_cloud);
    ROS_INFO_STREAM("Pose Estimation: Concave Hull Started, Input Size: " << rad_cloud->points.size()
    << "\t Output Size: " << hull_cloud->points.size());

    ROS_INFO_STREAM("Pose Estimation:\th_FOV: " << h_FOV << "\tv_FOV: " << v_FOV);
    // Filters for Feild of View
//    pcl::FrustumCulling<PCPoint> fc;
//    fc.setInputCloud (hull_cloud);
//    fc.setVerticalFOV (v_FOV);
//    fc.setHorizontalFOV (h_FOV);
//    fc.setNearPlaneDistance (near_plane_dist);
//    fc.setFarPlaneDistance (far_plane_dist);
//    Eigen::Matrix4f camera_pose;
//    camera_pose <<  0, 0,-1, 0,
//            0, 1, 0, 0,
//            1, 0, 0, 0,
//            0, 0, 0, 1;
//    fc.setCameraPose (camera_pose);
//    fc.filter(*output_cloud);
    for(auto&& point:hull_cloud->points){
        double h_angle = atan(point.x/point.z);
        double v_angle = atan(point.y/point.z);
        if(fabs(h_angle*180/M_PI) < h_FOV/2 && fabs(v_angle*180/M_PI) < v_FOV/2){
            output_cloud->points.push_back(point);
        }
    }
    output_cloud->width = output_cloud->points.size();
    output_cloud->height = 1;
    ROS_INFO_STREAM("Pose Estimation: Frustum Culling Started, Input Size: " << hull_cloud->points.size()
    << "\t Output Size: " << output_cloud->points.size());
    if(output_cloud->points.empty())return false;
    return true;
}
