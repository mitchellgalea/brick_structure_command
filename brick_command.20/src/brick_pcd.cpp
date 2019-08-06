
#include "ros/ros.h"
#include "ros/package.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include "brick_structure/geometry.h"
#include "brick_structure/brick.h"

#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <random>
#include <math.h>
#include <algorithm>

typedef pcl::PointNormal PointN;
typedef pcl::PointCloud<PointN> PointCloudN;

using namespace brick_structure;

struct PNormDirs{
    PNormDirs(int x, int y, int z): normal_x(x), normal_y(y), normal_z(z){}
    int normal_x;
    int normal_y;
    int normal_z;
};

////Point Cloud Filter Node Class
class BrickPCD {
////ROS MEMBERS
    ros::NodeHandle nh_;

    void pcdFromBrickCorners(PointCloudN &cloud, Brick &brick)
    {
        // Get Brick Corner point
        std::vector<Point> points = brick.getCornerPoints();
        // Fill the cloud with data
        cloud.width    = 24;
        cloud.height   = 1;
        cloud.is_dense = false;
        cloud.points.resize (cloud.width * cloud.height);

        std::vector<PNormDirs> norms = {PNormDirs(-1, -1, -1), PNormDirs(-1, -1, 1), PNormDirs(1, -1, 1), PNormDirs(1, -1, -1),
                                        PNormDirs(-1, 1, -1), PNormDirs(-1, 1, 1), PNormDirs(1, 1, 1), PNormDirs(1, 1, -1)};
        // Set the Cloud Points
        int dim = 3;
        for(unsigned i = 0; i < points.size(); i++)
        {
            for(unsigned d = 0; d < dim; d++)
            {
                cloud.points[d+i*dim].x = points[i].x;
                cloud.points[d+i*dim].y = points[i].y;
                cloud.points[d+i*dim].z = points[i].z;
            }
            cloud.points[i*dim].normal_x = norms[i].normal_x;
            cloud.points[i*dim].normal_y = 0;
            cloud.points[i*dim].normal_z = 0;
            cloud.points[i*dim + 1].normal_x = 0;
            cloud.points[i*dim + 1].normal_y = norms[i].normal_y;
            cloud.points[i*dim + 1].normal_z = 0;
            cloud.points[i*dim + 2].normal_x = 0;
            cloud.points[i*dim + 2].normal_y = 0;
            cloud.points[i*dim + 2].normal_z = norms[i].normal_z;
        }
    }
    void pcdFromBrickFaces(PointCloudN &cloud, Brick &brick, std::vector<int> ignore_idx, double interval, double noise)
    {
        // Sets Plane Variables
        std::vector<std::vector<int>> plane_norms = {std::vector<int>{0, -1, 0}, std::vector<int>{0, 0, 1}, std::vector<int>{0, 1, 0},
                                                     std::vector<int>{0, 0, -1}, std::vector<int>{-1, 0, 0}, std::vector<int>{1, 0, 0}};
        std::vector<std::vector<int>> p_idxs = {std::vector<int>{0,1,2,3},std::vector<int>{1,5,6,2},std::vector<int>{4,5,6,7},
                                                std::vector<int>{0,4,7,3},std::vector<int>{0,1,5,4},std::vector<int>{6,7,2,3},};
        std::vector<Point> points = brick.getCornerPoints();

        for(unsigned i = 0; i < plane_norms.size(); i++)
        {
            if(std::find(ignore_idx.begin(), ignore_idx.end(), i) == ignore_idx.end()) {
                std::vector<double> limits = getPlaneLimits(p_idxs[i], points);
                if(fabs(limits[0] - limits[1]) < EPSILION)
                {
                    for(double y = limits[2]; y <= limits[3]+EPSILION; y = y + interval)
                    {
                        for(double z = limits[4]; z <= limits[5]+EPSILION; z = z + interval)
                        {
                            PointN point;
                            point.x = limits[0];
                            point.y = y;
                            point.z = z;
                            point.normal_x = plane_norms[i][0];
                            point.normal_y = plane_norms[i][1];
                            point.normal_z = plane_norms[i][2];
                            cloud.points.push_back(point);
                        }
                    }
                }
                else if(fabs(limits[2] - limits[3]) < EPSILION)
                {
                    for(double x = limits[0]; x <= limits[1]+EPSILION; x = x + interval)
                    {
                        for(double z = limits[4]; z <= limits[5]+EPSILION; z = z + interval)
                        {
                            PointN point;
                            point.x = x;
                            point.y = limits[2];
                            point.z = z;
                            point.normal_x = plane_norms[i][0];
                            point.normal_y = plane_norms[i][1];
                            point.normal_z = plane_norms[i][2];
                            cloud.points.push_back(point);
                        }
                    }
                }
                else if(fabs(limits[4] - limits[5]) < EPSILION)
                {
                    for(double y = limits[2]; y <= limits[3]+EPSILION; y = y + interval)
                    {
                        for(double x = limits[0]; x <= limits[1]+EPSILION; x = x + interval)
                        {
                            PointN point;
                            point.x = x;
                            point.y = y;
                            point.z = limits[4];
                            point.normal_x = plane_norms[i][0];
                            point.normal_y = plane_norms[i][1];
                            point.normal_z = plane_norms[i][2];
                            cloud.points.push_back(point);
                        }
                    }
                }

                if(noise > EPSILION)
                {
                    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
                    std::default_random_engine generator(seed);
                    std::normal_distribution<double> dist(0, noise);
                    for(auto&& point:cloud.points)
                    {
                        point.x += dist(generator);
                        point.y += dist(generator);
                        point.z += dist(generator);
                    }
                }
            }
        }
        // Set Cloud Variables
        cloud.width    = cloud.points.size();
        cloud.height   = 1;
        cloud.is_dense = false;
    }
    std::vector<double> getPlaneLimits(std::vector<int> p_idxs, std::vector<Point> points)
    {
        double x_min = points[p_idxs[0]].x, x_max = points[p_idxs[0]].x, y_min = points[p_idxs[0]].y,
        y_max = points[p_idxs[0]].y, z_min = points[p_idxs[0]].z, z_max = points[p_idxs[0]].z;
        for(auto idx:p_idxs)
        {
            if(points[idx].x < x_min) x_min = points[idx].x;
            if(points[idx].y < y_min) y_min = points[idx].y;
            if(points[idx].z < z_min) z_min = points[idx].z;
            if(points[idx].x > x_max) x_max = points[idx].x;
            if(points[idx].y > y_max) y_max = points[idx].y;
            if(points[idx].z > z_max) z_max = points[idx].z;
        }
        return std::vector<double>{x_min, x_max, y_min, y_max, z_min, z_max};
    }

public:
////CONSTRUCTOR
    BrickPCD(ros::NodeHandle nh)
            : nh_(nh)
    {
        int pcd_type;
        double interval;
        std::string name;
        ros::NodeHandle pn("~");
        double noise;
        bool tf, face_0, face_1, face_2, face_3, face_4, face_5;
        pn.param<int>("pcd_type", pcd_type, 1);
        pn.param<double>("interval", interval, 0.005);
        pn.param<double>("noise", noise, 0.0);
        pn.param<bool>("tf", tf, false);
        pn.param<std::string>("name", name, "no_name");
        pn.param<bool>("face_0", face_0, true);
        pn.param<bool>("face_1", face_1, true);
        pn.param<bool>("face_2", face_2, false);
        pn.param<bool>("face_3", face_3, false);
        pn.param<bool>("face_4", face_4, true);
        pn.param<bool>("face_5", face_5, false);

        std::vector<int> ignore_idx;
        if(!face_0)ignore_idx.push_back(0);
        if(!face_1)ignore_idx.push_back(1);
        if(!face_2)ignore_idx.push_back(2);
        if(!face_3)ignore_idx.push_back(3);
        if(!face_4)ignore_idx.push_back(4);
        if(!face_5)ignore_idx.push_back(5);

        // Sets the path of brick pcds
        std::string path = ros::package::getPath("brick_detection");
        path.append("/data/");
        std::string r_brick_path = path, g_brick_path = path, b_brick_path = path, o_brick_path = path;
        if(name.compare("no_name") == 0)
        {
            r_brick_path.append("red_brick.pcd");
            g_brick_path.append("green_brick.pcd");
            b_brick_path.append("blue_brick.pcd");
            o_brick_path.append("orange_brick.pcd");
        }
        else
        {
            name.append(".pcd");
            r_brick_path.append(name);
            g_brick_path.append(name);
            b_brick_path.append(name);
            o_brick_path.append(name);
        }
        // Initializes clouds
        PointCloudN red_cloud, green_cloud, blue_cloud, orange_cloud;
        // Initializes bricks
        Pose pose = Geometry::zeroPose();

        Brick red_brick('R', pose), green_brick('G', pose), blue_brick('B', pose), orange_brick('O', pose);

        // Sets PCDS
        switch(pcd_type){
            case 0:
                pcdFromBrickCorners(red_cloud, red_brick);
                pcdFromBrickCorners(green_cloud, green_brick);
                pcdFromBrickCorners(blue_cloud, blue_brick);
                pcdFromBrickCorners(orange_cloud, orange_brick);
                break;
            case 1:
                pcdFromBrickFaces(red_cloud, red_brick,ignore_idx, interval,noise);
                pcdFromBrickFaces(green_cloud, green_brick, ignore_idx,interval,noise);
                pcdFromBrickFaces(blue_cloud, blue_brick, ignore_idx,interval,noise);
                pcdFromBrickFaces(orange_cloud, orange_brick, ignore_idx,interval, noise);
                break;
        }
        if(tf)
        {
            Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
            tf(0,0) = 0.8936;
            tf(0,1) = -0.4067;
            tf(0,2) = 0.1899;
            tf(1,0) = 0.3978;
            tf(1,1) = 0.9135;
            tf(1,2) = 0.0846;
            tf(2,0) = -0.2079;
            tf(2,1) = 0;
            tf(2,2) = 0.9781;
            tf(0,3) = 1.2;
            tf(1,3) = -2.3;
            tf(2,3) = 1.8;

            std::cout << tf << std::endl;
            // Executing the transformation
            PointCloudN t_red_cloud, t_green_cloud, t_blue_cloud, t_orange_cloud;
            // You can either apply transform_1 or transform_2; they are the same
            pcl::transformPointCloudWithNormals (red_cloud, t_red_cloud, tf);
            pcl::transformPointCloudWithNormals (green_cloud, t_green_cloud, tf);
            pcl::transformPointCloudWithNormals (blue_cloud, t_blue_cloud, tf);
            pcl::transformPointCloudWithNormals (orange_cloud, t_orange_cloud, tf);

            pcl::io::savePCDFileASCII (r_brick_path, t_red_cloud);
            pcl::io::savePCDFileASCII (g_brick_path, t_green_cloud);
            pcl::io::savePCDFileASCII (b_brick_path, t_blue_cloud);
            pcl::io::savePCDFileASCII (o_brick_path, t_orange_cloud);
        }
        else{
            // Saves PCDS
            pcl::io::savePCDFileASCII (r_brick_path, red_cloud);
            pcl::io::savePCDFileASCII (g_brick_path, green_cloud);
            pcl::io::savePCDFileASCII (b_brick_path, blue_cloud);
            pcl::io::savePCDFileASCII (o_brick_path, orange_cloud);
        }
    }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "png_pcd_remove_black");

    ros::NodeHandle nh;

    BrickPCD a(nh);

    return 0;
}
