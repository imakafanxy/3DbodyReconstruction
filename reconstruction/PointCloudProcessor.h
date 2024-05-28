#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <librealsense2/rs.hpp>
#include <pcl/io/pcd_io.h>

class PointCloudProcessor {
public:
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertToPointCloud(const rs2::depth_frame& depth, const rs2::video_frame& color);
    static void savePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const std::string& filename) {
        pcl::io::savePCDFileBinary(filename, *cloud);
    }
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeNoise(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
};
