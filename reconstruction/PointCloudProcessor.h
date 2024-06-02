#ifndef POINT_CLOUD_PROCESSOR_H
#define POINT_CLOUD_PROCESSOR_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <librealsense2/rs.hpp>

class PointCloudProcessor {
public:
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertToPointCloud(const rs2::depth_frame& depth, const rs2::video_frame& color);
    static void savePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const std::string& filename);
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeNoise(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
};

#endif // POINT_CLOUD_PROCESSOR_H
