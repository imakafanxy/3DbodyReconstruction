#ifndef UTILITIES_H
#define UTILITIES_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <librealsense2/rs.hpp>

namespace Utilities {
    void removeNaNFromPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
    void deprojectPixelToPoint(float point[3], const rs2_intrinsics* intrin, const float pixel[2], float depth);
}

#endif // UTILITIES_H
