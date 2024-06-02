#include "Utilities.h"
#include <pcl/filters/filter.h>
#include <librealsense2/rsutil.h>

void Utilities::removeNaNFromPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, index);
}

void Utilities::deprojectPixelToPoint(float point[3], const rs2_intrinsics* intrin, const float pixel[2], float depth) {
    rs2_deproject_pixel_to_point(point, intrin, pixel, depth);
}
