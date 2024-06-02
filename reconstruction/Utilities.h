#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <librealsense2/rs.hpp>

class Utilities {
public:
    static void removeNaNFromPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud); //NAN 필터링
    static void deprojectPixelToPoint(float point[3], const rs2_intrinsics* intrin, const float pixel[2], float depth);//좌표변환
};
