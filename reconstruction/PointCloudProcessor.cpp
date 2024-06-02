#include "PointCloudProcessor.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <memory>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <cmath>
#include <filesystem>

namespace fs = std::filesystem;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudProcessor::convertToPointCloud(const rs2::depth_frame& depth, const rs2::video_frame& color) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    auto color_intrin = color.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    auto depth_intrin = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    auto depth_to_color_extrin = depth.get_profile().get_extrinsics_to(color.get_profile());

    cloud->width = static_cast<uint32_t>(depth.get_width());
    cloud->height = static_cast<uint32_t>(depth.get_height());
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);

    int valid_points = 0;

    for (int dy = 0; dy < depth_intrin.height; ++dy) {
        for (int dx = 0; dx < depth_intrin.width; ++dx) {
            uint16_t depth_value = reinterpret_cast<const uint16_t*>(depth.get_data())[dy * depth_intrin.width + dx];
            float depth_in_meters = depth_value * 0.001f; // millimeters to meters

            if (depth_value == 0 || std::isnan(depth_in_meters) || depth_in_meters < 0.5f || depth_in_meters > 1.0f) {
                continue;
            }

            float pixel[2] = { static_cast<float>(dx), static_cast<float>(dy) };
            float point[3];
            rs2_deproject_pixel_to_point(point, &depth_intrin, pixel, depth_in_meters);

            if (std::isnan(point[0]) || std::isnan(point[1]) || std::isnan(point[2])) {
                continue; // Skip invalid points
            }

            float color_point[3];
            rs2_transform_point_to_point(color_point, &depth_to_color_extrin, point);

            float color_pixel[2];
            rs2_project_point_to_pixel(color_pixel, &color_intrin, color_point);
            int cx = static_cast<int>(color_pixel[0]), cy = static_cast<int>(color_pixel[1]);

            if (cx < 0 || cy < 0 || cx >= color_intrin.width || cy >= color_intrin.height) {
                continue;
            }

            pcl::PointXYZRGB& p = cloud->points[dy * depth_intrin.width + dx];
            p.x = point[0];
            p.y = point[1];
            p.z = point[2];
            p.r = reinterpret_cast<const uint8_t*>(color.get_data())[cy * color_intrin.width * 3 + cx * 3];
            p.g = reinterpret_cast<const uint8_t*>(color.get_data())[cy * color_intrin.width * 3 + cx * 3 + 1];
            p.b = reinterpret_cast<const uint8_t*>(color.get_data())[cy * color_intrin.width * 3 + cx * 3 + 2];

            valid_points++;
        }
    }

    std::cerr << "Valid points in point cloud: " << valid_points << std::endl;

    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, index);

    if (cloud->empty()) {
        std::cerr << "No data in point cloud to save." << std::endl;
        return nullptr;
    }

    return cloud;
}

void PointCloudProcessor::savePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const std::string& filename) {
    if (!cloud) {
        std::cerr << "Null pointer passed to savePointCloud." << std::endl;
        return;
    }

    try {
        fs::path pcd_directory = "saved_pcd";
        if (!fs::exists(pcd_directory)) {
            fs::create_directory(pcd_directory);
        }

        fs::path file_path = pcd_directory / filename;

        if (pcl::io::savePCDFileASCII(file_path.string(), *cloud) == -1) {
            std::cerr << "Failed to save PCD file: " << file_path.string() << std::endl;
        }
        else {
            std::cout << "PCD file saved: " << file_path.string() << std::endl;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Exception while saving PCD file: " << e.what() << std::endl;
    }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudProcessor::removeNoise(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    if (cloud->empty()) {
        std::cerr << "No data in point cloud to filter." << std::endl;
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);
    return cloud_filtered;
}
