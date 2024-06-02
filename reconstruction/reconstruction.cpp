#include "RealSenseCamera.h"
#include "PointCloudProcessor.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <mutex>
#include <thread>
#include <atomic>
#include <filesystem>

std::mutex viewer_mutex;
bool save_triggered = false;
std::atomic<int> file_index(1);

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing) {
    if (event.getKeySym() == "s" && event.keyDown()) {
        save_triggered = true;
    }
}

void savePointCloudToFile(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    if (!cloud) {
        std::cerr << "Error: Null pointer passed to savePointCloudToFile." << std::endl;
        return;
    }

    int index = file_index++;
    std::string filename = "outputCloud_" + std::to_string(index) + ".pcd";
    PointCloudProcessor::savePointCloud(cloud, filename);
}

int main() {
    try {
        RealSenseCamera camera;
        camera.startCamera();

        pcl::visualization::PCLVisualizer viewer("3D Viewer");
        viewer.setBackgroundColor(0.05, 0.05, 0.05);
        viewer.initCameraParameters();
        viewer.registerKeyboardCallback(keyboardEventOccurred);

        viewer.addCoordinateSystem(0.1);

        while (!viewer.wasStopped()) {
            viewer.spinOnce(100);
            std::lock_guard<std::mutex> lock(viewer_mutex);

            auto frames = camera.getFrames();
            if (!frames) continue;

            auto depth = frames.get_depth_frame();
            auto color = frames.get_color_frame();

            if (!depth || !color) continue;

            auto cloud = PointCloudProcessor::convertToPointCloud(depth, color);
            if (!cloud || cloud->empty()) {
                std::cerr << "No valid point cloud data available." << std::endl;
                continue;
            }

            viewer.removeAllPointClouds();
            viewer.addPointCloud<pcl::PointXYZRGB>(cloud, "Sample Cloud");

            if (save_triggered) {
                std::thread saveThread(savePointCloudToFile, cloud);
                saveThread.detach();
                save_triggered = false;
            }
        }

        camera.stopCamera();
    }
    catch (const rs2::error& e) {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n" << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
