// 메인 실행 파일, 프로세스 흐름 제어
/* 기능:
	- 각 모듈 초기화 및 설정
	- 주요 처리 루프 구현
	- 사용자 입력 처리 및 결과 표시
	- 시스템 상태 모니터링 및 오류 처리
*/
#include "RealSenseCamera.h"
#include "PointCloudProcessor.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <mutex>

std::mutex viewer_mutex;
bool save_triggered = false;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing) {
    if (event.getKeySym() == "s" && event.keyDown()) {
        save_triggered = true;
    }
}

int main() {
    try {
        RealSenseCamera camera;
        camera.startCamera();

        pcl::visualization::PCLVisualizer viewer("3D Viewer");
        viewer.setBackgroundColor(0.05, 0.05, 0.05);
        viewer.initCameraParameters();
        viewer.registerKeyboardCallback(keyboardEventOccurred);

        //좌표축
        viewer.addCoordinateSystem(0.1);

        while (!viewer.wasStopped()) {
            viewer.spinOnce(100);
            std::lock_guard<std::mutex> lock(viewer_mutex);

            auto frames = camera.getFrames(); // Get frames from the camera
            if (!frames) continue;

            auto depth = frames.get_depth_frame(); // Get the depth frame
            auto color = frames.get_color_frame(); // Get the color frame

            if (!depth || !color) continue;

            auto cloud = PointCloudProcessor::convertToPointCloud(depth, color);
            if (!cloud || cloud->empty()) {
                std::cerr << "No valid point cloud data available." << std::endl;
                continue;
            }

            viewer.removeAllPointClouds();
            viewer.addPointCloud<pcl::PointXYZRGB>(cloud, "Sample Cloud");

            if (save_triggered) {
                PointCloudProcessor::savePointCloud(cloud, "outputCloud.pcd");
                auto filteredCloud = PointCloudProcessor::removeNoise(cloud);
                PointCloudProcessor::savePointCloud(filteredCloud, "filteredOutputCloud.pcd");
                std::cout << "Cloud saved and filtered!" << std::endl;
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
