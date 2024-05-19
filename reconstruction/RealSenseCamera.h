#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

class RealSenseCamera {
public:
    RealSenseCamera();
    void startCamera();
    rs2::frameset getFrames();
    void stopCamera();

private:
    rs2::pipeline pipe;
    rs2::config cfg;
};

