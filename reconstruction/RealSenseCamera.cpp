// RealSense ī�޶���� ���� �� ������ ��Ʈ�� ����
/* ���:
	- ����Ʈ Ŭ���� �����͸� �Է¹޾� �޽� ���� ����
	- �޽� ����ȭ �� ���� ����(������, ���� ��)
	- �޽� ���Ϸ��� ���� �� �ε� ���
*/

#include "RealSenseCamera.h"

RealSenseCamera::RealSenseCamera() {
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
}

void RealSenseCamera::startCamera() {
    pipe.start(cfg);  
}

rs2::frameset RealSenseCamera::getFrames() {
    return pipe.wait_for_frames();  
}

void RealSenseCamera::stopCamera() {
    pipe.stop();  
}
