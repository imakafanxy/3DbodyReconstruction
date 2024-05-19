// RealSense 카메라와의 연결 및 데이터 스트림 관리
/* 기능:
	- 포인트 클라우드 데이터를 입력받아 메쉬 구조 생성
	- 메쉬 최적화 및 세부 조정(스무딩, 정리 등)
	- 메쉬 파일로의 저장 및 로딩 기능
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
