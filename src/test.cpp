#include <istream>
#include <opencv2/opencv.hpp>
#include "ArgusCamera.hpp"

#define ONE_SECOND_NANOS 1000000000

int main() {
  ArgusCameraConfig config;
  config.mDeviceId = 0;
  config.mSourceClipRect = { 0.0, 0.0, 1.0, 1.0 };
  config.mStreamResolution = { 640, 360 };
  config.mVideoConverterResolution = { 640, 360 };
  config.mFrameDurationRange = { ONE_SECOND_NANOS / 30, ONE_SECOND_NANOS / 30 }; // 30fps
  config.mSensorMode = 0;
  ArgusCamera *camera = ArgusCamera::createArgusCamera(config);

  cv::Mat left_img(360, 640, CV_8UC4);
  cv::Mat right_img(360, 640, CV_8UC4);
  int count = 0;

  while(1) {
    CameraInfo camera_info;
    camera_info.left_data = new uint8_t[640*360*4];
    camera_info.right_data = new uint8_t[640*360*4];
    camera->read(&camera_info);
    // for(int i=0;i<640;i++)
    //     std::cout << int(camera_info.right_data[i]) << ",";
    memcpy(left_img.data, camera_info.left_data, sizeof(uint8_t)*640*360*4);
    // cv::cvtColor(left_img, left_img, cv::COLOR_RGBA2BGRA);
    // cv::imshow("left_img", left_img);
    
    memcpy(right_img.data, camera_info.right_data, sizeof(uint8_t)*640*360*4);
    // cv::cvtColor(right_img, right_img, cv::COLOR_RGBA2BGRA);
    // cv::imshow("right_img", right_img);
    if(count%500 ==0) {
      std::string img_name = std::to_string(count);
      cv::imwrite("../" + img_name + "left.bmp", left_img);
      cv::imwrite("../" + img_name + "right.bmp", right_img);
    }
    cv::waitKey(5);
    count++;

  }
//   delete camera_info.left_data;
//   delete camera_info.right_data;
  return 1;
}