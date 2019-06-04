#include "common.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "node");

  ros::NodeHandle node;

  ros::AsyncSpinner spinner(4);
  spinner.start();

  std::vector<cv::VideoCapture> cameras;
  for (size_t i = 0; ; i++) {
    cameras.emplace_back(i);
    if(!cameras.back().isOpened()) {
      cameras.pop_back();
      break;
    }
  }

  mkdir("recordings", 0777);

  std::string path =
      "recordings/rec-" + std::to_string(ros::WallTime::now().toNSec());
  mkdir(path.c_str(), 0777);

  std::vector<cv::VideoWriter> writers;
  for (size_t i = 0; i < cameras.size(); i++) {
    writers.emplace_back();
    cv::Mat image;
    cameras[i].read(image);
    writers.back().open(path + "/" + std::to_string(i) + ".avi",
                        CV_FOURCC('M', 'J', '2', 'C'), 2,
                        cv::Size(image.cols, image.rows), true);
  }

  std::vector<cv::Mat> images(cameras.size());

  while (ros::ok()) {

    for (size_t i = 0; i < cameras.size(); i++) {
      cameras[i].read(images[i]);
    }

    for (size_t i = 0; i < cameras.size(); i++) {
      cv::imshow(std::to_string(i).c_str(), images[i]);
    }

    int key = cv::waitKey(1);
    // ROS_INFO_STREAM(key);
    if (key == 27)
      break;
    if (key == 32) {
      for (size_t i = 0; i < cameras.size(); i++) {
        writers[i].write(images[i]);
      }
      for (size_t i = 0; i < cameras.size(); i++) {
        images[i] = images[i] / 2 + 127;
        cv::imshow(std::to_string(i).c_str(), images[i]);
      }
      cv::waitKey(100);
    }

    // ros::Duration(0.5).sleep();
  }
}
