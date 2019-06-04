// 2018, Philipp Ruppel, ruppel@informatik.uni-hamburg.de

#pragma once

#include "common.h"

namespace tams_tracker {

struct Calibration {

  std::vector<cv::Mat> camera_matrices;
  std::vector<cv::Mat> dist_coeffs;
  std::vector<Eigen::Affine3d> camera_poses;
  size_t view_count = 0;

  void load() {

    ros::NodeHandle node("~");

    std::vector<cv::Mat> translations;
    std::vector<cv::Mat> rotations;

    std::string calibration_file;
    node.getParam("calibration_file", calibration_file);
    ROS_INFO_STREAM("calibration file: \"" << calibration_file << "\"");
    cv::FileStorage fs(calibration_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      ROS_ERROR_STREAM("failed to open calibration file");
      return;
    }
    ROS_INFO_STREAM("cameras: " << fs["cameras"].size());
    for (auto c : fs["cameras"]) {
      ROS_INFO("camera");
      camera_matrices.emplace_back();
      dist_coeffs.emplace_back();
      translations.emplace_back();
      rotations.emplace_back();
      c["k"] >> camera_matrices.back();
      c["d"] >> dist_coeffs.back();
      c["t"] >> translations.back();
      c["r"] >> rotations.back();
      camera_matrices.back().convertTo(camera_matrices.back(), CV_64F);
      dist_coeffs.back().convertTo(dist_coeffs.back(), CV_64F);
      translations.back().convertTo(translations.back(), CV_64F);
      rotations.back().convertTo(rotations.back(), CV_64F);
    }

    view_count = camera_matrices.size();

    camera_poses.resize(camera_matrices.size());
    for (size_t icam = 0; icam < camera_matrices.size(); icam++) {
      Eigen::Matrix3d r;
      cv::cv2eigen(rotations[icam], r);
      Eigen::Vector3d t;
      cv::cv2eigen(translations[icam], t);
      Eigen::Affine3d pose = Eigen::Affine3d::Identity();
      pose.translation() = t;
      pose.linear() = r;
      pose = pose.inverse();
      camera_poses[icam] = pose;
    }
  }
};

} // namespace tams_tracker
