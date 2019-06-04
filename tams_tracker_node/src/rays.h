// 2018, Philipp Ruppel, ruppel@informatik.uni-hamburg.de

#pragma once

#include "common.h"

#include "calibration.h"
#include "camera.h"
#include "config.h"

namespace tams_tracker {

struct Ray {
  size_t marker_index;
  Eigen::Vector3d origin;
  Eigen::Vector3d direction;
};

class RayBuilder {
  Config config;
  Calibration calibration;
  std::vector<Camera::Point> points;
  std::vector<Ray> rays, x_rays;
  mutable std::mutex x_mutex;

public:
  RayBuilder(const Config &config, const Calibration &calibration)
      : config(config), calibration(calibration) {}

  void update(const std::deque<Camera> &cameras) {
    rays.clear();
    for (size_t icam = 0; icam < cameras.size(); icam++) {
      auto &cam = cameras[icam];
      cam.getPoints(points);
      for (auto &pt : points) {
        Ray ray;
        cv::Point2f point(pt.position.x(), pt.position.y());
        cv::Point2f p;
        cv::undistortPoints(cv::_InputArray(&point, 1), cv::_OutputArray(&p, 1),
                            calibration.camera_matrices[icam],
                            calibration.dist_coeffs[icam]);
        ray.origin = calibration.camera_poses[icam].translation();
        ray.direction =
            (calibration.camera_poses[icam] * Eigen::Vector3d(p.x, p.y, 1.0) -
             ray.origin)
                .normalized();
        ray.marker_index = pt.marker_index;
        rays.push_back(ray);
      }
    }
    {
      std::lock_guard<std::mutex> lock(x_mutex);
      x_rays = rays;
    }
  }
  void getRays(std::vector<Ray> &rays) {
    {
      std::lock_guard<std::mutex> lock(x_mutex);
      rays = x_rays;
    }
  }
};

} // namespace tams_tracker
