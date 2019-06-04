// 2018, Philipp Ruppel, ruppel@informatik.uni-hamburg.de

#pragma once

#include "common.h"

#include "config.h"
#include "rays.h"

namespace tams_tracker {

struct Marker {
  size_t marker_index;
  Eigen::Vector3d position;
  double min_squared_distance_to_camera;
};

class MarkerDetector {

  Config config;
  std::vector<Marker> markers;
  std::vector<Eigen::Vector3d> ray_up, ray_side;
  mutable std::mutex x_mutex;
  std::vector<Marker> x_markers;

public:
  MarkerDetector(const Config &config) : config(config) {}

  void update(const std::vector<Ray> &rays) {

    ray_up.resize(rays.size());
    ray_side.resize(rays.size());
    Eigen::Vector3d v = Eigen::Vector3d::Random();
    for (size_t iray = 0; iray < rays.size(); iray++) {
      ray_up[iray] = rays[iray].direction.cross(v).normalized();
      ray_side[iray] = rays[iray].direction.cross(ray_up[iray]).normalized();
    }

    markers.clear();
    for (size_t ir = 0; ir < rays.size(); ir++) {
      for (size_t jr = 0; jr < rays.size(); jr++) {

        if (rays[ir].marker_index != rays[jr].marker_index) {
          continue;
        }

        std::array<size_t, 2> ray_indices;
        ray_indices[0] = ir;
        ray_indices[1] = jr;

        Eigen::Matrix<double, 2 * 2, 3> jacobian;
        Eigen::Matrix<double, 2 * 2, 1> residual;
        Eigen::Matrix<double, 3, 1> solution;

        Eigen::Vector3d position = (rays[ir].origin + rays[jr].origin) * 0.5;

        for (size_t i = 0; i < 2; i++) {

          jacobian(i * 2 + 0, 0) = ray_up[ray_indices[i]].x();
          jacobian(i * 2 + 0, 1) = ray_up[ray_indices[i]].y();
          jacobian(i * 2 + 0, 2) = ray_up[ray_indices[i]].z();

          jacobian(i * 2 + 1, 0) = ray_side[ray_indices[i]].x();
          jacobian(i * 2 + 1, 1) = ray_side[ray_indices[i]].y();
          jacobian(i * 2 + 1, 2) = ray_side[ray_indices[i]].z();

          residual[i * 2 + 0] =
              ray_up[ray_indices[i]].dot(position) -
              ray_up[ray_indices[i]].dot(rays[ray_indices[i]].origin);
          residual[i * 2 + 1] =
              ray_side[ray_indices[i]].dot(position) -
              ray_side[ray_indices[i]].dot(rays[ray_indices[i]].origin);
        }

        solution = jacobian.colPivHouseholderQr().solve(residual);

        position -= solution;

        bool ok = true;
        for (size_t i = 0; i < 2; i++) {
          auto &origin = rays[ray_indices[i]].origin;
          auto &direction = rays[ray_indices[i]].direction;
          Eigen::Vector3d diff = position - origin;
          if (diff.dot(direction) < config.min_distance_to_camera) {
            ok = false;
            break;
          }
          diff -= direction * diff.dot(direction);
          if (diff.squaredNorm() / (position - origin).squaredNorm() >
              squared(config.max_reprojection_error)) {
            ok = false;
            break;
          }
        }
        if (!ok) {
          continue;
        }

        Marker marker;
        marker.position = position;
        marker.marker_index = rays[ir].marker_index;
        markers.push_back(marker);
      }
    }

    {
      std::vector<std::vector<Marker>> clusters;
      for (auto &m : markers) {
        bool added = false;
        for (auto &c : clusters) {
          for (auto &n : c) {
            if (m.marker_index == n.marker_index &&
                (m.position - n.position).squaredNorm() <
                    squared(config.marker_radius)) {
              c.push_back(m);
              added = true;
              break;
            }
          }
          if (added) {
            break;
          }
        }
        if (!added) {
          clusters.emplace_back();
          clusters.back().push_back(m);
        }
      }
      markers.clear();
      for (auto &cluster : clusters) {
        auto m = cluster.front();
        m.position.setZero();
        for (auto &n : cluster) {
          m.position += n.position;
        }
        m.position /= cluster.size();
        markers.push_back(m);
      }
    }

    {
      std::lock_guard<std::mutex> lock(x_mutex);
      x_markers = markers;
    }
  }

  void getMarkers(std::vector<Marker> &markers) {
    std::lock_guard<std::mutex> lock(x_mutex);
    markers = x_markers;
  }
};

} // namespace tams_tracker
