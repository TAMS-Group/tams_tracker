// 2018, Philipp Ruppel, ruppel@informatik.uni-hamburg.de

#pragma once

#include "common.h"

#include "imu.h"
#include "markers.h"

namespace tams_tracker {

struct Rigid {
  Eigen::Transform<double, 3, Eigen::Isometry, Eigen::Unaligned> pose =
      Eigen::Isometry3d::Identity();
  size_t rigid_index;
  double error;
};

class RigidTracker {
  Config config;
  std::vector<Rigid> rigids, x_rigids;
  std::mutex x_mutex;
  std::vector<std::vector<Marker>> marker_map;
  std::vector<std::vector<Rigid>> rigid_map;
  std::vector<double> lmeds_buffer;
  std::vector<std::shared_ptr<Imu>> imus;
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
      previous_poses;

public:
  RigidTracker(const Config &config) : config(config) {
    marker_map.resize(config.markers.size());
    rigid_map.resize(config.rigids.size());
    for (auto &rigid : config.rigids) {
      imus.emplace_back(std::make_shared<Imu>(rigid.imu_topic));
    }
    previous_poses.resize(config.rigids.size(), Eigen::Isometry3d::Identity());
  }

  void findPose(Rigid &rigid,
                const std::array<Eigen::Vector3d, 3> &marker_positions,
                const std::array<size_t, 3> &edge_indices) {

    auto &rigid_config = config.rigids[rigid.rigid_index];

    rigid.pose = Eigen::Isometry3d::Identity();
    for (size_t i = 0; i < 3; i++) {
      rigid.pose.translation() += marker_positions[i] * (1.0 / 3.0);
    }
    rigid.pose.linear() =
        Eigen::AngleAxisd(
            Eigen::Vector3d::Random().x() * M_PI,
            ((Eigen::Vector3d)Eigen::Vector3d::Random()).normalized())
            .matrix();

    for (size_t iteration = 0; iteration < 8; iteration++) {

      Eigen::Matrix<double, 3 * 3, 1> residuals;
      Eigen::Matrix<double, 3 * 3, 6> gradients;

      residuals.setZero();
      gradients.setZero();

      for (size_t marker_index = 0; marker_index < 3; marker_index++) {

        Eigen::Vector3d observed_position = marker_positions[marker_index];
        Eigen::Vector3d model_position =
            rigid.pose *
            rigid_config.markers[edge_indices[marker_index]].position;

        residuals.segment<3>(marker_index * 3) =
            model_position - observed_position;

        gradients(marker_index * 3 + 0, 0) = 1.0;
        gradients(marker_index * 3 + 1, 1) = 1.0;
        gradients(marker_index * 3 + 2, 2) = 1.0;

        Eigen::Matrix3d rx, ry, rz;
        rx << 0, 0, 0, 0, 0, -1, 0, 1, 0;
        ry << 0, 0, 1, 0, 0, 0, -1, 0, 0;
        rz << 0, -1, 0, 1, 0, 0, 0, 0, 0;

        Eigen::Vector3d tx =
            rx * -(observed_position - rigid.pose.translation());
        Eigen::Vector3d ty =
            ry * -(observed_position - rigid.pose.translation());
        Eigen::Vector3d tz =
            rz * -(observed_position - rigid.pose.translation());

        gradients.block<3, 1>(marker_index * 3, 3) = tx;
        gradients.block<3, 1>(marker_index * 3, 4) = ty;
        gradients.block<3, 1>(marker_index * 3, 5) = tz;
      }

      Eigen::Matrix<double, 6, 1> solution;
      solution.setZero();
      solution = gradients.colPivHouseholderQr().solve(residuals);

      rigid.pose.translation() -= solution.head<3>();

      Eigen::Vector3d rvec = solution.tail<3>();
      Eigen::Matrix3d l = rigid.pose.linear();
      if (rvec.squaredNorm() > 0.0) {
        l = Eigen::AngleAxisd(rvec.norm(), rvec.normalized()).matrix() * l;
      }
      rigid.pose.linear() = l;
    }
  }

  void adjustPose(Rigid &rigid, const std::vector<Ray> &rays,
                  bool all = false) {
    auto &rigid_config = config.rigids[rigid.rigid_index];

    // ROS_INFO_STREAM("adjust");

    Eigen::Vector3d rvec = Eigen::Vector3d::Random().normalized();

    for (size_t iteration = 0; iteration < 32; iteration++) {

      std::vector<Eigen::Vector2d> residuals;
      std::vector<Eigen::Matrix<double, 2, 6>> gradients;

      for (auto &m : rigid_config.markers) {
        Eigen::Vector3d p = rigid.pose * m.position;
        for (auto &ray : rays) {

          if (m.marker_index != ray.marker_index) {
            continue;
          }

          Eigen::Vector3d d = p - ray.origin;

          double f = d.dot(ray.direction);
          if (f < config.min_distance_to_camera) {
            continue;
          }
          double f_rcp = 1.0 / f;

          d -= ray.direction * f;
          if (!all) {
            if ((d * f_rcp).squaredNorm() >
                squared(config.max_reprojection_error)) {
              continue;
            }
          }

          Eigen::Vector3d up = ray.direction.cross(rvec).normalized();
          Eigen::Vector3d side = ray.direction.cross(up).normalized();

          Eigen::Vector2d residual;
          residual.x() = side.dot(d) * f_rcp;
          residual.y() = up.dot(d) * f_rcp;
          residuals.push_back(residual);

          Eigen::Matrix<double, 2, 6> grad;
          grad.setZero();

          grad.block<1, 3>(0, 0) = side * f_rcp;
          grad.block<1, 3>(1, 0) = up * f_rcp;

          Eigen::Matrix3d rx, ry, rz;
          rx << 0, 0, 0, 0, 0, -1, 0, 1, 0;
          ry << 0, 0, 1, 0, 0, 0, -1, 0, 0;
          rz << 0, -1, 0, 1, 0, 0, 0, 0, 0;

          Eigen::Vector3d q = rigid.pose.linear() * m.position;
          grad(0, 3) = side.dot(rx * q) * f_rcp;
          grad(0, 4) = side.dot(ry * q) * f_rcp;
          grad(0, 5) = side.dot(rz * q) * f_rcp;
          grad(1, 3) = up.dot(rx * q) * f_rcp;
          grad(1, 4) = up.dot(ry * q) * f_rcp;
          grad(1, 5) = up.dot(rz * q) * f_rcp;

          gradients.push_back(grad);
        }
      }

      Eigen::MatrixXd gradient_matrix(gradients.size() * 2 + 6, 6);
      for (size_t i = 0; i < gradients.size(); i++) {
        gradient_matrix.block<2, 6>(i * 2, 0) = gradients[i];
      }

      Eigen::VectorXd residual_vector(residuals.size() * 2 + 6);
      for (size_t i = 0; i < residuals.size(); i++) {
        residual_vector[i * 2 + 0] = residuals[i][0];
        residual_vector[i * 2 + 1] = residuals[i][1];
      }

      for (size_t i = 0; i < 6; i++) {
        size_t row = residuals.size() * 2 + i;
        residual_vector[row] = 0.0;
        for (size_t col = 0; col < 6; col++) {
          gradient_matrix(row, col) = 0.0;
        }
        gradient_matrix(row, i) = 0.1;
      }

      Eigen::Matrix<double, 6, 1> solution;

      solution = gradient_matrix.colPivHouseholderQr().solve(residual_vector);

      // ROS_INFO_STREAM("residual " << residual_vector);
      // ROS_INFO_STREAM("gradient " << gradient_matrix);
      // ROS_INFO_STREAM("solution " << solution);

      rigid.pose.translation() -= solution.head<3>();

      Eigen::Matrix3d l = rigid.pose.linear();

      Eigen::Vector3d rvec = -solution.tail(3);
      if (rvec.squaredNorm() > 0.0) {
        l = Eigen::AngleAxisd(rvec.norm(), rvec.normalized()).matrix() * l;
      }

      /*l = Eigen::Quaterniond(1.0, -solution[3], -solution[4], -solution[5])
              .normalized() *
          l;*/

      rigid.pose.linear() = l;
    }
  }

  void update(const std::vector<Ray> &rays,
              const std::vector<Marker> &markers) {
    if (config.rigids.empty()) {
      return;
    }
    for (auto &m : marker_map) {
      m.clear();
    }
    for (auto &m : markers) {
      marker_map[m.marker_index].push_back(m);
    }

    for (auto &rigid : rigids) {
      Eigen::Vector3d rotation_vector =
          config.rigids[rigid.rigid_index].imu_orientation *
          imus[rigid.rigid_index]->getAngularVelocity() * (1.0 / 60);
      if (rotation_vector.squaredNorm() > 0) {
        rigid.pose =
            rigid.pose * Eigen::AngleAxisd(rotation_vector.norm(),
                                           rotation_vector.normalized());
      }
    }

    /*{
      std::vector<Rigid> rigids2;
      for (auto &rigid : rigids) {
        {
          auto rigid2 = rigid;
          rigid2.pose =
              rigid.pose *
              (previous_poses[rigid.rigid_index].inverse() * rigid.pose);
          rigids2.push_back(rigid2);
        }
        {
          auto rigid2 = rigid;
          rigid2.pose.translation() +=
              rigid.pose.translation() -
              previous_poses[rigid.rigid_index].translation();
          rigids2.push_back(rigid2);
        }
      }
      for (auto &rigid : rigids2) {
        rigids.push_back(rigid);
      }
    }*/

    // rigids.clear();
    for (size_t i = 0; i < config.ransac_iterations; i++) {

      size_t rigid_index = rand() % config.rigids.size();
      auto &rigid_config = config.rigids[rigid_index];
      if (rigid_config.markers.size() < 3) {
        continue;
      }

      /*Rigid rigid;
      rigid.rigid_index = rigid_index;
      rigid.pose = Eigen::Isometry3d::Identity();
      rigid.error = 0.0;
      adjustPose(rigid, rays, true);*/

      std::array<size_t, 3> edge_indices;
      for (size_t i = 0; i < 3;) {
        edge_indices[i] = rand() % rigid_config.markers.size();
        bool good = true;
        for (size_t j = 0; j < i; j++) {
          if (edge_indices[j] == edge_indices[i]) {
            good = false;
          }
        }
        if (good) {
          i++;
        }
      }

      std::array<Eigen::Vector3d, 3> edge_positions;
      for (size_t i = 0; i < 3; i++) {
        edge_positions[i] = rigid_config.markers[edge_indices[i]].position;
      }

      std::array<Eigen::Vector3d, 3> marker_positions;
      {
        bool good = true;
        for (size_t i = 0; i < 3; i++) {
          auto &l =
              marker_map[rigid_config.markers[edge_indices[i]].marker_index];
          if (l.empty()) {
            good = false;
            break;
          }
          marker_positions[i] = l[rand() % l.size()].position;
        }
        if (!good) {
          continue;
        }
      }

      /*auto makeFrame = [](const std::array<Eigen::Vector3d, 3> &points) {
        Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
        m.col(3).head(3) = points[0];
        m.col(0).head(3) = (points[1] - points[0]).normalized();
        m.col(1).head(3) = ((Eigen::Vector3d)m.col(0).head(3))
                               .cross((Eigen::Vector3d(points[2] - points[0])))
                               .normalized();
        m.col(2).head(3) = ((Eigen::Vector3d)m.col(0).head(3))
                               .cross((Eigen::Vector3d)m.col(1).head(3))
                               .normalized();
        return m;
      };*/

      // Eigen::MatrixXd frame =
      //    makeFrame(marker_positions) * makeFrame(edge_positions).inverse();

      // ROS_INFO_STREAM(frame);

      /*ROS_INFO_STREAM("points");
      for (auto &p : marker_positions) {
        ROS_INFO_STREAM(p);
      }*/

      Rigid rigid;
      rigid.rigid_index = rigid_index;
      rigid.pose = Eigen::Isometry3d::Identity();
      // rigid.pose.matrix() = frame;
      rigid.error = 0.0;
      findPose(rigid, marker_positions, edge_indices);

      rigids.push_back(rigid);
    }

    for (auto &rigid : rigids) {

      size_t rigid_index = rigid.rigid_index;

      auto &rigid_config = config.rigids[rigid_index];

      // ransac
      rigid.error = 0.0;
      for (auto &m : rigid_config.markers) {
        Eigen::Vector3d p = rigid.pose * m.position;
        for (auto &ray : rays) {
          if (ray.marker_index != m.marker_index) {
            continue;
          }
          Eigen::Vector3d d = p - ray.origin;
          double f = d.dot(ray.direction);
          if (f < config.min_distance_to_camera) {
            continue;
          }
          d -= ray.direction * f;
          d /= f;
          if (!(d.squaredNorm() < squared(config.max_reprojection_error))) {
            continue;
          }
          rigid.error--;
        }
      }

      // lmeds
      /*lmeds_buffer.clear();
      for (auto &m : rigid_config.markers) {
        Eigen::Vector3d p = rigid.pose * m.position;
        for (auto &ray : rays) {
          Eigen::Vector3d d = p - ray.origin;
          double f = d.dot(ray.direction);
          if (f < config.min_distance_to_camera) {
            continue;
          }
          d -= ray.direction * f;
          d /= f;
          double e = d.squaredNorm();
          lmeds_buffer.push_back(e);
        }
      }
      if (lmeds_buffer.empty()) {
        continue;
      }
      std::sort(lmeds_buffer.begin(), lmeds_buffer.end());
      rigid.error = lmeds_buffer[lmeds_buffer.size() / 2];*/
    }

    {
      for (auto &m : rigid_map) {
        m.clear();
      }
      for (auto &r : rigids) {
        rigid_map[r.rigid_index].push_back(r);
      }
      rigids.clear();
      for (auto &m : rigid_map) {
        if (m.empty()) {
          continue;
        }
        std::partial_sort(
            m.begin(), m.begin() + 1, m.end(),
            [](const Rigid &a, const Rigid &b) { return a.error < b.error; });

        rigids.push_back(m.front());
      }
    }

    for (auto &rigid : rigids) {
      adjustPose(rigid, rays);
    }

    for (auto &rigid : rigids) {
      previous_poses[rigid.rigid_index] = rigid.pose;
    }

    {
      std::lock_guard<std::mutex> lock(x_mutex);
      x_rigids = rigids;
    }
  }
  void getRigids(std::vector<Rigid> &rigids) {
    std::lock_guard<std::mutex> lock(x_mutex);
    rigids = x_rigids;
  }
};

} // namespace tams_tracker
