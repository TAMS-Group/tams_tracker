

#include "common.h"

#include "config.h"

#include "imu.h"

#include <tams_tracker_msgs/MarkerArrayStamped.h>
#include <tams_tracker_msgs/PeakArrayStamped.h>
#include <tams_tracker_msgs/RayArrayStamped.h>
#include <tams_tracker_msgs/RigidArrayStamped.h>

int main(int argc, char **argv) {

  using namespace tams_tracker;

  ros::init(argc, argv, "tams_tracker_filter");

  ros::NodeHandle node("~");

  Config config;
  config.load();

  struct Twist {
    Eigen::Vector3d linear = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular = Eigen::Vector3d::Zero();
    Twist operator*(double f) const {
      Twist ret = *this;
      ret.linear *= f;
      ret.angular *= f;
      return ret;
    }
    Twist operator-(const Twist &other) const {
      Twist ret;
      ret.linear = linear - other.linear;
      ret.angular = angular - other.angular;
      return ret;
    }
  };

  auto twist_to_isometry = [](const Twist &twist) {
    Eigen::Isometry3d isometry(Eigen::Translation3d(twist.linear));
    if (twist.angular.norm() > 0.0) {
      isometry = isometry * Eigen::AngleAxisd(twist.angular.norm(),
                                              twist.angular.normalized());
    }
    return isometry;
  };

  auto isometry_to_twist = [](const Eigen::Isometry3d &isometry) {
    Eigen::AngleAxisd aa(isometry.linear());
    Twist ret;
    ret.linear = isometry.translation();
    ret.angular = aa.axis() * aa.angle();
    return ret;
  };

  struct RigidBodyState {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(12, 12) * 10;
    Twist velocity;
    Eigen::Isometry3d visual_pose = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d previous_visual_pose = Eigen::Isometry3d::Identity();
    Eigen::VectorXd last_measurement = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd measurement_noise_estimate = Eigen::VectorXd::Ones(12);
  };

  std::vector<RigidBodyState, Eigen::aligned_allocator<RigidBodyState>> rigids(
      config.rigids.size());

  std::unordered_map<size_t, size_t> rigid_id_to_index_map;
  for (size_t i = 0; i < config.rigids.size(); i++) {
    rigid_id_to_index_map[config.rigids[i].id] = i;
  }

  std::mutex mutex;

  boost::function<void(const tams_tracker_msgs::RigidArrayStamped &)>
      rigid_callback =
          [&](const tams_tracker_msgs::RigidArrayStamped &message) {

            std::lock_guard<std::mutex> lock(mutex);
            for (auto &r : message.rigids) {
              auto iter = rigid_id_to_index_map.find(r.id);
              if (iter != rigid_id_to_index_map.end()) {
                tf::poseMsgToEigen(r.pose, rigids[iter->second].visual_pose);
              }
            }
          };
  ros::Subscriber rigid_sub =
      node.subscribe<tams_tracker_msgs::RigidArrayStamped>(
          "/tams_tracker_node/rigids", 10, rigid_callback);

  std::deque<Imu> imus;
  for (auto &r : config.rigids) {
    imus.emplace_back(r.imu_topic);
  }

  tf::TransformBroadcaster tf_broadcaster;

  double frequency = config.frequency;

  ros::Rate rate(frequency);
  double delta_time = 1.0 / frequency;
  while (ros::ok()) {

    ros::spinOnce();

    rate.sleep();

    std::lock_guard<std::mutex> lock(mutex);

    for (size_t rigid_index = 0; rigid_index < rigids.size(); rigid_index++) {
      auto &rigid = rigids[rigid_index];

      Twist pose_measurement =
          isometry_to_twist(rigid.pose.inverse() * rigid.visual_pose);
      Twist velocity_measurement =
          isometry_to_twist(rigid.previous_visual_pose.inverse() *
                            rigid.visual_pose) *
          frequency;

      Eigen::MatrixXd measurement_noise(12, 12);
      measurement_noise.setZero();

      measurement_noise.diagonal().head(6).setConstant(0.1);
      measurement_noise.diagonal().tail(6).setConstant(0.001);

      if (imus[rigid_index].hasReceivedMessages()) {

        velocity_measurement.angular =
            config.rigids[rigid_index].imu_orientation *
            imus[rigid_index].getAngularVelocity() * 1;
      }

      Eigen::VectorXd state_measurement(12);
      state_measurement.setZero();
      state_measurement.segment(0, 3) = pose_measurement.linear;
      state_measurement.segment(3, 3) = pose_measurement.angular;
      state_measurement.segment(6, 3) = velocity_measurement.linear;
      state_measurement.segment(9, 3) = velocity_measurement.angular;

      double noise_estimation_delta = delta_time;
      rigid.measurement_noise_estimate =
          rigid.measurement_noise_estimate * (1 - noise_estimation_delta) +
          (rigid.last_measurement - state_measurement).cwiseAbs() *
              (noise_estimation_delta);

      Eigen::VectorXd state(12);
      state.setZero();
      state << 0, 0, 0, 0, 0, 0, rigid.velocity.linear.x(),
          rigid.velocity.linear.y(), rigid.velocity.linear.z(),
          rigid.velocity.angular.x(), rigid.velocity.angular.y(),
          rigid.velocity.angular.z();

      Eigen::MatrixXd state_transition_matrix(12, 12);
      state_transition_matrix.setIdentity();
      for (size_t i = 0; i < 6; i++) {
        state_transition_matrix(i, 6 + i) = delta_time;
      }

      Eigen::MatrixXd process_noise(12, 12);
      process_noise.setIdentity();
      process_noise *= 0.00001;

      Eigen::VectorXd predicted_state = state_transition_matrix * state;
      Eigen::MatrixXd prediction_covariance =
          state_transition_matrix * rigid.covariance *
              state_transition_matrix.transpose() +
          process_noise;

      Eigen::VectorXd state_residual = state_measurement - predicted_state;
      Eigen::MatrixXd residual_covariance =
          measurement_noise + prediction_covariance;

      Eigen::MatrixXd kalman_gain =
          prediction_covariance * residual_covariance.inverse();

      Eigen::VectorXd result_state =
          predicted_state + kalman_gain * state_residual;
      Eigen::MatrixXd result_covariance =
          (Eigen::MatrixXd::Identity(kalman_gain.rows(), kalman_gain.cols()) -
           kalman_gain) *
          prediction_covariance;

      Twist result_twist;
      result_twist.linear = result_state.segment(0, 3);
      result_twist.angular = result_state.segment(3, 3);
      rigid.pose = rigid.pose * twist_to_isometry(result_twist);

      rigid.velocity.linear = result_state.segment(6, 3);
      rigid.velocity.angular = result_state.segment(9, 3);

      rigid.covariance = result_covariance;

      rigid.last_measurement = state_measurement;

      {
        tf::StampedTransform t;
        tf::poseEigenToTF(rigid.pose, t);
        t.frame_id_ = config.base_frame;
        t.child_frame_id_ = config.base_frame + "/rigid/" +
                            config.rigids[rigid_index].name + "/filtered";
        t.stamp_ = ros::Time::now();
        tf_broadcaster.sendTransform(t);
      }

      rigid.previous_visual_pose = rigid.visual_pose;
    }
  }
}
