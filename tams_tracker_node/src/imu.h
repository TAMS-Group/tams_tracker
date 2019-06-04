// 2018, Philipp Ruppel, ruppel@informatik.uni-hamburg.de

#pragma once

#include "common.h"

namespace tams_tracker {

class Imu {
  ros::NodeHandle node;
  ros::Subscriber subscriber;
  mutable std::mutex x_mutex;
  Eigen::Vector3d x_angular_velocity = Eigen::Vector3d::Zero();
  bool ok = false;

public:
  Imu() {}
  Imu(const std::string &topic_name) {
    if (!topic_name.empty()) {
      ROS_INFO_STREAM("imu topic " << topic_name);
      boost::function<void(const sensor_msgs::Imu &)> callback =
          [this](const sensor_msgs::Imu &msg) {
            std::lock_guard<std::mutex> lock(x_mutex);
            ok = true;
            tf::vectorMsgToEigen(msg.angular_velocity, x_angular_velocity);
          };
      subscriber = node.subscribe<sensor_msgs::Imu>(topic_name, 10, callback);
    }
  }
  bool hasReceivedMessages() const { return ok; }
  Eigen::Vector3d getAngularVelocity() const {
    Eigen::Vector3d angular_velocity;
    {
      std::lock_guard<std::mutex> lock(x_mutex);
      angular_velocity = x_angular_velocity;
    }
    /*ROS_INFO_STREAM("imu " << angular_velocity.x() << " "
                           << angular_velocity.y() << " "
                           << angular_velocity.z());*/
    return angular_velocity;
  }
};

} // namespace tams_tracker
