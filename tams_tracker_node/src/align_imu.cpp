// 2018, Philipp Ruppel, ruppel@informatik.uni-hamburg.de

#include "common.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <tams_tracker_msgs/RigidArrayStamped.h>

int main(int argc, char **argv) {

  using namespace tams_tracker;

  if (argc < 2) {
    ROS_ERROR("usage: rosrun tams_tracker tams_tracker_align_imu <data.bag>");
    return -1;
  }

  ros::init(argc, argv, "tams_tracker_align_imu", 0);

  ros::NodeHandle node("~");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  struct OrientationSample {
    double time = 0.0;
    Eigen::Quaterniond rigid_orientation = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond imu_orientation = Eigen::Quaterniond::Identity();
  };

  std::vector<OrientationSample> orientation_samples;

  {
    OrientationSample orientation_sample;
    bool rigid_orientation_valid = false;
    bool imu_orientation_valid = false;

    rosbag::Bag bag;
    bag.open(argv[1], rosbag::bagmode::Read);

    for (auto &message : rosbag::View(bag)) {

      bool new_data = false;

      if (message.getTopic() == "/tams_tracker_node/rigids") {
        if (auto msg =
                message.instantiate<tams_tracker_msgs::RigidArrayStamped>()) {
          if (rigid_orientation_valid = (msg->rigids.size() == 1)) {
            tf::quaternionMsgToEigen(msg->rigids[0].pose.orientation,
                                     orientation_sample.rigid_orientation);
            orientation_sample.time = msg->header.stamp.toSec();
            new_data = true;
          }
        }
      }

      if (message.getTopic() == "/tams_tracker_imu/data_raw") {
        if (auto msg = message.instantiate<sensor_msgs::Imu>()) {
          tf::quaternionMsgToEigen(msg->orientation,
                                   orientation_sample.imu_orientation);
          imu_orientation_valid = true;
          orientation_sample.time = msg->header.stamp.toSec();
          new_data = true;
        }
      }

      if (new_data && rigid_orientation_valid && imu_orientation_valid) {
        orientation_samples.push_back(orientation_sample);
      }
    }
  }

  ROS_INFO_STREAM(orientation_samples.size() << " orientation samples");

  struct RotationSample {
    Eigen::Vector3d rigid_rotation = Eigen::Vector3d::Zero();
    Eigen::Vector3d imu_rotation = Eigen::Vector3d::Zero();
  };

  std::vector<RotationSample> rotation_samples;

  for (auto &s : orientation_samples) {
    for (auto &t : orientation_samples) {
      double dt = s.time - t.time;
      if (dt > 0.3 && dt < 0.7) {
        Eigen::AngleAxisd rigid_relative = Eigen::AngleAxisd(
            s.rigid_orientation.inverse() * t.rigid_orientation);
        Eigen::AngleAxisd imu_relative =
            Eigen::AngleAxisd(s.imu_orientation.inverse() * t.imu_orientation);
        RotationSample rotation_sample;
        rotation_sample.rigid_rotation =
            rigid_relative.axis() * rigid_relative.angle();
        rotation_sample.imu_rotation =
            imu_relative.axis() * imu_relative.angle();
        rotation_samples.push_back(rotation_sample);
      }
    }
  }

  ROS_INFO_STREAM(rotation_samples.size() << " rotation samples");

  /*for (auto &rotation_sample : rotation_samples) {
    ROS_INFO_STREAM(rotation_sample.rigid_rotation.x()
                    << "," << rotation_sample.rigid_rotation.y() << ","
                    << rotation_sample.rigid_rotation.z() << " "
                    << rotation_sample.imu_rotation.x() << ","
                    << rotation_sample.imu_rotation.y() << ","
                    << rotation_sample.imu_rotation.z());
  }*/

  Eigen::Quaterniond imu_to_rigid = Eigen::Quaterniond::Identity();

  Eigen::MatrixXd gradients(rotation_samples.size() * 3, 3);
  Eigen::VectorXd residuals(rotation_samples.size() * 3);

  Eigen::Matrix3d rx, ry, rz;
  rx << 0, 0, 0, 0, 0, -1, 0, 1, 0;
  ry << 0, 0, 1, 0, 0, 0, -1, 0, 0;
  rz << 0, -1, 0, 1, 0, 0, 0, 0, 0;

  ROS_INFO_STREAM("optimizing imu alignment calibration");

  while (true) {

    for (size_t i = 0; i < rotation_samples.size(); i++) {
      auto &rotation_sample = rotation_samples[i];

      Eigen::Vector3d a = rotation_sample.rigid_rotation;
      Eigen::Vector3d b = imu_to_rigid * rotation_sample.imu_rotation;

      double l = a.norm() + b.norm();
      a = a.normalized() * l;
      b = b.normalized() * l;

      residuals.segment<3>(i * 3) = a - b;

      Eigen::Vector3d tx = rx * b;
      Eigen::Vector3d ty = ry * b;
      Eigen::Vector3d tz = rz * b;

      gradients.block<3, 1>(i * 3, 0) = tx;
      gradients.block<3, 1>(i * 3, 1) = ty;
      gradients.block<3, 1>(i * 3, 2) = tz;
    }

    Eigen::Vector3d solution = Eigen::Vector3d::Zero();
    solution = gradients.colPivHouseholderQr().solve(residuals);

    ROS_INFO_STREAM(solution.norm());

    if (solution.norm() < 1e-15) {
      ROS_INFO_STREAM("ready");
      break;
    }

    imu_to_rigid = Eigen::AngleAxisd(solution.norm(), solution.normalized()) *
                   imu_to_rigid;
  }

  ROS_INFO_STREAM("imu_to_rigid:");
  ROS_INFO_STREAM("  x: " << imu_to_rigid.x());
  ROS_INFO_STREAM("  y: " << imu_to_rigid.y());
  ROS_INFO_STREAM("  z: " << imu_to_rigid.z());
  ROS_INFO_STREAM("  w: " << imu_to_rigid.w());
}
