// 2018, Philipp Ruppel, ruppel@informatik.uni-hamburg.de

#include "common.h"

#include <geometry_msgs/Vector3.h>
#include <tf/transform_listener.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "publish_angles", 0);

  ros::NodeHandle node("~");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::Duration(1.0).sleep();

  std::string parent_name = argv[1];

  std::vector<std::string> frame_names;
  for (size_t i = 2; i < argc; i++) {
    frame_names.push_back(argv[i]);
  }

  tf::TransformListener tf_listener;

  auto getOrientation = [&](const std::string &frame_name) {
    auto time = ros::Time(0);
    /*std::string parent_name;
    bool ok = tf_listener.getParent(frame_name, time, parent_name);
    if (!ok) {
      ROS_ERROR_STREAM("failed to get parent frame");
      throw std::runtime_error("failed to get parent frame");
    }*/
    tf::StampedTransform transform;
    tf_listener.waitForTransform(parent_name, frame_name, time,
                                 ros::Duration(3.0));
    tf_listener.lookupTransform(parent_name, frame_name, time, transform);
    return transform.getRotation();
  };

  std::vector<tf::Quaternion> inverse_reference_orientations;
  for (auto &frame_name : frame_names) {
    inverse_reference_orientations.push_back(
        getOrientation(frame_name).inverse());
  }

  std::vector<ros::Publisher> angle_publishers;
  for (auto &frame_name : frame_names) {
    angle_publishers.emplace_back(node.advertise<geometry_msgs::Vector3>(
        frame_name + "/orientation", 1000));
  }

  while (ros::ok()) {
    for (size_t i = 0; i < frame_names.size(); i++) {
      tf::Quaternion quat =
          inverse_reference_orientations[i] * getOrientation(frame_names[i]);
      tf::Vector3 vec = quat.getAxis() * (quat.getAngle() * 180 / M_PI);
      geometry_msgs::Vector3 msg;
      msg.x = vec.x();
      msg.y = vec.y();
      msg.z = vec.z();
      angle_publishers[i].publish(msg);
    }
    ros::Duration(0.01).sleep();
  }
}
