// 2018, Philipp Ruppel, ruppel@informatik.uni-hamburg.de

#pragma once

#include "common.h"

namespace tams_tracker {

struct Config {

  struct Marker {
    uint32_t id = 0;
    std::string name;
    double frequency = 1.0;
    std::string pattern;
  };
  std::vector<Marker> markers;

  struct Rigid {
    struct Marker {
      size_t marker_index;
      Eigen::Vector3d position;
    };
    std::vector<Marker> markers;
    std::string name;
    uint32_t id;
    std::string imu_topic;
    Eigen::Quaterniond imu_orientation = Eigen::Quaterniond::Identity();
  };
  std::vector<Rigid> rigids;

  double tracking_time = 2.0;
  double tf_frequency = 60.0;
  double frequency = 60.0;
  double visualization_frequency = 60.0;
  std::string base_frame = "/tams_tracker";
  std::string visualization_marker_topic = "visualization_markers";
  double visualization_ray_length = 100;
  double visualization_ray_width = 0.05;
  double visualization_marker_size = 0.15;
  double max_reprojection_error = 0.01;
  double marker_radius = 0.1;
  double min_distance_to_camera = 1.0;
  std::string marker_array_stamped_topic = "markers";
  std::string rigid_array_stamped_topic = "rigids";
  std::string peak_array_stamped_topic = "peaks";
  std::string ray_array_stamped_topic = "rays";
  int ransac_iterations = 1000;
  double min_marker_confidence = 200.0;
  double max_marker_velocity = 20.0;
  double min_calibration_sample_distance = 50.0;

  void load() {
    ros::NodeHandle node("~");

    node.param("tracking_time", tracking_time, tracking_time);
    ROS_INFO_STREAM("tracking_time: " << tracking_time);

    node.param("frequency", frequency, frequency);
    ROS_INFO_STREAM("frequency: " << frequency);

    tf_frequency = frequency;
    node.param("tf_frequency", tf_frequency, tf_frequency);
    ROS_INFO_STREAM("tf_frequency: " << tf_frequency);

    visualization_frequency = frequency;
    node.param("visualization_frequency", visualization_frequency,
               visualization_frequency);
    ROS_INFO_STREAM("visualization_frequency: " << visualization_frequency);

    node.param("base_frame", base_frame, base_frame);
    ROS_INFO_STREAM("base_frame: " << base_frame);

    node.param("visualization_marker_topic", visualization_marker_topic,
               visualization_marker_topic);
    ROS_INFO_STREAM(
        "visualization_marker_topic: " << visualization_marker_topic);

    node.param("visualization_ray_length", visualization_ray_length,
               visualization_ray_length);
    ROS_INFO_STREAM("visualization_ray_length: " << visualization_ray_length);

    node.param("visualization_ray_width", visualization_ray_width,
               visualization_ray_width);
    ROS_INFO_STREAM("visualization_ray_width: " << visualization_ray_width);

    node.param("visualization_marker_size", visualization_marker_size,
               visualization_marker_size);
    ROS_INFO_STREAM("visualization_marker_size: " << visualization_marker_size);

    node.param("max_reprojection_error", max_reprojection_error,
               max_reprojection_error);
    ROS_INFO_STREAM("max_reprojection_error: " << max_reprojection_error);

    node.param("marker_array_stamped_topic", marker_array_stamped_topic,
               marker_array_stamped_topic);
    ROS_INFO_STREAM(
        "marker_array_stamped_topic: " << marker_array_stamped_topic);

    node.param("rigid_array_stamped_topic", rigid_array_stamped_topic,
               rigid_array_stamped_topic);
    ROS_INFO_STREAM("rigid_array_stamped_topic: " << rigid_array_stamped_topic);

    node.param("peak_array_stamped_topic", peak_array_stamped_topic,
               peak_array_stamped_topic);
    ROS_INFO_STREAM("peak_array_stamped_topic: " << peak_array_stamped_topic);

    node.param("ray_array_stamped_topic", ray_array_stamped_topic,
               ray_array_stamped_topic);
    ROS_INFO_STREAM("ray_array_stamped_topic: " << ray_array_stamped_topic);

    node.param("min_distance_to_camera", min_distance_to_camera,
               min_distance_to_camera);
    ROS_INFO_STREAM("min_distance_to_camera: " << min_distance_to_camera);

    node.param("min_marker_confidence", min_marker_confidence,
               min_marker_confidence);
    ROS_INFO_STREAM("min_marker_confidence: " << min_marker_confidence);

    node.param("ransac_iterations", ransac_iterations, ransac_iterations);
    ROS_INFO_STREAM("ransac_iterations: " << ransac_iterations);

    node.param("max_marker_velocity", max_marker_velocity, max_marker_velocity);
    ROS_INFO_STREAM("max_marker_velocity: " << max_marker_velocity);

    node.param("marker_radius", marker_radius, marker_radius);
    ROS_INFO_STREAM("marker_radius: " << marker_radius);

    node.param("min_calibration_sample_distance",
               min_calibration_sample_distance,
               min_calibration_sample_distance);
    ROS_INFO_STREAM(
        "min_calibration_sample_distance: " << min_calibration_sample_distance);

    {
      XmlRpc::XmlRpcValue markers;
      node.getParam("markers", markers);
      if (markers.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("the markers parameter should be an array");
      } else {
        ROS_INFO_STREAM("tracking " << markers.size() << " markers");
        for (size_t i = 0; i < markers.size(); i++) {
          Marker marker;

          auto &m = markers[i];

          if (m["id"].getType() == XmlRpc::XmlRpcValue::TypeInt)
            marker.id = (int)m["id"];

          if (m["name"].getType() == XmlRpc::XmlRpcValue::TypeString)
            marker.name = (std::string)m["name"];

          if (m["frequency"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
            marker.frequency = (double)m["frequency"];

          if (m["frequency"].getType() == XmlRpc::XmlRpcValue::TypeInt)
            marker.frequency = (int)m["frequency"];

          if (m["pattern"].getType() == XmlRpc::XmlRpcValue::TypeString)
            marker.pattern = (std::string)m["pattern"];

          ROS_INFO_STREAM("marker"
                          << " id:" << marker.id
                          << " pattern:" << marker.pattern << " name:\""
                          << marker.name << "\"");

          this->markers.push_back(marker);
        }
      }
    }

    {
      XmlRpc::XmlRpcValue rr;
      node.getParam("rigids", rr);
      if (rr.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_INFO("the rigids parameter should be an array");
      } else {
        ROS_INFO_STREAM("tracking " << rr.size() << " rigids");
        for (size_t i = 0; i < rr.size(); i++) {
          Rigid rigid;
          auto &r = rr[i];

          if (r["id"].getType() == XmlRpc::XmlRpcValue::TypeInt)
            rigid.id = (int)r["id"];

          if (r["name"].getType() == XmlRpc::XmlRpcValue::TypeString)
            rigid.name = (std::string)r["name"];

          for (size_t j = 0; j < r["markers"].size(); j++) {
            Rigid::Marker m;
            size_t id = (int)r["markers"][j]["id"];
            for (size_t mi = 0; mi < markers.size(); mi++) {
              if (markers[mi].id == id) {
                m.marker_index = mi;
              }
            }
            m.position.x() = (double)r["markers"][j]["position"]["x"];
            m.position.y() = (double)r["markers"][j]["position"]["y"];
            m.position.z() = (double)r["markers"][j]["position"]["z"];
            rigid.markers.push_back(m);
          }

          if (r["imu"].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
            rigid.imu_topic = (std::string)r["imu"]["topic"];
            rigid.imu_orientation.x() = (double)r["imu"]["orientation"]["x"];
            rigid.imu_orientation.y() = (double)r["imu"]["orientation"]["y"];
            rigid.imu_orientation.z() = (double)r["imu"]["orientation"]["z"];
            rigid.imu_orientation.w() = (double)r["imu"]["orientation"]["w"];
          }

          ROS_INFO_STREAM("rigid"
                          << " id:" << rigid.id << " name:\"" << rigid.name
                          << "\"");
          for (auto &m : rigid.markers) {
            ROS_INFO_STREAM("rigid marker"
                            << " id: " << markers[m.marker_index].id << " x:"
                            << m.position.x() << " y:" << m.position.y()
                            << " z:" << m.position.z());
          }

          this->rigids.push_back(rigid);
        }
      }
    }
  }
};

} // namespace tams_tracker
