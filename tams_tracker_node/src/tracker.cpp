// 2018, Philipp Ruppel, ruppel@informatik.uni-hamburg.de

#include "common.h"

#include "calibration.h"
#include "camera.h"
#include "markers.h"
#include "rays.h"
#include "rigids.h"

#include <tams_tracker_msgs/MarkerArrayStamped.h>
#include <tams_tracker_msgs/PeakArrayStamped.h>
#include <tams_tracker_msgs/RayArrayStamped.h>
#include <tams_tracker_msgs/RigidArrayStamped.h>

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include <pthread.h>

int main(int argc, char **argv) {

  using namespace tams_tracker;

  // omp_set_dynamic(0);
  // omp_set_num_threads(1);
  cv::setNumThreads(0);

  ros::init(argc, argv, "node");

  ros::NodeHandle node("~");

  // ros::MultiThreadedSpinner spinner(4);

  Config config;
  config.load();

  std::string mode = "tracking";
  node.param("mode", mode, mode);
  for (auto &c : mode) {
    c = std::tolower(c);
  }

  ROS_INFO_STREAM("mode " << mode);

  /*if (mode == "capture") {
    tams_tracker_msgs::MarkerArrayStamped x_msg;
    std::mutex x_mutex;
    boost::function<void(const tams_tracker_msgs::MarkerArrayStamped &)>
        callback = [&](const tams_tracker_msgs::MarkerArrayStamped &msg) {
          std::lock_guard<std::mutex> lock(x_mutex);
          x_msg = msg;
        };
    ros::Subscriber sub = node.subscribe<tams_tracker_msgs::MarkerArrayStamped>(
        config.marker_array_stamped_topic, 10, callback);
    while (ros::ok()) {
      spinner.spin();
      char key = 0;
      fcntl(0, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK);
      if (read(0, &key, 1)) {
        ROS_INFO_STREAM("key " << key);
      }
    }
    return 0;
  }*/

  Calibration calibration;
  calibration.load();

  std::deque<Camera> cameras;
  for (size_t i = 0; i < calibration.view_count; i++) {
    cameras.emplace_back(config, i);
  }

  std::string rigid_name = "";
  node.param("rigid", rigid_name, rigid_name);

  if (mode == "tracking") {

    RayBuilder ray_builder(config, calibration);
    MarkerDetector marker_detector(config);
    RigidTracker rigid_tracker(config);

    std::thread([&]() {
      pthread_setname_np(pthread_self(), "markers&poses");
      ros::Rate rate(config.frequency);
      std::vector<Ray> rays;
      std::vector<Marker> markers;
      while (ros::ok()) {
        ray_builder.update(cameras);
        ray_builder.getRays(rays);
        marker_detector.update(rays);
        marker_detector.getMarkers(markers);
        rigid_tracker.update(rays, markers);
        rate.sleep();
      }
    })
        .detach();

    std::thread([&]() {
      pthread_setname_np(pthread_self(), "peak publisher");
      ros::Rate rate(config.frequency);
      ros::Publisher pub = node.advertise<tams_tracker_msgs::PeakArrayStamped>(
          config.peak_array_stamped_topic, 10);
      tams_tracker_msgs::PeakArrayStamped msg;
      std::vector<Camera::Point> points;
      while (ros::ok()) {
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = config.base_frame;
        msg.peaks.clear();
        for (size_t icam = 0; icam < cameras.size(); icam++) {
          auto &cam = cameras[icam];
          cam.getPoints(points);
          for (auto &point : points) {
            msg.peaks.emplace_back();
            msg.peaks.back().marker_id = config.markers[point.marker_index].id;
            msg.peaks.back().marker_name =
                config.markers[point.marker_index].name;
            msg.peaks.back().brightness = point.brightness;
            msg.peaks.back().marker_confidence = point.confidence;
            msg.peaks.back().x = point.position.x();
            msg.peaks.back().y = point.position.y();
            msg.peaks.back().camera_index = icam;
          }
        }
        pub.publish(msg);
        rate.sleep();
      }
    })
        .detach();

    std::thread([&]() {
      pthread_setname_np(pthread_self(), "ray publisher");
      ros::Rate rate(config.frequency);
      ros::Publisher pub = node.advertise<tams_tracker_msgs::RayArrayStamped>(
          config.ray_array_stamped_topic, 10);
      tams_tracker_msgs::RayArrayStamped msg;
      std::vector<Ray> rays;
      while (ros::ok()) {
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = config.base_frame;
        msg.rays.clear();
        ray_builder.getRays(rays);
        for (auto &ray : rays) {
          msg.rays.emplace_back();
          msg.rays.back().marker_id = config.markers[ray.marker_index].id;
          msg.rays.back().marker_name = config.markers[ray.marker_index].name;
          msg.rays.back().origin.x = ray.origin.x();
          msg.rays.back().origin.y = ray.origin.y();
          msg.rays.back().origin.z = ray.origin.z();
          msg.rays.back().direction.x = ray.direction.x();
          msg.rays.back().direction.y = ray.direction.y();
          msg.rays.back().direction.z = ray.direction.z();
        }
        pub.publish(msg);
        rate.sleep();
      }
    })
        .detach();

    std::thread([&]() {
      pthread_setname_np(pthread_self(), "rigid publisher");
      ros::Rate rate(config.frequency);
      std::vector<Rigid> rigids;
      ros::Publisher pub = node.advertise<tams_tracker_msgs::RigidArrayStamped>(
          config.rigid_array_stamped_topic, 10);
      tams_tracker_msgs::RigidArrayStamped msg;
      while (ros::ok()) {
        rigid_tracker.getRigids(rigids);
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = config.base_frame;
        msg.rigids.resize(rigids.size());
        for (size_t i = 0; i < rigids.size(); i++) {
          auto &rigid = rigids[i];
          msg.rigids[i].id = config.rigids[rigid.rigid_index].id;
          msg.rigids[i].name = config.rigids[rigid.rigid_index].name;
          tf::poseEigenToMsg(rigid.pose, msg.rigids[i].pose);
        }
        pub.publish(msg);
        rate.sleep();
      }
    })
        .detach();

    std::thread([&]() {
      pthread_setname_np(pthread_self(), "marker publisher");
      ros::Rate rate(config.frequency);
      std::vector<Marker> markers;
      ros::Publisher pub =
          node.advertise<tams_tracker_msgs::MarkerArrayStamped>(
              config.marker_array_stamped_topic, 10);
      tams_tracker_msgs::MarkerArrayStamped msg;
      while (ros::ok()) {
        marker_detector.getMarkers(markers);
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = config.base_frame;
        msg.markers.resize(markers.size());
        for (size_t mi = 0; mi < markers.size(); mi++) {
          auto &m = markers[mi];
          msg.markers[mi].position.x = m.position.x();
          msg.markers[mi].position.y = m.position.y();
          msg.markers[mi].position.z = m.position.z();
          msg.markers[mi].id = config.markers[m.marker_index].id;
          msg.markers[mi].name = config.markers[m.marker_index].name;
        }
        pub.publish(msg);
        rate.sleep();
      }
    })
        .detach();

    std::thread([&]() {
      pthread_setname_np(pthread_self(), "tf broadcaster");
      tf::TransformBroadcaster br;
      ros::Rate rate(config.tf_frequency);
      while (ros::ok()) {
        auto stamp = ros::Time::now();
        std::vector<tf::StampedTransform> tt;
        for (size_t i = 0; i < calibration.camera_poses.size(); i++) {
          tf::StampedTransform t;
          tf::poseEigenToTF(calibration.camera_poses[i], t);
          t.frame_id_ = config.base_frame;
          t.child_frame_id_ =
              config.base_frame + "/camera/" + std::to_string(i);
          t.stamp_ = stamp;
          tt.push_back(t);
        }
        {
          std::vector<Rigid> rigids;
          rigid_tracker.getRigids(rigids);
          for (auto &rigid : rigids) {
            tf::StampedTransform t;
            tf::poseEigenToTF(rigid.pose, t);
            t.frame_id_ = config.base_frame;
            t.child_frame_id_ = config.base_frame + "/" + "rigid" + "/" +
                                config.rigids[rigid.rigid_index].name;
            t.stamp_ = stamp;
            tt.push_back(t);
          }
        }
        br.sendTransform(tt);
        rate.sleep();
      }
    })
        .detach();

    std::thread([&]() {
      pthread_setname_np(pthread_self(), "viz markers");
      ros::NodeHandle node("~");
      ros::Publisher marker_pub =
          node.advertise<visualization_msgs::MarkerArray>(
              config.visualization_marker_topic, 10);
      std::vector<Ray> rays;
      std::vector<Rigid> rigids;
      std::vector<Marker> markers;
      ros::Rate rate(config.visualization_frequency);
      std::map<uint32_t, std_msgs::ColorRGBA> colors;
      auto getColor = [&](uint32_t i) {
        auto it = colors.find(i);
        if (it != colors.end()) {
          return it->second;
        }
        std_msgs::ColorRGBA color;
        color.r = rand() * 1.0 / RAND_MAX * 0.7 + 0.3;
        color.g = rand() * 1.0 / RAND_MAX * 0.7 + 0.3;
        color.b = rand() * 1.0 / RAND_MAX * 0.7 + 0.3;
        color.a = 1.0;
        colors[i] = color;
        return color;
      };
      while (ros::ok()) {
        visualization_msgs::MarkerArray marker_array;
        {
          ray_builder.getRays(rays);
          visualization_msgs::Marker marker;
          marker.type = visualization_msgs::Marker::LINE_LIST;
          marker.action = visualization_msgs::Marker::ADD;
          marker.header.frame_id = config.base_frame;
          marker.ns = "rays";
          marker.pose.orientation.w = 1.0;
          marker.color.a = 1.0;
          marker.scale.x = config.visualization_ray_width;
          for (auto &ray : rays) {
            Eigen::Vector3d a = ray.origin;
            Eigen::Vector3d b =
                ray.origin + ray.direction * config.visualization_ray_length;
            marker.points.emplace_back();
            marker.points.back().x = a.x();
            marker.points.back().y = a.y();
            marker.points.back().z = a.z();
            marker.points.emplace_back();
            marker.points.back().x = b.x();
            marker.points.back().y = b.y();
            marker.points.back().z = b.z();
            auto color = getColor(ray.marker_index);
            marker.colors.push_back(color);
            marker.colors.push_back(color);
          }
          marker_array.markers.push_back(marker);
        }
        {
          std::vector<Rigid> rigids;
          rigid_tracker.getRigids(rigids);
          visualization_msgs::Marker marker;
          marker.type = visualization_msgs::Marker::SPHERE_LIST;
          marker.action = visualization_msgs::Marker::ADD;
          marker.header.frame_id = config.base_frame;
          marker.ns = "rigids";
          marker.pose.orientation.w = 1.0;
          marker.color.a = 1.0;
          marker.scale.x = config.visualization_marker_size;
          for (auto &rigid : rigids) {
            for (auto &m : config.rigids[rigid.rigid_index].markers) {
              Eigen::Vector3d a = rigid.pose * m.position;
              marker.points.emplace_back();
              marker.points.back().x = a.x();
              marker.points.back().y = a.y();
              marker.points.back().z = a.z();
              // marker.colors.push_back(getColor(rigid.rigid_index + 100));
              std_msgs::ColorRGBA color;
              color.r = color.g = color.b = color.a = 1.0;
              marker.colors.push_back(color);
            }
          }
          marker_array.markers.push_back(marker);
        }
        {
          marker_detector.getMarkers(markers);
          visualization_msgs::Marker marker;
          marker.type = visualization_msgs::Marker::SPHERE_LIST;
          marker.action = visualization_msgs::Marker::ADD;
          marker.header.frame_id = config.base_frame;
          marker.ns = "markers";
          marker.pose.orientation.w = 1.0;
          marker.color.a = 1.0;
          marker.scale.x = config.visualization_marker_size;
          for (auto &m : markers) {
            Eigen::Vector3d a = m.position;
            marker.points.emplace_back();
            marker.points.back().x = a.x();
            marker.points.back().y = a.y();
            marker.points.back().z = a.z();
            marker.colors.push_back(getColor(m.marker_index));
          }
          marker_array.markers.push_back(marker);
        }
        marker_pub.publish(marker_array);
        rate.sleep();
      }
    })
        .detach();

    std::thread([&]() {
      pthread_setname_np(pthread_self(), "opencv viz");
      while (ros::ok()) {
        std::vector<Camera::Point> points;
        for (size_t i = 0; i < calibration.view_count; i++) {
          cv::Mat image = cameras[i].getImage();
          cameras[i].getPoints(points);
          if (image.rows == 0 || image.cols == 0) {
            // ROS_INFO_STREAM("image empty " << i);
            continue;
          }
          for (auto &p : points) {
            /*for (size_t j = 1; j < traj.points.size(); j++) {
              Eigen::Vector2d a = traj.points[j - 1].position;
              Eigen::Vector2d b = traj.points[j].position;
              cv::line(image, cv::Point2d(a.x(), a.y()),
                       cv::Point2d(b.x(), b.y()), cv::Scalar(255, 0, 255, 255));
            }*/
            for (int i = 0; i < 2; i++) {
              cv::putText(
                  image,
                  (std::to_string(config.markers[p.marker_index].id) /*+ " " +
                   std::to_string(traj.points.size())*/
                   + " " + std::to_string((int)p.confidence))
                      .c_str(),
                  cv::Point(p.position.x() - i, p.position.y() - i),
                  cv::FONT_HERSHEY_COMPLEX_SMALL, 0.7,
                  i ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 0, 0), 1.5,
                  CV_AA);
            }
            cv::circle(image, cv::Point(p.position.x(), p.position.y()),
                       p.brightness * 0.05, cv::Scalar(0, 0, 0), 1.5, CV_AA);
            // cv::circle(image, cv::Point(p.position.x(),
            // p.position.y()), 10.0,
            //           cv::Scalar(1.0, 1.0, 1.0), -1.0, CV_AA);
          }
          /*cv::putText(image, std::to_string(camera_fps).c_str(),
                      cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                      cv::Scalar(0, 255, 255));*/
          cv::imshow(std::to_string(i).c_str(), image);
          cv::waitKey(1);
        }
        cv::waitKey(10);
      }
    })
        .detach();

    ros::spin();
  }
}
