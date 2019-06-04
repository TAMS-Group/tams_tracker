// 2018, Philipp Ruppel, ruppel@informatik.uni-hamburg.de

#pragma once

#include "common.h"

#include "config.h"

namespace tams_tracker {

class Camera {

public:
  struct Point {
    size_t marker_index = 0;
    Eigen::Vector2d position = Eigen::Vector2d::Zero(),
                    velocity = Eigen::Vector2d::Zero();
    double time = 0;
    double size = 0;
    double brightness = 0;
    double confidence = 0;
  };

  struct Trajectory {
    size_t marker_index = 0;
    std::deque<Point> points;
    double confidence = 0;
  };

private:
  /*Eigen::Vector3d pointSpaceTime(const Point &p) {
    return Eigen::Vector3d(p.position.x(), p.position.y(), p.time * 100);
  }*/

  struct View {
    cv::Mat image;
    std::vector<cv::KeyPoint> keypoints;
    std::vector<Trajectory> trajectories;
    cv::Ptr<cv::SimpleBlobDetector> blob_detector;
    std::mutex point_frame_mutex;
    std::deque<std::vector<Point>> point_frames;
    View() {
      static cv::SimpleBlobDetector::Params blob_params;

      blob_params.filterByColor = true;
      blob_params.blobColor = 255;

      // blob_params.minDistBetweenBlobs = 20;

      blob_params.minThreshold = 253;
      blob_params.maxThreshold = 255;
      blob_params.thresholdStep = 1;

      blob_params.filterByArea = false;
      // blob_params.minArea = 1;
      // blob_params.maxArea = 200;

      blob_params.filterByInertia = false;

      blob_params.filterByConvexity = false;

      blob_params.filterByCircularity = false;

      blob_detector = cv::SimpleBlobDetector::create(blob_params);
    }
  };

  double match(Trajectory &traj, size_t imarker) {
    double lo = traj.points.front().brightness;
    double hi = traj.points.front().brightness;
    for (auto &p : traj.points) {
      lo = std::min(lo, p.brightness);
      hi = std::max(hi, p.brightness);
    }
    double mid = (hi + lo) * 0.5;

    Eigen::Vector2d accu = Eigen::Vector2d::Zero();
    for (size_t ipoint = 0; ipoint < traj.points.size(); ipoint++) {
      auto &point = traj.points[ipoint];
      /*double t = point.time.toSec() * (2.0 * M_PI) *
                 config.markers[imarker].frequency;*/
      // double t =
      //    (ipoint / 60.0) * (2.0 * M_PI) * config.markers[imarker].frequency;
      double t = point.time * (2.0 * M_PI) * config.markers[imarker].frequency;
      accu.x() += std::sin(t) * (point.brightness - mid);
      accu.y() += std::cos(t) * (point.brightness - mid);
    }

    double match = accu.dot(accu) / traj.points.size();
    return match;
  };

  Config config;
  cv::VideoCapture camera;
  std::thread thread;
  volatile bool shutdown = false;
  cv::Mat x_image;
  mutable std::mutex x_mutex;
  std::vector<Trajectory> x_trajectories;
  View view;

public:
  ~Camera() { shutdown = true; }

  cv::Mat getImage() const {
    cv::Mat ret;
    {
      std::lock_guard<std::mutex> lock(x_mutex);
      x_image.copyTo(ret);
    }
    return ret;
  }

  /*std::vector<Trajectory> getTrajectories() const {
    std::vector<Trajectory> trajectories;
    {
      std::lock_guard<std::mutex> lock(x_mutex);
      for (auto &t : x_trajectories) {
        if (t.confidence >= config.min_marker_confidence) {
          trajectories.emplace_back(t);
        }
      }
    }
    return trajectories;
  }*/

  void getPoints(std::vector<Point> &points) const {
    std::lock_guard<std::mutex> lock(x_mutex);
    points.clear();
    for (auto &t : x_trajectories) {
      if (t.confidence >= config.min_marker_confidence) {
        points.emplace_back(t.points.back());
        points.back().confidence = t.confidence;
      }
    }
  }

  Camera(const Config &config_, int icam)
      : config(config_), camera(icam)
  // camera("v4l2src device=/dev/video" + std::to_string(icam))
  {

    camera.set(CV_CAP_PROP_FPS, 60);

    // camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    // camera.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

    // camera.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    // camera.set(CV_CAP_PROP_FRAME_HEIGHT, 720);

    // camera.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
    // camera.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);

    // camera.set(CV_CAP_PROP_FRAME_WIDTH, 2304);
    // camera.set(CV_CAP_PROP_FRAME_HEIGHT, 1296);

    // ROS_INFO_STREAM("video backend " << camera.getBackendName());
    /*ros::Duration(1.0).sleep();
    camera.set(CV_CAP_PROP_AUTO_EXPOSURE, 1);
    ros::Duration(1.0).sleep();
    camera.set(CV_CAP_PROP_EXPOSURE, 30);*/
    // camera.set(cv::CAP_PROP_AUTO_WB, 0);

    // camera.set(CV_CAP_PROP_AUTO_EXPOSURE, 0);
    // camera.set(CV_CAP_PROP_EXPOSURE, -30);

    // camera.set(CV_CAP_PROP_XI_AUTO_WB, 0);
    // camera.set(CV_CAP_PROP_XI_MANUAL_WB, 1);

    {
      cv::Mat img;
      if (!camera.read(img)) {
        ROS_ERROR_STREAM("camera error");
        return;
      }
    }

    thread = std::thread([this, icam]() {
      std::string name = "t markers " + std::to_string(icam);
      ROS_INFO_STREAM("marker thread " << name);
      pthread_setname_np(pthread_self(), name.c_str());

      std::mt19937 rng{std::random_device()()};
      std::normal_distribution<double> normal_distribution;
      std::uniform_real_distribution<double> uniform_real_distribution;

      // cv::Mat mask;
      cv::Mat gray;

      double time = 0.0;

      ros::Rate rate(config.frequency);

      while (!shutdown) {

        time += 1.0 / config.frequency;

        cv::Mat img;

        if (!camera.read(img)) {
          ROS_INFO("finished");
          return 0;
        }

        static auto ta_last = ros::WallTime::now();
        auto ta = ros::WallTime::now();
        ROS_INFO_STREAM_THROTTLE(0.2, "timestep " << (ta - ta_last).toSec());
        ta_last = ta;

        cv::cvtColor(img, img, CV_BGR2GRAY);

        // cv::GaussianBlur(img, img, cv::Size(3, 3), 0, 0);

        cv::cvtColor(img, img, CV_GRAY2BGR);

        cv::threshold(img, img, 200, 255, cv::THRESH_BINARY);

        view.image = img;

        view.keypoints.clear();
        view.blob_detector->detect(img, view.keypoints);

        std::vector<Point> current_points;
        for (auto &keypoint : view.keypoints) {
          Point q;
          q.position = Eigen::Vector2d(keypoint.pt.x, keypoint.pt.y);
          q.time = time;
          q.size = keypoint.size;
          q.brightness = keypoint.size;
          q.time = time;
          current_points.emplace_back(q);
        }

        {
          std::lock_guard<std::mutex> lock(view.point_frame_mutex);
          view.point_frames.emplace_back(current_points);
          if (view.point_frames.size() > 60 * 2) {
            view.point_frames.pop_front();
          }
        }

        {
          std::lock_guard<std::mutex> lock(x_mutex);
          view.image.copyTo(x_image);
        }

        auto tb = ros::WallTime::now();

        ROS_INFO_STREAM_THROTTLE(0.2, tb - ta);

        rate.sleep();
      }
    });

    std::thread([this, icam]() {
      std::string name = "trajectories " + std::to_string(icam);
      pthread_setname_np(pthread_self(), name.c_str());

      std::deque<std::vector<Point>> point_frames;

      ros::Rate rate(config.frequency);

      while (!shutdown) {

        {
          std::lock_guard<std::mutex> lock(view.point_frame_mutex);
          for (auto &frame : view.point_frames) {
            point_frames.push_back(frame);
            if (point_frames.size() > 60 * 2) {
              point_frames.pop_front();
            }
          }
          view.point_frames.clear();
        }

        if (point_frames.empty()) {
          continue;
        }

        auto pointDistance = [](const Point &a, const Point &b) {
          Eigen::Vector2d p = a.position + a.velocity * (b.time - a.time);
          return Eigen::Vector3d(p.x() - b.position.x(), p.y() - b.position.y(),
                                 (b.time - a.time) * 100)
              .squaredNorm();
        };

        view.trajectories.clear();
        for (const Point &p : point_frames.back()) {
          Trajectory trajectory;
          trajectory.points.emplace_front(p);

          while (true) {
            auto &p = trajectory.points.front();
            Point *best_point = nullptr;
            for (auto &frame : point_frames) {
              for (auto &point : frame) {
                if ((point.time < p.time) &&
                    (best_point == nullptr ||

                     (pointDistance(point, p) < pointDistance(*best_point, p))

                     /*(pointSpaceTime(point) - pointSpaceTime(p)).squaredNorm()
                        < (pointSpaceTime(*best_point) - pointSpaceTime(p))
                             .squaredNorm()*/

                     )) {
                  best_point = &point;
                }
              }
            }
            if (best_point) {
              best_point->velocity = (p.position - best_point->position) /
                                     (p.time - best_point->time);
              trajectory.points.emplace_front(*best_point);
              continue;
            } else {
              break;
            }
          }

          view.trajectories.emplace_back(trajectory);
        }

        for (auto &traj : view.trajectories) {
          traj.marker_index = 0;
          traj.confidence = -1;
          for (size_t imarker = 0; imarker < config.markers.size(); imarker++) {
            double confidence = match(traj, imarker);
            if (confidence > traj.confidence) {
              traj.marker_index = imarker;
              traj.confidence = confidence;
            }
          }
          for (auto &p : traj.points) {
            p.marker_index = traj.marker_index;
          }
        }

        {
          std::lock_guard<std::mutex> lock(x_mutex);
          x_trajectories = view.trajectories;
        }

        rate.sleep();
      }
    })
        .detach();
  }
};

} // namespace tams_tracker
