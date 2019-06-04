// 2018, Philipp Ruppel, ruppel@informatik.uni-hamburg.de

#include "common.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "calibration", 0);

  ros::NodeHandle node("~");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::Publisher marker_pub =
      node.advertise<visualization_msgs::MarkerArray>("markers", 10);

  std::vector<cv::VideoCapture> cameras;
  for (size_t i = 0; ; i++) {
    cameras.emplace_back(std::string(argv[1]) + "/" + std::to_string(i) +
                         ".avi");
    // cameras.emplace_back(i);
    if(!cameras.back().isOpened()) {
      cameras.pop_back();
      break;
    }
  }

  std::vector<cv::Mat> images(cameras.size());

  cv::Size chessboard_size(5, 4);
  // node.param("width", chessboard_size.width, chessboard_size.width);
  // node.param("height", chessboard_size.height, chessboard_size.height);

  double scale = 0.0245;
  // node.param("scale", scale, scale);

  std::vector<cv::Point3f> corners3d;
  for (int i = 0; i < chessboard_size.height; i++) {
    for (int j = 0; j < chessboard_size.width; j++) {
      corners3d.emplace_back(j * scale, i * scale, 0);
    }
  }

  std::vector<std::vector<std::vector<cv::Point2f>>> corners2d(cameras.size());

  cv::Size image_size;

  for (size_t i = 0; ros::ok(); i++) {

    bool finished = false;
    for (size_t i = 0; i < cameras.size(); i++) {
      if (!cameras[i].read(images[i])) {
        ROS_INFO("finished");
        finished = true;
        break;
      }
      image_size = cv::Size(images[i].cols, images[i].rows);
    }
    if (finished) {
      break;
    }

    /*if (i % 16) {
      continue;
    }*/

    ROS_INFO("frame");

    std::vector<std::vector<cv::Point2f>> chessboard_corners;

    for (size_t i = 0; i < cameras.size(); i++) {
      auto &image = images[i];
      chessboard_corners.emplace_back();
      bool chessboard_found = cv::findChessboardCorners(
          image, chessboard_size, chessboard_corners.back(),
          /*cv::CALIB_CB_ADAPTIVE_THRESH |*/ cv::CALIB_CB_FAST_CHECK |
              cv::CALIB_CB_FILTER_QUADS);
      /*if (!chessboard_found) {
        chessboard_corners.clear();
        break;
      }*/
      if (chessboard_found) {
        auto &image = images[i];
        cv::Mat gray_image;
        cv::cvtColor(image, gray_image, CV_BGR2GRAY);
        cv::cornerSubPix(
            gray_image, chessboard_corners.back(), cv::Size(5, 5),
            cv::Size(-1, -1),
            cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
      } else {
        chessboard_corners.back().clear();
      }
    }

    /*if (chessboard_corners.size() < images.size()) {
      continue;
    }*/

    /*for (size_t i = 0; i < cameras.size(); i++) {
      auto &image = images[i];
      cv::Mat gray_image;
      cv::cvtColor(image, gray_image, CV_BGR2GRAY);
      cv::cornerSubPix(
          gray_image, chessboard_corners[i], cv::Size(5, 5), cv::Size(-1, -1),
          cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    }*/

    /*corners3d.emplace_back();
    for (int i = 0; i < chessboard_size.height; i++) {
      for (int j = 0; j < chessboard_size.width; j++) {
        corners3d.back().emplace_back(j * scale, i * scale, 0);
      }
    }*/

    for (size_t icam = 0; icam < cameras.size(); icam++) {
      corners2d[icam].emplace_back();
      for (auto &p : chessboard_corners[icam]) {
        corners2d[icam].back().emplace_back(p);
      }
    }

    for (size_t i = 0; i < cameras.size(); i++) {
      auto &image = images[i];
      cv::drawChessboardCorners(image, chessboard_size, chessboard_corners[i],
                                true);
      cv::imshow(std::to_string(i).c_str(), image);
    }

    /*int k = cv::waitKey(250);
    if (k > 0) {
      break;
    }*/

    cv::waitKey(1);
  }

  std::vector<cv::Mat> camera_matrices(cameras.size());
  std::vector<cv::Mat> dist_coeffs(cameras.size());
  for (size_t icam = 0; icam < cameras.size(); icam++) {
    ROS_INFO("intrinsic calibration for camera %i", (int)icam);
    ROS_INFO("%i %i", (int)corners3d.size(), (int)corners2d[icam].size());
    cv::Mat tvec, rvec;
    std::vector<std::vector<cv::Point3f>> c3d;
    std::vector<std::vector<cv::Point2f>> c2d;
    for (auto &l : corners2d[icam]) {
      if (!l.empty()) {
        // ROS_INFO("view");
        c2d.push_back(l);
        c3d.push_back(corners3d);
      }
    }
    double err =
        cv::calibrateCamera(c3d, c2d, image_size, camera_matrices[icam],
                            dist_coeffs[icam], tvec, rvec, 0);
    ROS_INFO("%f", err);
  }

  size_t icenter = 2;

  std::vector<cv::Mat> translations(cameras.size(),
                                    (cv::Mat)cv::Vec3f(0, 0, 0));
  std::vector<cv::Mat> rotations(
      cameras.size(), (cv::Mat_<float>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1));
  for (size_t icam = 0; icam < cameras.size(); icam++) {
    if (icam == icenter) {
      continue;
    }
    ROS_INFO("stereo calibration %i", (int)icam);
    size_t ia = icenter;
    size_t ib = icam;
    cv::Mat r, t, e, f;
    std::vector<std::vector<cv::Point3f>> c3d;
    std::vector<std::vector<cv::Point2f>> c2da, c2db;
    for (size_t i = 0; i < corners2d[0].size(); i++) {
      if (!corners2d[ia][i].empty() && !corners2d[ib][i].empty()) {
        c3d.push_back(corners3d);
        c2da.push_back(corners2d[ia][i]);
        c2db.push_back(corners2d[ib][i]);
      }
    }
    ROS_INFO_STREAM(c3d.size() << " " << c2da.size() << " " << c2db.size());
    cv::stereoCalibrate(c3d, c2da, c2db, camera_matrices[ia], dist_coeffs[ia],
                        camera_matrices[ib], dist_coeffs[ib], image_size, r, t,
                        e, f);
    ROS_INFO_STREAM("ready");
    translations[ib] = t;
    rotations[ib] = r;
  }

  std::vector<Eigen::Affine3d> camera_poses(cameras.size());
  for (size_t icam = 0; icam < cameras.size(); icam++) {
    Eigen::Matrix3d r;
    cv::cv2eigen(rotations[icam], r);
    Eigen::Vector3d t;
    cv::cv2eigen(translations[icam], t);
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation() = t;
    pose.linear() = r;
    pose = pose.inverse();
    camera_poses[icam] = pose;
  }

#if 0
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.header.frame_id = "/world";
  marker.pose.orientation.w = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.scale.x = 0.01;
  for (size_t icam = 0; icam < cameras.size(); icam++) {
    for (auto &points : corners2d[icam]) {
      std::vector<cv::Point2f> pp;
      /*cv::Mat pp;
      cv::undistortPoints(points, pp, camera_matrices[icam], dist_coeffs[icam],
                          rotations[icam], translations[icam]);
      for (auto &p : pp) {
        marker.points.emplace_back();
        marker.points.back().x = p.x;
        marker.points.back().y = p.y;
        marker.points.back().z = p.z;
      }*/
      cv::undistortPoints(points, pp, camera_matrices[icam], dist_coeffs[icam]);
      for (auto &p : pp) {
        /*marker.points.emplace_back();
        marker.points.back().x = translations[icam][0];
        marker.points.back().y = translations[icam][1];
        marker.points.back().z = translations[icam][2];

        marker.points.emplace_back();
        cv::Vec3f q = rotations[icam] * cv marker.points.back().x =
                          translations[icam][0];
        marker.points.back().y = translations[icam][1];
        marker.points.back().z = translations[icam][2];*/

        Eigen::Vector3d a = camera_poses[icam].translation();
        marker.points.emplace_back();
        marker.points.back().x = a.x();
        marker.points.back().y = a.y();
        marker.points.back().z = a.z();

        Eigen::Vector3d b = camera_poses[icam] * Eigen::Vector3d(p.x, p.y, 1.0);

        b = a + (b - a) * 40;

        marker.points.emplace_back();
        marker.points.back().x = b.x();
        marker.points.back().y = b.y();
        marker.points.back().z = b.z();
      }
      break;
    }
  }
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.push_back(marker);
  marker_pub.publish(marker_array);
#endif

  cv::FileStorage fs("calibration.yml", cv::FileStorage::WRITE);
  fs << "cameras"
     << "[";
  for (size_t icam = 0; icam < cameras.size(); icam++) {
    fs << "{";
    fs << "t" << translations[icam];
    fs << "r" << rotations[icam];
    fs << "k" << camera_matrices[icam];
    fs << "d" << dist_coeffs[icam];
    fs << "}";
  }
  fs << "]";
  fs.release();

  ros::Duration(10.0).sleep();
}
