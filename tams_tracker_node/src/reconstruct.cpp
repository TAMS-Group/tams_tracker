#include "common.h"

struct Marker {
  int id = 0;
  Eigen::Vector3d position;
};

struct View {
  std::vector<Marker> markers;
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "node");

  ros::NodeHandle node;

  ros::AsyncSpinner spinner(4);
  spinner.start();

  if (argc < 2) {
    ROS_ERROR_STREAM("usage: rosrun tams_tracker_node tams_tracker_reconstruct "
                     "marker_views_123.json");
    return -1;
  }

  ros::Publisher marker_pub = node.advertise<visualization_msgs::MarkerArray>(
      "/tams_tracker_node/visualization_markers", 10, true);
  auto publish_markers = [&](const std::vector<Marker> &markers) {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.frame_id = "tams_tracker";
    marker.ns = "reconstruction";
    marker.pose.orientation.w = 1.0;
    marker.color.a = 1.0;
    marker.scale.x = 0.01;
    for (auto &m : markers) {
      marker.points.emplace_back();
      marker.points.back().x = m.position.x();
      marker.points.back().y = m.position.y();
      marker.points.back().z = m.position.z();
      std_msgs::ColorRGBA color;
      color.r = color.g = color.b = color.a = 1.0;
      marker.colors.push_back(color);
    }
    marker_array.markers.push_back(marker);
    marker_pub.publish(marker_array);
  };

  std::vector<View> views;
  {
    cv::FileStorage fs(std::string(argv[1]), cv::FileStorage::READ);
    if (!fs.isOpened()) {
      ROS_ERROR_STREAM("failed to open file");
      return -1;
    }
    for (auto v : fs["views"]) {
      View view;
      for (auto m : v["markers"]) {
        Marker marker;
        m["id"] >> marker.id;
        m["position"]["x"] >> marker.position.x();
        m["position"]["y"] >> marker.position.y();
        m["position"]["z"] >> marker.position.z();
        view.markers.push_back(marker);
      }
      views.push_back(view);
    }
  }

  {
    std::vector<Marker> all_markers;
    for (auto &view : views) {
      for (auto &m : view.markers) {
        all_markers.push_back(m);
      }
    }
    publish_markers(all_markers);
  }

  for (auto &view : views) {
    ROS_INFO_STREAM("{");
    for (auto &m : view.markers) {
      ROS_INFO_STREAM(" {");
      ROS_INFO_STREAM("  id: " << m.id);
      ROS_INFO_STREAM("  x: " << m.position.x());
      ROS_INFO_STREAM("  y: " << m.position.y());
      ROS_INFO_STREAM("  z: " << m.position.z());
      ROS_INFO_STREAM(" }");
    }
    ROS_INFO_STREAM("}");
  }

  std::vector<Eigen::Isometry3d> view_poses(views.size(),
                                            Eigen::Isometry3d::Identity());

  std::vector<size_t> marker_index_to_id;
  std::map<size_t, size_t> marker_id_to_index;
  for (auto &view : views) {
    for (auto &m : view.markers) {
      if (marker_id_to_index.find(m.id) == marker_id_to_index.end()) {
        marker_id_to_index[m.id] = marker_index_to_id.size();
        marker_index_to_id.emplace_back(m.id);
      }
    }
  }

  for (auto &id : marker_index_to_id) {
    ROS_INFO_STREAM("marker id " << id);
  }

  std::vector<Eigen::Vector3d> marker_positions(marker_index_to_id.size(),
                                                Eigen::Vector3d::Zero());

  size_t total_observation_count = 0;
  for (auto &view : views) {
    for (auto &m : view.markers) {
      total_observation_count++;
    }
  }

  size_t max_iterations = 64;

  double last_error = 10000;

  // while (ros::ok()) {
  for (size_t iteration = 0; iteration < max_iterations; iteration++) {

    size_t pose_columns = 6;

    size_t cols =
        view_poses.size() * pose_columns + marker_positions.size() * 3;
    size_t rows = total_observation_count * 3 + cols;

    size_t first_regularization_row = 0;
    size_t first_observation_row = cols;

    size_t first_view_col = 0;
    size_t first_marker_col = view_poses.size() * pose_columns;

    Eigen::VectorXd residuals = Eigen::VectorXd::Zero(rows);
    std::vector<Eigen::Triplet<double>> gradients;

    // regularization
    /*for (size_t i = 0; i < cols; i++) {
      residuals[first_regularization_row + i] = 0;
      gradients.emplace_back(first_regularization_row + i, i, 1.0);
    }*/

    // observations
    {
      size_t row = first_observation_row;

      for (size_t view_index = 0; view_index < views.size(); view_index++) {

        auto &view = views[view_index];
        auto &view_pose = view_poses[view_index];

        for (auto &m : view.markers) {

          size_t marker_index = marker_id_to_index[m.id];
          Eigen::Vector3d observed_position = view_pose * m.position;
          Eigen::Vector3d current_position = marker_positions[marker_index];

          residuals[row + 0] = observed_position.x() - current_position.x();
          residuals[row + 1] = observed_position.y() - current_position.y();
          residuals[row + 2] = observed_position.z() - current_position.z();

          gradients.emplace_back(row + 0,
                                 first_marker_col + marker_index * 3 + 0, 1.0);
          gradients.emplace_back(row + 1,
                                 first_marker_col + marker_index * 3 + 1, 1.0);
          gradients.emplace_back(row + 2,
                                 first_marker_col + marker_index * 3 + 2, 1.0);

          gradients.emplace_back(
              row + 0, first_view_col + view_index * pose_columns + 0, -1.0);
          gradients.emplace_back(
              row + 1, first_view_col + view_index * pose_columns + 1, -1.0);
          gradients.emplace_back(
              row + 2, first_view_col + view_index * pose_columns + 2, -1.0);

          Eigen::Matrix3d rx, ry, rz;
          rx << 0, 0, 0, 0, 0, -1, 0, 1, 0;
          ry << 0, 0, 1, 0, 0, 0, -1, 0, 0;
          rz << 0, -1, 0, 1, 0, 0, 0, 0, 0;

          Eigen::Vector3d tx =
              rx * -(observed_position - view_pose.translation());
          Eigen::Vector3d ty =
              ry * -(observed_position - view_pose.translation());
          Eigen::Vector3d tz =
              rz * -(observed_position - view_pose.translation());

          gradients.emplace_back(
              row + 0, first_view_col + view_index * pose_columns + 3, tx.x());
          gradients.emplace_back(
              row + 1, first_view_col + view_index * pose_columns + 3, tx.y());
          gradients.emplace_back(
              row + 2, first_view_col + view_index * pose_columns + 3, tx.z());

          gradients.emplace_back(
              row + 0, first_view_col + view_index * pose_columns + 4, ty.x());
          gradients.emplace_back(
              row + 1, first_view_col + view_index * pose_columns + 4, ty.y());
          gradients.emplace_back(
              row + 2, first_view_col + view_index * pose_columns + 4, ty.z());

          gradients.emplace_back(
              row + 0, first_view_col + view_index * pose_columns + 5, tz.x());
          gradients.emplace_back(
              row + 1, first_view_col + view_index * pose_columns + 5, tz.y());
          gradients.emplace_back(
              row + 2, first_view_col + view_index * pose_columns + 5, tz.z());

          row += 3;
        }
      }
    }

    // ROS_INFO_STREAM("residuals");
    // ROS_INFO_STREAM(residuals);

    Eigen::SparseMatrix<double> matrix(rows, cols);
    matrix.setFromTriplets(gradients.begin(), gradients.end());
    matrix.makeCompressed();

    Eigen::VectorXd result = Eigen::VectorXd::Zero(cols);

    Eigen::LeastSquaresConjugateGradient<Eigen::SparseMatrix<double>> solver;
    solver.compute(matrix);
    result = solver.solve(residuals);

    // ROS_INFO_STREAM("results");
    // ROS_INFO_STREAM(result);

    for (size_t view_index = 0; view_index < views.size(); view_index++) {

      view_poses[view_index].translation().x() +=
          result[first_view_col + view_index * pose_columns + 0];
      view_poses[view_index].translation().y() +=
          result[first_view_col + view_index * pose_columns + 1];
      view_poses[view_index].translation().z() +=
          result[first_view_col + view_index * pose_columns + 2];

      Eigen::Vector3d rvec(
          result[first_view_col + view_index * pose_columns + 3],
          result[first_view_col + view_index * pose_columns + 4],
          result[first_view_col + view_index * pose_columns + 5]);
      Eigen::Matrix3d l = view_poses[view_index].linear();
      if (rvec.squaredNorm() > 0.0) {
        l = Eigen::AngleAxisd(rvec.norm(), rvec.normalized()).matrix() * l;
      }
      view_poses[view_index].linear() = l;
    }

    for (size_t marker_index = 0; marker_index < marker_index_to_id.size();
         marker_index++) {

      marker_positions[marker_index].x() +=
          result[first_marker_col + marker_index * 3 + 0];
      marker_positions[marker_index].y() +=
          result[first_marker_col + marker_index * 3 + 1];
      marker_positions[marker_index].z() +=
          result[first_marker_col + marker_index * 3 + 2];
    }

    // getchar();

    std::vector<Marker> object_markers(marker_positions.size());
    for (size_t marker_index = 0; marker_index < marker_index_to_id.size();
         marker_index++) {
      object_markers[marker_index].position = marker_positions[marker_index];
      object_markers[marker_index].id = marker_index_to_id[marker_index];
    }
    publish_markers(object_markers);

    double error = 0.0;
    for (size_t i = 0; i < residuals.size(); i++) {
      error += std::abs(residuals[i]);
    }
    error /= residuals.size();

    ROS_INFO_STREAM("iteration: " << iteration << ", error: " << error);

    if (std::abs(last_error - error) <= 0.0000001) {
      ROS_INFO_STREAM("convergence");
      break;
    }

    last_error = error;

    // ros::Duration(1.0).sleep();
  }

  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for (auto &p : marker_positions) {
    center += p;
  }
  center /= marker_positions.size();
  for (auto &p : marker_positions) {
    p -= center;
  }

  YAML::Node yaml;
  for (size_t marker_index = 0; marker_index < marker_index_to_id.size();
       marker_index++) {
    YAML::Node yaml_marker;
    yaml_marker["id"] = marker_index_to_id[marker_index];
    yaml_marker["position"]["x"] = marker_positions[marker_index].x();
    yaml_marker["position"]["y"] = marker_positions[marker_index].y();
    yaml_marker["position"]["z"] = marker_positions[marker_index].z();
    yaml["markers"].push_back(yaml_marker);
  }

  ROS_INFO_STREAM("marker config\n" << yaml);
}
