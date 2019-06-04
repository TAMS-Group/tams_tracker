// 2018, Philipp Ruppel, ruppel@informatik.uni-hamburg.de

#pragma once

#include <ros/ros.h>

#include <eigen3/Eigen/StdVector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>

#include <thread>

#include <array>
#include <assert.h>
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <random>
#include <unordered_set>
#include <vector>

#include <execinfo.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <boost/functional/hash.hpp>

#include <opencv2/opencv.hpp>

#include <opencv2/bgsegm.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/optflow.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <yaml-cpp/yaml.h>

#include <std_msgs/String.h>

#include <actionlib/server/simple_action_server.h>

#include <dirent.h>

#include <sys/stat.h>
#include <sys/types.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <eigen_conversions/eigen_msg.h>

#include <visualization_msgs/MarkerArray.h>

#include <opencv2/core/eigen.hpp>

#include <Eigen/Sparse>

#include <sensor_msgs/Imu.h>

namespace tams_tracker {

double squared(double d) { return d * d; }

} // namespace tams_tracker
