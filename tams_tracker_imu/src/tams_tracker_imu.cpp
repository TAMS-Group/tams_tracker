#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <termios.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "tams_tracker_imu");

  ros::NodeHandle node("~");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::Publisher publisher =
      node.advertise<sensor_msgs::Imu>("data_raw", 10, true);

  tf::TransformBroadcaster transform_broadcaster;

  if (argc < 2) {
    ROS_ERROR_STREAM(
        "usage: tams_tracker_imu <serial|tcp> [<path|address> [<port>]]");
    return -1;
  }

  std::string base_frame = "tams_tracker";
  node.param("base_frame", base_frame, base_frame);

  std::string imu_frame = "tams_tracker_imu";
  node.param("imu_frame", imu_frame, imu_frame);

  std::string mode = argv[1];

  int device = -1;

  if (mode == "serial") {
    const char *device_name = "/dev/ttyUSB0";
    if (argc > 2) {
      device_name = argv[2];
    }
    device = open(device_name, O_RDWR);

    struct termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(device, &tty) != 0) {
      ROS_ERROR_STREAM("serial i/o error: failed to get attributes");
      return -1;
    }

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;
    tty.c_cc[VTIME] = 10;
    tty.c_cc[VMIN] = 0;

    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    if (tcsetattr(device, TCSANOW, &tty) != 0) {
      ROS_ERROR_STREAM("serial i/o error: failed to set attributes");
      return -1;
    }
  }

  if (mode == "tcp") {
    device = socket(AF_INET, SOCK_STREAM, 0);
    if (device < 0) {
      ROS_ERROR_STREAM("failed to open socket");
      return -1;
    }

    const char *address = "192.168.1.200";
    if (argc > 2) {
      address = argv[2];
    }
    auto *host = gethostbyname(address);
    if (!host) {
      ROS_INFO_STREAM("invalid address");
      return -1;
    }

    int port = 10000;
    if (argc > 3) {
      port = atoi(argv[3]);
    }

    struct sockaddr_in serv_addr;
    bzero((char *)&serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    bcopy((char *)host->h_addr, (char *)&serv_addr.sin_addr.s_addr,
          host->h_length);

    if (connect(device, (sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
      ROS_ERROR_STREAM("failed to connect to host");
      return false;
    }
  }

  if (device < 0) {
    ROS_ERROR_STREAM("failed to open device or invalid mode");
    return -1;
  }

  std::string line;

  bool imu_running = false;

  double acceleration_scaling = 2.0 * 9.81 / (1 << 15);
  double rotation_scaling = 250.0 * (M_PI / 180) / (1 << 15);

  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

  ros::Time last_time = ros::Time::now();

  ros::Time start_time = ros::Time::now();

  Eigen::Vector3d acceleration_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d rotation_bias = Eigen::Vector3d::Zero();
  double bias_counter = 0;

  auto process_line = [&]() {
    if (line.size() < 5) {
      ROS_ERROR_STREAM_THROTTLE(0.1, "message too short");
      return;
    }
    if (memcmp(line.data(), "imu ", 4) != 0) {
      ROS_ERROR_STREAM_THROTTLE(0.1, "invalid message");
      return;
    }
    int16_t ax, ay, az, rx, ry, rz, checksum;
    int rs = sscanf(line.data() + 4, "%hi %hi %hi %hi %hi %hi %hi", &ax, &ay,
                    &az, &rx, &ry, &rz, &checksum);
    if (rs != 7) {
      ROS_ERROR_STREAM_THROTTLE(0.1, "format error, parsed tokens: " << rs);
      return;
    }
    if ((int16_t)(ax + ay + az + rx + ry + rz) != checksum) {
      ROS_ERROR_STREAM_THROTTLE(0.1, "checksum error");
      return;
    }
    // ROS_INFO_STREAM(ax << " " << ay << " " << az << " " << rx << " " << ry
    //                   << " " << rz);
    if (!imu_running) {
      imu_running = true;
      ROS_INFO("imu running");
    }

    ros::Time current_time = ros::Time::now();

    Eigen::Vector3d acceleration;
    acceleration.x() = ax * acceleration_scaling;
    acceleration.y() = ay * acceleration_scaling;
    acceleration.z() = az * acceleration_scaling;

    Eigen::Vector3d rotation;
    rotation.x() = rx * rotation_scaling;
    rotation.y() = ry * rotation_scaling;
    rotation.z() = rz * rotation_scaling;

    if ((current_time - start_time).toSec() < 5.0 || bias_counter < 100) {
      acceleration_bias += acceleration;
      rotation_bias += rotation;
      bias_counter++;
      return;
    }

    // acceleration -= acceleration_bias / bias_counter;
    rotation -= rotation_bias / bias_counter;

    Eigen::Vector3d rotation_vector =
        rotation * (current_time - last_time).toSec();
    Eigen::Quaterniond rotation_quaternion = Eigen::Quaterniond::Identity();
    if (rotation_vector.squaredNorm() > 0) {
      rotation_quaternion = Eigen::AngleAxisd(rotation_vector.norm(),
                                              rotation_vector.normalized());
    }
    orientation = orientation * rotation_quaternion;
    orientation = orientation.normalized();

    sensor_msgs::Imu msg;
    msg.header.stamp = current_time;
    msg.header.frame_id = base_frame;
    msg.linear_acceleration.x = acceleration.x();
    msg.linear_acceleration.y = acceleration.y();
    msg.linear_acceleration.z = acceleration.z();
    msg.angular_velocity.x = rotation.x();
    msg.angular_velocity.y = rotation.y();
    msg.angular_velocity.z = rotation.z();
    tf::quaternionEigenToMsg(orientation, msg.orientation);
    publisher.publish(msg);

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = orientation.matrix();

    tf::StampedTransform transform;
    tf::poseEigenToTF(pose, transform);
    transform.frame_id_ = base_frame;
    transform.child_frame_id_ = imu_frame;
    transform.stamp_ = current_time;
    transform_broadcaster.sendTransform(transform);

    last_time = current_time;
  };

  while (ros::ok()) {

    char chr;
    if (!read(device, &chr, 1)) {
      continue;
    }
    if (chr == '\n') {
      process_line();
      line.clear();
    } else {
      line.push_back(chr);
    }
  }
}
