#ifndef ADIN_VIEWER
#define ADIN_VIEWER

#include <iostream>
#include <memory>
#include <mutex>
#include <pangolin/pangolin.h>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <queue>

struct GNSS {
  long timestamp;
  double lon;
  double lat;
  double alt;
};

struct Odom {
  long timestamp;
  Eigen::Vector3d pos;
  Eigen::Quaterniond quat;
};


class DataViewer {
public:
  ~DataViewer();

  void run();

  void init(const std::string config);

  static DataViewer &getInstance();

  void setIMU(Eigen::Vector3d imu_acc, Eigen::Vector3d imu_gyro, double pitch);

  void setMovingNode(Eigen::Vector3d pos);

  void addGNSSFrame(GNSS gnss_frame);

private:
  DataViewer();

  void drawAxis();

  void drawGrid();

  void drawIMU();

  void drawNode(Eigen::Vector3d pos, Eigen::Vector3d color, double size);

  void drawTrajectory();

  void drawMovingNode();

private:
  static std::mutex global_optimizer_mutex_;
  std::mutex mutex_data_;
  std::mutex mutex_gnss_data_;

  std::shared_ptr<pangolin::GlFont> text_font_;

  pangolin::DataLog imu_acc_log_;
  pangolin::DataLog imu_gyro_log_;
  pangolin::DataLog imu_orientation_log_;

  Eigen::Vector3d imu_acc_;
  Eigen::Vector3d imu_gyro_;

  // pose deque
  std::deque<GNSS> gnss_deque_;
  std::deque<Odom> odometry_gnss_deque_;

  // moving node
  Eigen::Vector3d node_;

  double pitch_;
};

#endif // ADIN_VIEWER