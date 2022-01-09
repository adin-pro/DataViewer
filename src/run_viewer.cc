#include "data_viewer.hpp"
#include "basic_defs.hpp"

#include <iostream>
#include <memory>
#include <signal.h>
#include <thread>
#include <unistd.h>

#include <Eigen/Dense>
#include <random>

void exit_logger(int s) {
  std::cout << kColorPurple << "\ncatch ctrl^C, system will exit" << kColorReset
            << std::endl;
  _exit(1);
}

int main(int, char **) {
  std::cout << kColorYellow << "initing..." << kColorReset << std::endl;
  signal(SIGINT, exit_logger);
  std::shared_ptr<std::thread> viewer_thread = std::make_shared<std::thread>(
      &DataViewer::run, &DataViewer::getInstance());
  viewer_thread->detach();
  int tick = 0;

  while (true) {
    tick++;

    Eigen::Vector3d imu_acc((rand() % 1000) * 0.01, (rand() % 1000) * 0.01,
                            (rand() % 1000) * 0.01);
    Eigen::Vector3d imu_gyro((rand() % 1000) * 0.001, (rand() % 1000) * 0.001,
                            (rand() % 1000) * 0.001);
    double pitch = (rand() % 1000) * 0.01;
    DataViewer::getInstance().setIMU(imu_acc, imu_gyro, pitch);
    double theta = (tick % 360) / 180.0 * 3.1415926;
    DataViewer::getInstance().setMovingNode(Eigen::Vector3d(30*cos(theta), 30*sin(theta), 0));
    GNSS gnss_frame;
    gnss_frame.lon = 30*cos(theta);
    gnss_frame.lat = 30*sin(theta);
    DataViewer::getInstance().addGNSSFrame(gnss_frame);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  std::cout << kColorYellow << "main thread die" << kColorReset << std::endl;
  return 0;
}