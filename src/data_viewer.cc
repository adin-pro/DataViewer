#include "data_viewer.hpp"
#include "basic_defs.hpp"
#include <thread>
#include <vector>

std::mutex DataViewer::global_optimizer_mutex_;

DataViewer::DataViewer() {}

DataViewer::~DataViewer() {}

DataViewer &DataViewer::getInstance() {
  static DataViewer *instance = nullptr;
  if (!instance) {
    global_optimizer_mutex_.lock();
    if (!instance) {
      instance = new DataViewer;
    }
    global_optimizer_mutex_.unlock();
  }
  return *instance;
}

void DataViewer::init(std::string config) {
  text_font_ = std::make_shared<pangolin::GlFont>(config, 20);
}

void DataViewer::run() {
  // init pangolin window
  std::cout << kColorBlue << "Adin Viewer starts to run" << kColorReset
            << std::endl;
  pangolin::CreateWindowAndBind("Data Viewer", 1624, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // const numbers
  // horizontal boarder
  const int panel_width = 300;
  const int cam_width = 1324;
  // viewpoint
  double viewdpoint_focal_length = 2000.0;
  double view_point_x = 0.0;
  double view_point_y = 0.0;
  double view_point_z = 200.0;

  // setup imu ploters
  std::vector<std::string> imu_gyro_labels{"gyro_x", "gyro_y", "gyro_z"};
  std::vector<std::string> imu_acc_labels{"acc_x", "acc_y", "acc_z"};
  // setup log labels
  imu_acc_log_.SetLabels(imu_acc_labels);
  imu_gyro_log_.SetLabels(imu_gyro_labels);
  imu_orientation_log_.SetLabels(std::vector<std::string>{"pitch"});

  // shape of plotter: left right bottom top xtick ytick
  pangolin::Plotter gyro_plotter(&imu_gyro_log_, 0.0f, 300.0f, -15.0f, 15.0f,
                                 100.0f, 2.0f);
  pangolin::Plotter acc_plotter(&imu_acc_log_, 0.0f, 300.0f, -20.0f, 20.0f,
                                100.0f, 2.0f);
  pangolin::Plotter imu_orientation_plotter(&imu_orientation_log_, 0.0f, 300.0f,
                                            -20.0f, 20.0f, 100.0f, 2.0f);
  // position of plotter
  acc_plotter.SetBounds(0.66, 1.0, pangolin::Attach::Pix(cam_width), 1.0);
  acc_plotter.Track("$i");

  gyro_plotter.SetBounds(0.33, 0.66, pangolin::Attach::Pix(cam_width), 1.0);
  gyro_plotter.Track("$i");

  imu_orientation_plotter.SetBounds(0, 0.33, pangolin::Attach::Pix(cam_width),
                                    1.0);
  imu_orientation_plotter.Track("$i");

  pangolin::DisplayBase().AddDisplay(gyro_plotter);
  pangolin::DisplayBase().AddDisplay(acc_plotter);
  pangolin::DisplayBase().AddDisplay(imu_orientation_plotter);

  // create panel
  pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0,
                                        pangolin::Attach::Pix(panel_width));

  pangolin::Var<bool> menu_draw_trajectory("ui.DrawTrjectory", true, true);
  pangolin::Var<bool> menu_draw_moving_node("ui.DrawMovingNode", true, true);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, viewdpoint_focal_length,
                                 viewdpoint_focal_length, 512, 389, 0.1, 2000),
      pangolin::ModelViewLookAt(view_point_x + 1, view_point_y, view_point_z,
                                view_point_x, view_point_y, 0, 1.0, 0.0, 0.0));

  pangolin::View &d_cam =
      pangolin::CreateDisplay()
          .SetBounds(0.0, 1.0, pangolin::Attach::Pix(panel_width),
                     pangolin::Attach::Pix(cam_width), -1024.0 / 768.0)
          .SetHandler(new pangolin::Handler3D(s_cam));

  // pangolin::CreateDisplay()
  //     .SetBounds(0.0, 0.0, pangolin::Attach::Pix(panel_width),
  //                pangolin::Attach::Pix(cam_width))
  //     .SetLayout(pangolin::LayoutEqual);

  // init s_cam
  pangolin::OpenGlMatrix twc;
  twc.SetIdentity();
  s_cam.Follow(twc);

  // main loop
  while (true) {
    // TODO
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.18, 0.18, 0.18, 1.0f);
    d_cam.Activate(s_cam);

    pangolin::glDrawColouredCube();

    drawAxis();

    drawGrid();

    if (menu_draw_trajectory) {
      drawTrajectory();
    }

    if (menu_draw_moving_node) {
      drawMovingNode();
    }

    drawIMU();
    // sleep for 100ms
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    pangolin::FinishFrame();
  }
}

void DataViewer::drawIMU() {
  std::unique_lock<std::mutex> lock(mutex_data_);
  imu_gyro_log_.Log(imu_gyro_.x() * 180 / M_PI, imu_gyro_.y() * 180 / M_PI,
                    imu_gyro_.z() * 180 / M_PI);
  imu_acc_log_.Log(imu_acc_.x(), imu_acc_.y(), imu_acc_.z());
  imu_orientation_log_.Log(pitch_);
}

void DataViewer::setIMU(Eigen::Vector3d imu_acc, Eigen::Vector3d imu_gyro,
                        double pitch) {
  std::unique_lock<std::mutex> lock(mutex_data_);
  imu_acc_ = imu_acc;
  imu_gyro_ = imu_gyro;
  pitch_ = pitch;
}

void DataViewer::drawAxis() {
  glColor3f(1, 0, 0);
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(10, 0, 0);
  glColor3f(0, 1, 0);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 10, 0);
  glColor3f(0, 0, 1);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 0, 10);
  glEnd();
}

void DataViewer::drawGrid() {
  glLineWidth(1.0);
  double x = 0.0;
  double y = 0.0;
  glBegin(GL_LINES);
  glColor3f(0.443, 0.443, 0.443);
  for (float i = -400; i <= 400; i = i + 20) {
    glVertex3d(400 + x, i + y, 0);
    glVertex3d(-400 + x, i + y, 0);
    glVertex3d(i + x, 400 + y, 0);
    glVertex3d(i + x, -400 + y, 0);
  }
  glEnd();
}

void DataViewer::drawNode(Eigen::Vector3d pos, Eigen::Vector3d color,
                          double size) {
  glBegin(GL_POLYGON);
  glColor3d(color.x(), color.y(), color.z());
  const int segments = 20;
  for (int i = 0; i < segments; i++) {
    double theta = i * 360.0 / segments / 180.0 * 3.1415926;
    glVertex3d(pos.x() + size * cos(theta), pos.y() + size * sin(theta), 0.0);
  }
  glEnd();
}

void DataViewer::drawMovingNode() {
  Eigen::Vector3d node_pose;
  {
    std::unique_lock<std::mutex>(mutex_data_);
    node_pose = node_;
  }
  drawNode(node_pose, Eigen::Vector3d(0.7, 0.3, 0.), 1.0);
}

void DataViewer::drawTrajectory() {
  std::deque<GNSS> gnss_data;
  {
    std::unique_lock<std::mutex> lock(mutex_gnss_data_);
    gnss_data = gnss_deque_;
  }
  glLineWidth(6);
  glBegin(GL_LINE_STRIP);
  glColor3d(1, 1, 0);
  for (auto& frame : gnss_data) {
    glVertex3d(frame.lon, frame.lat, 0);
  }
  glEnd();

}


void DataViewer::setMovingNode(Eigen::Vector3d pose) { node_ = pose; }

void DataViewer::addGNSSFrame(GNSS gnss_frame) {
  std::unique_lock<std::mutex> lock(mutex_gnss_data_);
  gnss_deque_.push_back(gnss_frame);
  if (gnss_deque_.size() > 60) {
    gnss_deque_.pop_front();
  }
}