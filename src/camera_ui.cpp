#include <QApplication>

#include <rclcpp/rclcpp.hpp>
#include "cpp_camera/camera_ui_window.h"

class mApp : public QApplication {
public:
  rclcpp::Node::SharedPtr psn_;

  explicit mApp(int& argc, char** argv) : QApplication(argc, argv) {
    rclcpp::init(argc, argv);
    psn_ = rclcpp::Node::make_shared("camera_ui");
  }

  ~mApp() {
    rclcpp::shutdown();
  }

  int exec() {

    cpp_camera::CameraWindow CameraWin(psn_);
    CameraWin.show();

    return QApplication::exec();
  }
};

int main(int argc, char** argv) {
  mApp app(argc, argv);
  return app.exec();
}
