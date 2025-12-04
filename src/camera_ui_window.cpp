#include "cpp_camera/camera_ui_window.h"

namespace cpp_camera {

CameraWindow::CameraWindow(rclcpp::Node::SharedPtr& node_handle, QWidget* parent, Qt::WindowFlags f) {
  pns_ = node_handle;

  RCLCPP_INFO(pns_->get_logger(), "Starting Camera Ui with node name %s", pns_->get_fully_qualified_name());
}

CameraWindow::~CameraWindow() {
}

void CameraWindow::paramEventCalb(const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr event) {

}

void CameraWindow::onUpdate() {
  if (!rclcpp::ok()) {
    close();
    return;
  }

  rclcpp::spin_some(pns_);
}

void CameraWindow::paintEvent(QPaintEvent *) {

}

}
