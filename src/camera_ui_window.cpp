#include "cpp_puzzlesim/puzzlesim_window.h"

namespace cpp_camera {

PuzzlesimWindow::PuzzlesimWindow(rclcpp::Node::SharedPtr& node_handle, QWidget* parent, Qt::WindowFlags f) {
  pns_ = node_handle;

  RCLCPP_INFO(pns_->get_logger(), "Starting Camera Ui with node name %s", pns_->get_fully_qualified_name());
}

PuzzlesimWindow::~PuzzlesimWindow() {
}

void PuzzlesimWindow::paramEventCalb(const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr event) {

}

void PuzzlesimWindow::onUpdate() {
  if (!rclcpp::ok()) {
    close();
    return;
  }

  rclcpp::spin_some(pns_);

  update();
}

void PuzzlesimWindow::paintEvent(QPaintEvent *) {

}

}
