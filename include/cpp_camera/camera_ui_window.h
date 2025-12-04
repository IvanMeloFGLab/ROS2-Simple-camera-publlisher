#include <QWidget>

#include <math.h>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using std::string;
using std::vector;
using std::shared_ptr;
using std::make_shared;
using std::to_string;

namespace cpp_puzzlesim {

class PuzzlesimWindow : public QWidget {

public:
  PuzzlesimWindow(rclcpp::Node::SharedPtr& node_handle, QWidget* parent = 0, Qt::WindowFlags f = Qt::WindowFlags());
  ~PuzzlesimWindow();

private slots:
  void onUpdate();

private:
  void paramEventCalb(const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr event);

  rclcpp::Node::SharedPtr pns_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_event_sub_;

  int width_, heigh_;
};

}
