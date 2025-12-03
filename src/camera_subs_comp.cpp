#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

class CompressedSubscriber : public rclcpp::Node {
public:
  CompressedSubscriber() : Node("compressed_subscriber") {
    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("/video_source/compressed", 10, std::bind(&CompressedSubscriber::callback, this, std::placeholders::_1));
    frame_count_ = 0;
    last_fps_time_ = this->get_clock()->now();
  }

private:
  void callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    // Crear un cv::Mat envolviendo los datos comprimidos
    cv::Mat raw_data(1, msg->data.size(), CV_8UC1, const_cast<unsigned char*>(msg->data.data()));

    // Elegir modo de lectura según formato
    int imread_flag = cv::IMREAD_COLOR;
    // Si quisieras usar msg->format:
    // if (msg->format == "mono8") imread_flag = cv::IMREAD_GRAYSCALE;

    cv::Mat image = cv::imdecode(raw_data, imread_flag);

    if (image.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to decode compressed image");
      return;
    }

    update_fps();
    cv::namedWindow("View", cv::WINDOW_AUTOSIZE);
    std::string title = "View - FPS: " + std::to_string((fps_));
    cv::setWindowTitle("View", title);
    cv::imshow("View", image);
    cv::waitKey(1);
  }

  void update_fps() {
    frame_count_++;

    auto now = this->get_clock()->now();
    double elapsed = (now - last_fps_time_).seconds();
    //RCLCPP_INFO(this->get_logger(), "elapsed: %.6f", elapsed);

    // Recalcular FPS cada ~1 segundo
    if (elapsed >= 1.0) {
      fps_ = frame_count_ / elapsed;
      frame_count_ = 0;
      last_fps_time_ = now;
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
  rclcpp::Time last_fps_time_;
  int frame_count_;
  double fps_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CompressedSubscriber>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
