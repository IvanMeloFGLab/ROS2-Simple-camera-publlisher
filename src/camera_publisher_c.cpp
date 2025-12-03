#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <thread>
#include <chrono>
#include <string>
#include <cstring>
#include <cerrno>
#include <vector>
#include <map>

class ProcessGuard {
public:
  ProcessGuard(const rclcpp::Logger& logger) : logger_(logger) {}

  ~ProcessGuard() { stop(); }

  bool start(const std::string &cmd) {
    if (running()) return true;
    cmd_ = cmd;

    pid_ = fork();
    if (pid_ == -1) {
      RCLCPP_ERROR(logger_, "Error: '%s'", strerror(errno));
      pid_ = -1;
      return false;
    }

    if (pid_ == 0) {
      setpgid(0, 0);
      execl("/bin/bash", "bash", "-lc", cmd_.c_str(), (char*)nullptr);
      _exit(127); // exec failed
    }

    // parent
    setpgid(pid_, pid_);
    return true;
  }

  void stop(int timeout_ms = 4000) {
    if (!running()) return;

    // SIGTERM to the whole process group (negative PID)
    kill(-pid_, SIGTERM);

    auto t0 = std::chrono::steady_clock::now();
    int status = 0;
    while (true) {
      pid_t r = waitpid(pid_, &status, WNOHANG);
      if (r == pid_) break;
      if (r == -1 && errno == ECHILD) break;

      if (std::chrono::steady_clock::now() - t0 > std::chrono::milliseconds(timeout_ms)) {
        kill(-pid_, SIGKILL);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    pid_ = -1;
  }

  bool restart(const std::string &cmd) {
    if (running()) stop();
    return start(cmd);
  }

  bool running() const { return pid_ > 0; }

private:
  pid_t pid_{-1};
  std::string cmd_;
  const rclcpp::Logger& logger_;
};

class CameraPublisher : public rclcpp::Node {
public:
  CameraPublisher()
  : Node("camera_publisher"), proc_(this->get_logger()) {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/video_source/raw", 10);
    com_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/video_source/compressed", 10);

    this->declare_parameter<std::string>("board", "PC");
    this->declare_parameter<bool>("Compression", false);
    this->declare_parameter<bool>("preview", false);

    rcl_interfaces::msg::ParameterDescriptor desc;
    if (this->get_parameter("board").as_string() != "rasp") {
      desc.description = "0:640x480/30,1:640x480/60,2:1280x720/30,3:1280x720/60,4:1640x1232/30,5:1920x1080/30,6:3280x1848/28,7:3264x2464/21";
      desc.additional_constraints = "0:640x480/30,1:640x480/60,2:1280x720/30,3:1280x720/60,4:1640x1232/30,5:1920x1080/30,6:3280x1848/28,7:3264x2464/21";
    } else {
      desc.description = "0:640x480/30,1:640x480/60,2:640x480/90,3:640x480/103,4:1640x1232/30,5:1640x1232/41,6:1920x1080/30,7:1920x1080/47";
      desc.additional_constraints = "0:640x480/30,1:640x480/60,2:640x480/90,3:640x480/103,4:1640x1232/30,5:1640x1232/41,6:1920x1080/30,7:1920x1080/47";
    }
    rcl_interfaces::msg::IntegerRange rng;
    rng.set__from_value(0).set__to_value(7).set__step(1);
    desc.integer_range.push_back(rng);
    this->declare_parameter<int>("mode", 0, desc);

    rcl_interfaces::msg::ParameterDescriptor desc2;
    desc2.description = "0:JPEG, 1:PNG, 2:WEBP, 3:AVIF, 4:JPEGXL";
    desc2.additional_constraints = "0:JPEG, 1:PNG, 2:WEBP, 3:AVIF, 4:JPEGXL";
    rcl_interfaces::msg::IntegerRange rng2;
    rng2.set__from_value(0).set__to_value(2).set__step(1);
    desc2.integer_range.push_back(rng2);
    this->declare_parameter<int>("format", 0, desc2);

    this->declare_parameter<int>("VCam_num", 100);
    this->declare_parameter<bool>("Hflip", false);
    this->declare_parameter<bool>("Vflip", true);

    rcl_interfaces::msg::ParameterDescriptor desc3;
    desc3.description = "JPEG_quality";
    rcl_interfaces::msg::IntegerRange rng3;
    rng3.set__from_value(0).set__to_value(100).set__step(1);
    desc3.integer_range.push_back(rng3);
    this->declare_parameter<int>("JPEG_quality", 90, desc3);

    rcl_interfaces::msg::ParameterDescriptor desc4;
    desc4.description = "PNG_quality";
    rcl_interfaces::msg::IntegerRange rng4;
    rng4.set__from_value(0).set__to_value(9).set__step(1);
    desc4.integer_range.push_back(rng4);
    this->declare_parameter<int>("PNG_quality", 3, desc4);

    rcl_interfaces::msg::ParameterDescriptor desc5;
    desc5.description = "WEBP_quality";
    rcl_interfaces::msg::IntegerRange rng5;
    rng5.set__from_value(0).set__to_value(101).set__step(1);
    desc5.integer_range.push_back(rng5);
    this->declare_parameter<int>("WEBP_quality", 90, desc5);

    /*rcl_interfaces::msg::ParameterDescriptor desc6;
    desc6.description = "AVIF_quality";
    rcl_interfaces::msg::IntegerRange rng6;
    rng6.set__from_value(0).set__to_value(100).set__step(1);
    desc6.integer_range.push_back(rng6);
    this->declare_parameter<int>("AVIF_quality", 95, desc6);

    rcl_interfaces::msg::ParameterDescriptor desc7;
    desc7.description = "AVIF_speed";
    rcl_interfaces::msg::IntegerRange rng7;
    rng7.set__from_value(0).set__to_value(10).set__step(1);
    desc7.integer_range.push_back(rng7);
    this->declare_parameter<int>("AVIF_speed", 9, desc7);

    rcl_interfaces::msg::ParameterDescriptor desc8;
    desc8.description = "JPEGXL_quality";
    rcl_interfaces::msg::IntegerRange rng8;
    rng8.set__from_value(0).set__to_value(100).set__step(1);
    desc8.integer_range.push_back(rng8);
    this->declare_parameter<int>("JPEGXL_quality", 90, desc8);

    rcl_interfaces::msg::ParameterDescriptor desc9;
    desc9.description = "JPEGXL_speed";
    rcl_interfaces::msg::IntegerRange rng9;
    rng9.set__from_value(1).set__to_value(9).set__step(1);
    desc9.integer_range.push_back(rng9);
    this->declare_parameter<int>("JPEGXL_speed", 5, desc9);*/

    res_map_[0] = {"640", "480", "30"};
    res_map_[1] = {"640", "480", "60"};
    res_map_[2] = {"640", "480", "90"};
    res_map_[3] = {"640", "480", "103"};
    res_map_[4] = {"1640", "1232", "30"};
    res_map_[5] = {"1640", "1232", "41"};
    res_map_[6] = {"1920", "1080", "30"};
    res_map_[7] = {"1920", "1080", "47"};

    res_map2_[0] = {"640", "480", "30"};
    res_map2_[1] = {"640", "480", "60"};
    res_map2_[2] = {"1280", "720", "30"};
    res_map2_[3] = {"1280", "720", "60"};
    res_map2_[4] = {"1640", "1232", "30"};
    res_map2_[5] = {"1920", "1080", "30"};
    res_map2_[6] = {"3280", "1848", "28"};
    res_map2_[7] = {"3264", "2464", "21"};

    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&CameraPublisher::parameters_callback, this, std::placeholders::_1));
  }

  ~CameraPublisher() {
    proc_.stop();
  }

  bool init() {
    std::string board = this->get_parameter("board").as_string();
    compression_ = this->get_parameter("Compression").as_bool();
    std::string preview = this->get_parameter("preview").as_bool() ? "" : "-n";
    std::string vcam = std::to_string(this->get_parameter("VCam_num").as_int());
    auto res = res_map_[this->get_parameter("mode").as_int()];
    std::string hflip = this->get_parameter("Hflip").as_bool() ? "--hflip" : "";
    std::string vflip = this->get_parameter("Vflip").as_bool() ? "--vflip" : "";
    com_format_ = this->get_parameter("format").as_int();
    jpegq_ = this->get_parameter("JPEG_quality").as_int();
    pngq_ = this->get_parameter("PNG_quality").as_int();
    webpq_ = this->get_parameter("WEBP_quality").as_int();
    /*avifq_ = this->get_parameter("AVIF_quality").as_int();
    avifs_ = this->get_parameter("AVIF_speed").as_int();
    jpegxlq_ = this->get_parameter("JPEGXL_quality").as_int();
    jpegxls_ = this->get_parameter("JPEGXL_speed").as_int();*/

    if (board != "rasp" && board != "jnano" && board != "jorin" && board != "PC") {
      RCLCPP_ERROR(this->get_logger(), "La tarjeta no es compatible.");
      return false;
    };

    return reinit(preview, res[0], res[1], res[2], vcam, hflip, vflip, board);
  }

private:
  void timer_callback() {
    cv::Mat frame;
    std::vector<uchar> buf;
    if (!cap_.read(frame)) {
      RCLCPP_WARN(this->get_logger(), "No se pudo leer frame de la cámara.");
      return;
    }

    if (board_ != "rasp") {
      if (vflip_ == "--vflip") {
        cv::flip(frame, frame, 0);
      }
      if (hflip_ == "--hflip") {
        cv::flip(frame, frame, 1);
      }
      if (prev_ == "") {
        cv::imshow("Preview - FPS: "+d_fps_, frame);
        cv::waitKey(1);
      }
    }

    if (!compression_) {
      // Convertir cv::Mat -> sensor_msgs::msg::Image
      std_msgs::msg::Header header;
      header.stamp = this->get_clock()->now();
      header.frame_id = "camera_link";

      auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
      //std::size_t data_bytes = msg->data.size();
      //double data_mb = static_cast<double>(data_bytes) / (1024.0 * 1024.0);

      //RCLCPP_INFO(this->get_logger(), "Image msg: %ux%u, encoding=%s, step=%u, data=%zu bytes (%.3f MB)", msg->width, msg->height, msg->encoding.c_str(), msg->step, data_bytes, data_mb);
      publisher_->publish(*msg);
    } else {
      sensor_msgs::msg::CompressedImage cmsg;
      cmsg.header.stamp = this->get_clock()->now();
      cmsg.header.frame_id = "camera_link";

      switch (com_format_) {
        case 0: {
          cmsg.format = "jpg";
          std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpegq_};
          cv::imencode(".jpg", frame, buf, params);
          cmsg.data = std::move(buf);
          break;
        }
        case 1: {
          cmsg.format = "png";
          std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, pngq_};
          cv::imencode(".png", frame, buf, params);
          cmsg.data = std::move(buf);
          break;
        }
        case 2: {
          cmsg.format = "webp";
          std::vector<int> params = {cv::IMWRITE_WEBP_QUALITY, webpq_};
          cv::imencode(".webp", frame, buf, params);
          cmsg.data = std::move(buf);
          break;
        }
        /*case 3: {
          cmsg.format = "avif";
          std::vector<int> params = {cv::IMWRITE_AVIF_QUALITY, avifq_, cv::IMWRITE_AVIF_SPEED, avifs_};
          cv::imencode(".avif", frame, buf, params);
          cmsg.data = std::move(buf);
          break;
        }
        case 4: {
          cmsg.format = "jxl";
          std::vector<int> params = {cv::IMWRITE_JPEGXL_QUALITY, jpegxlq_, cv::IMWRITE_JPEGXL_EFFORT, jpegxls_};
          cv::imencode(".jxl", frame, buf, params);
          cmsg.data = std::move(buf);
          break;
        }*/
      }

      //auto data_bytes = cmsg.data.size();
      //auto data_mb = static_cast<double>(data_bytes) / (1024.0 * 1024.0);

      //RCLCPP_INFO(this->get_logger(), "CompressedImage msg: data=%zu bytes (%.3f MB)", data_bytes, data_mb);
      com_publisher_->publish(cmsg);
    }
  }

  rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> & parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    std::string board = this->get_parameter("board").as_string();
    compression_ = this->get_parameter("Compression").as_bool();
    std::string preview = this->get_parameter("preview").as_bool() ? "" : "-n";
    int mode = this->get_parameter("mode").as_int();
    std::string vcam = std::to_string(this->get_parameter("VCam_num").as_int());
    std::string hflip = this->get_parameter("Hflip").as_bool() ? "--hflip" : "";
    std::string vflip = this->get_parameter("Vflip").as_bool() ? "--vflip" : "";
    com_format_ = this->get_parameter("format").as_int();
    jpegq_ = this->get_parameter("JPEG_quality").as_int();
    pngq_ = this->get_parameter("PNG_quality").as_int();
    webpq_ = this->get_parameter("WEBP_quality").as_int();
    /*avifq_ = this->get_parameter("AVIF_quality").as_int();
    avifs_ = this->get_parameter("AVIF_speed").as_int();
    jpegxlq_ = this->get_parameter("JPEGXL_quality").as_int();
    jpegxls_ = this->get_parameter("JPEGXL_speed").as_int();*/

    for (const auto & param : parameters) {
      if (param.get_name() == "preview") {
        preview = param.as_bool() ? "" : "-n";
        RCLCPP_INFO(this->get_logger(), "preview changed.");
      } else if (param.get_name() == "mode") {
        mode = param.as_int();
        RCLCPP_INFO(this->get_logger(), "mode changed.");
      } else if (param.get_name() == "VCam_num") {
        vcam = std::to_string(param.as_int());
        RCLCPP_INFO(this->get_logger(), "VCam_num changed.");
      } else if (param.get_name() == "Hflip") {
        vflip = param.as_bool() ? "--hflip" : "";
        RCLCPP_INFO(this->get_logger(), "Image fliped horizontally.");
      } else if (param.get_name() == "Vflip") {
        vflip = param.as_bool() ? "--vflip" : "";
        RCLCPP_INFO(this->get_logger(), "Image fliped vertically.");
      } else if (param.get_name() == "board") {
        board = param.as_string();
        if (board != "rasp" && board != "jnano" && board != "jorin" && board != "PC") {
          RCLCPP_ERROR(this->get_logger(), "La tarjeta no es compatible.");
          result.successful = false;
          result.reason = "La tarjeta no es compatible.";
        }
        RCLCPP_ERROR(this->get_logger(), "Not available during runtime. :(");
        result.successful = false;
        result.reason = "Not available during runtime. :(";
      } else if (param.get_name() == "Compression") {
        compression_ = param.as_bool();
        RCLCPP_INFO(this->get_logger(), "Starting compression.");
      } else if (param.get_name() == "format") {
        com_format_ = param.as_int();
        RCLCPP_INFO(this->get_logger(), "Changed format successfully.");
      } else if (param.get_name() == "JPEG_quality") {
        jpegq_ = param.as_int();
        RCLCPP_INFO(this->get_logger(), "Se cambio la calidad de JPEG.");
      } else if (param.get_name() == "PNG_quality") {
        pngq_ = param.as_int();
        RCLCPP_INFO(this->get_logger(), "Se cambio la calidad de PNG.");
      } else if (param.get_name() == "WEBP_quality") {
        webpq_ = param.as_int();
        RCLCPP_INFO(this->get_logger(), "Se cambio la calidad de WEBP.");
      } /*else if (param.get_name() == "AVIF_quality") {
        avifq_ = param.as_int();
        RCLCPP_INFO(this->get_logger(), "Se cambio la calidad de AVIF.");
      } else if (param.get_name() == "AVIF_speed") {
        avifs_ = param.as_int();
        RCLCPP_INFO(this->get_logger(), "Se cambio la velocidad de AVIF.");
      } else if (param.get_name() == "JPEGXL_quality") {
        jpegxlq_ = param.as_int();
        RCLCPP_INFO(this->get_logger(), "Se cambio la calidad de JPEGXL.");
      } else if (param.get_name() == "JPEGXL_speed") {
        jpegxls_ = param.as_int();
        RCLCPP_INFO(this->get_logger(), "Se cambio la velocidad de JPEGXL.");
      }*/
    }

    std::string width, height, fps;
    if (board == "rasp") {
      width = res_map_[mode][0];
      height = res_map_[mode][1];
      fps = res_map_[mode][2];
    } else {
      width = res_map2_[mode][0];
      height = res_map2_[mode][1];
      fps = res_map2_[mode][2];
    }

    if (result.successful) {
      if (!reinit(preview, width, height, fps, vcam, hflip, vflip, board)) {
        RCLCPP_ERROR(this->get_logger(), "El reinicio de la camara fallo, por favor reinicia el nodo.");
        result.successful = false;
        result.reason = "El reinicio de la camara fallo, por favor reinicia el nodo.";
        proc_.stop();
      }
    }

    return result;
  }

  bool reinit(std::string preview, std::string width, std::string height, std::string fps, std::string vcam, std::string hflip, std::string vflip, std::string board) {

    cv::destroyAllWindows();

    if (board == "rasp") {
      cmd_ = "rpicam-vid " + preview + " -t 0 " + hflip + " " + vflip + " --codec yuv420 --width " + width + " --height " + height + " --framerate " + fps + " -o - | ffmpeg -f rawvideo -pix_fmt yuv420p -s " + width + "x" + height + " -r " + fps + " -i - -f v4l2 -pix_fmt yuv420p /dev/video" + vcam;
      if (!proc_.restart(cmd_)) {
        RCLCPP_ERROR(this->get_logger(), "No se pudo iniciar la camara virtual.");
        return false;
      }

      std::this_thread::sleep_for(std::chrono::seconds(5));

      if (!cap_.open("/dev/video"+vcam)) {
        RCLCPP_ERROR(this->get_logger(), "No se pudo abrir la cámara virtual con GStreamer.");
        return false;
      }
    } else if (board == "jnano" || board == "jorin") {
      cmd_ =
      "nvarguscamerasrc ! "
      "video/x-raw(memory:NVMM), width=" + width + ", height=" + height + ", format=NV12, framerate=" + fps + "/1 ! "
      "nvvidconv ! video/x-raw, format=BGRx ! "
      "videoconvert ! video/x-raw, format=BGR ! "
      "appsink drop=1";

      if (!cap_.open(cmd_, cv::CAP_GSTREAMER)) {
        RCLCPP_ERROR(this->get_logger(), "No se pudo abrir la cámara con GStreamer.");
        return false;
      }
    } else if (board == "PC") {
      if (!cap_.open(0, cv::CAP_V4L2)) {
        RCLCPP_ERROR(this->get_logger(), "No se pudo abrir la webcam con OpenCV.");
        return false;
      }
      if (!cap_.set(cv::CAP_PROP_FRAME_WIDTH, stoi(width)) || !cap_.set(cv::CAP_PROP_FRAME_HEIGHT, stoi(height)) || !cap_.set(cv::CAP_PROP_FPS, stoi(fps))) {
        RCLCPP_ERROR(this->get_logger(), "No se pudo cambiar la configuración de la webcam.");
        return false;
      }

      double rw = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
      double rh = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
      double rfps = cap_.get(cv::CAP_PROP_FPS);

      RCLCPP_INFO(this->get_logger(), "Camera reports: %.0fx%.0f @ %.2f FPS", rw, rh, rfps);
    }

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(int(1000/stoi(fps))), std::bind(&CameraPublisher::timer_callback, this)
    );

    board_ = board;
    prev_ = preview;
    vflip_ = vflip;
    hflip_ = hflip;
    d_fps_ = fps;

    RCLCPP_INFO(this->get_logger(), "Empezando a publicar frames.");
    return true;
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr com_publisher_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture cap_;
  std::string cmd_, board_, prev_, vflip_, hflip_, d_fps_;
  bool compression_;
  int com_format_, jpegq_, pngq_, webpq_;
  ProcessGuard proc_;
  std::map<int, std::vector<std::string>> res_map_, res_map2_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraPublisher>();

  if (!node->init()) {
    RCLCPP_FATAL(node->get_logger(), "Inicialización fallida. Cerrando.");
    node.reset();
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
