#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class ImageToV4l2 : public rclcpp::Node {
public:
  ImageToV4l2() : Node("image_to_v4l2") {
    this->declare_parameter<std::string>("video_device", "/dev/video20");
    video_device_ = this->get_parameter("video_device").as_string();

    this->declare_parameter<std::string>("image_topic_name", "/image_raw");
    std::string image_topic_name =
        this->get_parameter("image_topic_name").as_string();

    std::string gstreamer_pipeline =
        "appsrc ! videoconvert ! videoscale ! video/x-raw,format=I420 ! "
        "v4l2sink device=" +
        video_device_;
    writer_.open(gstreamer_pipeline, 0, 10, cv::Size(640, 480), true);

    if (!writer_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Could not open video writer for device %s",
                   video_device_.c_str());
      rclcpp::shutdown();
    }

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_name, 10,
        std::bind(&ImageToV4l2::image_callback, this, std::placeholders::_1));
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
      cv_ptr =
          cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat depth_image = cv_ptr->image;
    cv::Mat normalized_depth, colored_depth, resized_depth;
    // Normalize the depth image to range [0, 255]
    cv::normalize(depth_image, normalized_depth, 0, 255, cv::NORM_MINMAX,
                  CV_8UC1);
    // Convert normalized depth to RGB using colormap
    cv::applyColorMap(normalized_depth, colored_depth, cv::COLORMAP_JET);
    // Resize the colored depth image to 640x480
    cv::resize(colored_depth, resized_depth, cv::Size(640, 480));

    if (!resized_depth.empty()) {
      writer_.write(resized_depth);
      cv::imshow("Received Depth Image", resized_depth);
      cv::waitKey(1); // Wait for a short moment. Adjust this value if needed.
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  cv::VideoWriter writer_;
  std::string video_device_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageToV4l2>());
  rclcpp::shutdown();
  return 0;
}
