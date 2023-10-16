#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>

class ImageToV4l2 : public rclcpp::Node
{
public:
    ImageToV4l2() : Node("image_to_v4l2")
    {
        this->declare_parameter<std::string>("video_device", "/dev/video20");
        video_device_ = this->get_parameter("video_device").as_string();

std::string gstreamer_pipeline = "appsrc ! videoconvert ! videoscale ! video/x-raw,format=I420 ! v4l2sink device=" + video_device_;
writer_.open(gstreamer_pipeline, 0, 10, cv::Size(640, 480), true);

if (!writer_.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Could not open video writer for device %s", video_device_.c_str());
    rclcpp::shutdown();
}


        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10, std::bind(&ImageToV4l2::image_callback, this, std::placeholders::_1));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;

        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        
        cv::Mat bgr_image;
        cv::cvtColor(cv_ptr->image, bgr_image, cv::COLOR_RGB2BGR);
        
        if (!bgr_image.empty()) {
            writer_.write(bgr_image);
            cv::imshow("Received Image", bgr_image);
            cv::waitKey(1);  // Wait for a short moment. Adjust this value if needed.
        }
    }


    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    cv::VideoWriter writer_;
    std::string video_device_;
    
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageToV4l2>());
    rclcpp::shutdown();
    return 0;
}

