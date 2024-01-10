#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.h>
#include <cv_bridge/cv_bridge.h>

using std::placeholders::_1;

class ImageConverter : public rclcpp::Node
{
private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "YAS");

    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      auto message = cv_ptr->toCompressedImageMsg();

      publisher_->publish(*message);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Fuck");
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;

public:
  ImageConverter() : Node("image_converter")
  {
    RCLCPP_INFO(this->get_logger(), "New test node started");

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/color/image_raw", 10, std::bind(&ImageConverter::topic_callback, this, _1));

    publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("Test", 10);
  };

  ~ImageConverter() {}
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageConverter>());
  rclcpp::shutdown();

  return 0;
}