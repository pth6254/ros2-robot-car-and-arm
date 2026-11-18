// red_center_subscriber.cpp
// "red_center" 토픽의 중심점 좌표를 받아 터미널에 출력한다.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "cv_msg/msg/msg_center.hpp"

class RedCenterSubscriber : public rclcpp::Node
{
public:
  RedCenterSubscriber()
  : Node("red_center_subscriber")
  {
    sub_ = create_subscription<cv_msg::msg::MsgCenter>(
      "red_center", 10,
      [this](const cv_msg::msg::MsgCenter::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "Red center -> x: %d, y: %d", msg->x, msg->y);
      });
    RCLCPP_INFO(get_logger(), "red_center_subscriber 시작");
  }

private:
  rclcpp::Subscription<cv_msg::msg::MsgCenter>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RedCenterSubscriber>());
  rclcpp::shutdown();
  return 0;
}
