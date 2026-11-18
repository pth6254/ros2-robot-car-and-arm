// red_center_publisher.cpp
// 웹캠 영상에서 빨간색 객체를 검출하고, 가장 큰 컨투어의 중심점을
// cv_msg::msg::MsgCenter 형식으로 "red_center" 토픽에 발행한다.

#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <vector>

#include <opencv2/opencv.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cv_msg/msg/msg_center.hpp"

using namespace std::chrono_literals;

class RedCenterPublisher : public rclcpp::Node
{
public:
  RedCenterPublisher()
  : Node("red_center_publisher")
  {
    // ---------- 파라미터 ----------
    camera_index_ = declare_parameter<int>("camera_index", 0);
    min_area_     = declare_parameter<double>("min_area", 500.0);
    show_window_  = declare_parameter<bool>("show_window", true);

    // 빨강은 Hue가 0 근처와 180 근처 두 구간에 걸쳐 있다.
    s_min_     = declare_parameter<int>("s_min", 120);
    v_min_     = declare_parameter<int>("v_min", 70);
    h_low_max_ = declare_parameter<int>("h_low_max", 10);    // 구간1: 0 ~ h_low_max
    h_high_min_ = declare_parameter<int>("h_high_min", 170); // 구간2: h_high_min ~ 180

    // 실행 중 `ros2 param set`으로 바꾼 값을 멤버 변수에 반영
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        for (const auto & p : params) {
          const auto & name = p.get_name();
          if (name == "s_min") {s_min_ = p.as_int();}
          else if (name == "v_min") {v_min_ = p.as_int();}
          else if (name == "h_low_max") {h_low_max_ = p.as_int();}
          else if (name == "h_high_min") {h_high_min_ = p.as_int();}
          else if (name == "min_area") {min_area_ = p.as_double();}
          else {continue;}
          RCLCPP_INFO(get_logger(), "파라미터 변경: %s = %s",
                      name.c_str(), p.value_to_string().c_str());
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

    // ---------- 카메라 ----------
    // V4L2 백엔드 지정(GStreamer 경고 제거), 버퍼 1장으로 지연 최소화
    cap_.open(camera_index_, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      RCLCPP_FATAL(get_logger(), "카메라(%d)를 열 수 없습니다.", camera_index_);
      throw std::runtime_error("camera open failed");
    }
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

    // ---------- 퍼블리셔 / 타이머 ----------
    pub_ = create_publisher<cv_msg::msg::MsgCenter>("red_center", 10);
    timer_ = create_wall_timer(33ms, std::bind(&RedCenterPublisher::timer_callback, this));

    kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    RCLCPP_INFO(get_logger(), "red_center_publisher 시작 (camera %d)", camera_index_);
  }

  ~RedCenterPublisher() override
  {
    cap_.release();
    if (show_window_) {
      cv::destroyAllWindows();
    }
  }

private:
  void timer_callback()
  {
    // 1. 프레임 읽기
    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "프레임을 읽지 못했습니다.");
      return;
    }

    // 2. BGR -> HSV
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    // 3. 빨간색 마스크 (두 Hue 구간을 OR)
    cv::Mat mask_low, mask_high, mask;
    cv::inRange(hsv, cv::Scalar(0, s_min_, v_min_),
                cv::Scalar(h_low_max_, 255, 255), mask_low);
    cv::inRange(hsv, cv::Scalar(h_high_min_, s_min_, v_min_),
                cv::Scalar(180, 255, 255), mask_high);
    cv::bitwise_or(mask_low, mask_high, mask);

    // 4. 노이즈 제거
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel_);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel_);

    // 5. 컨투어 검출
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 6. 가장 큰 컨투어 선택
    int best_idx = -1;
    double best_area = 0.0;
    for (size_t i = 0; i < contours.size(); ++i) {
      double area = cv::contourArea(contours[i]);
      if (area > best_area) {
        best_area = area;
        best_idx = static_cast<int>(i);
      }
    }

    if (best_idx >= 0 && best_area >= min_area_) {
      // 7. 중심점 계산 (모멘트)
      cv::Moments m = cv::moments(contours[best_idx]);
      if (m.m00 > 0.0) {
        int cx = static_cast<int>(m.m10 / m.m00);
        int cy = static_cast<int>(m.m01 / m.m00);

        // 8. 시각화
        cv::drawContours(frame, contours, best_idx, cv::Scalar(0, 255, 0), 2);
        cv::circle(frame, cv::Point(cx, cy), 6, cv::Scalar(255, 0, 0), -1);
        cv::putText(frame, "(" + std::to_string(cx) + ", " + std::to_string(cy) + ")",
                    cv::Point(cx + 10, cy - 10), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(255, 255, 255), 2);

        // 9. 발행
        cv_msg::msg::MsgCenter msg;
        msg.x = cx;
        msg.y = cy;
        pub_->publish(msg);
      }
    }

    if (show_window_) {
      cv::imshow("frame", frame);
      cv::imshow("mask", mask);
      cv::waitKey(1);
    }
  }

  cv::VideoCapture cap_;
  cv::Mat kernel_;
  rclcpp::Publisher<cv_msg::msg::MsgCenter>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  int camera_index_;
  double min_area_;
  bool show_window_;
  int s_min_, v_min_, h_low_max_, h_high_min_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int ret = 0;
  try {
    rclcpp::spin(std::make_shared<RedCenterPublisher>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("red_center_publisher"), "%s", e.what());
    ret = 1;  // 실패를 launch/스크립트가 알 수 있도록 0이 아닌 값으로 종료
  }
  rclcpp::shutdown();
  return ret;
}
