#include <vector>
#include <iostream>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "interfaces/msg/armors.hpp"
#include "interfaces/msg/armor.hpp"
#include "interfaces/msg/aim_points.hpp"
#include "conduction.hpp"

using Armors = interfaces::msg::Armors;
using Armor = interfaces::msg::Armor;
using AimPoints = interfaces::msg::AimPoints;

class Armor_detector : public rclcpp::Node {
  public:
  Armor_detector() : Node("armor_detector") {
    pub_ = this->create_publisher<Armors>("/armors", rclcpp::QoS(10));

    sub_ = this->create_subscription<AimPoints>("/aim_points", rclcpp::QoS(10), 
          [this] (const AimPoints::SharedPtr msg) {
            show_res(msg);
          });

    processing_thread_ = std::thread(&Armor_detector::video_processing_loop, this);
  }
  virtual ~Armor_detector() {
    running_ = false;
    if (processing_thread_.joinable()) processing_thread_.join();
  }

  private:
  void video_processing_loop() {
    cv::Mat frame;
    Conduction conduction(4);
    while (rclcpp::ok() && running_.load(std::memory_order_relaxed)) {
      conduction.video >> frame;

      if (frame.empty()) break;

      std::vector<FourPoints> result = conduction.work_and_no_imshow(frame);

      {
        std::lock_guard<std::mutex>guard(mtx_);
        // 保持队列最多3帧避免堆积
        while (buffer_.size() >= 3) {
          buffer_.pop();
        }
        buffer_.push(frame.clone());
        cv_.notify_one();
      }
  
      Armors armors;
      for (const auto& item : result) {
        Armor armor;
        armor.is_hero=false;
        armor.timestamp=this->now();
        armor.x1=std::get<0>(item).x;
        armor.y1=std::get<0>(item).y;
        armor.x2=std::get<1>(item).x;
        armor.y2=std::get<1>(item).y;
        armor.x3=std::get<2>(item).x;
        armor.y3=std::get<2>(item).y;
        armor.x4=std::get<3>(item).x;
        armor.y4=std::get<3>(item).y;
  
        armors.armors.push_back(std::move(armor));
      }
      pub_->publish(std::move(armors));
      RCLCPP_INFO(this->get_logger(),"armors 发送成功");
  
      cv::waitKey(40);
    }
  }

  void show_res(const AimPoints::SharedPtr msg) {
    cv::Mat frame;
    {
      std::unique_lock<std::mutex>lock(mtx_);
      cv_.wait(lock, [this] { return !buffer_.empty(); });
      frame = buffer_.front();
      buffer_.pop();
    }
    auto arr_x = msg->x, arr_y = msg->y;
    assert(arr_x.size()==arr_y.size());

    for (size_t i=0;i<arr_x.size();++i) {
      cv::circle(frame, cv::Point(arr_x[i],arr_y[i]),10,cv::Scalar(0,255,0));
    }

    cv::imshow("frame",frame);
    RCLCPP_INFO(this->get_logger(),"显示图片中...");
    cv::waitKey(40);
  }

  std::thread processing_thread_;
  std::atomic<bool>running_ {true};
  rclcpp::Publisher<Armors>::SharedPtr pub_;
  rclcpp::Subscription<AimPoints>::SharedPtr sub_;

  std::mutex mtx_;
  std::condition_variable cv_;
  std::queue<cv::Mat> buffer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc,argv);
  auto node = std::make_shared<Armor_detector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}