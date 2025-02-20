#include <iostream>
#include <vector>
#include <memory>
#include <optional>

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/armors3_d.hpp"
#include "interfaces/msg/armor3_d.hpp"
#include "interfaces/msg/aim_points.hpp"
#include <geometry_msgs/msg/point.hpp>

using Armors3D = interfaces::msg::Armors3D;
using Armor3D = interfaces::msg::Armor3D;
using AimPoints = interfaces::msg::AimPoints;
using Point = geometry_msgs::msg::Point;

class BallisticCalculator : public rclcpp::Node {
public:
    BallisticCalculator() : Node("ballistic_calculator"), last_armors_3d_(std::nullopt) {
        this->declare_parameter("bullet_speed", 15.0);
        this->declare_parameter("camera.fx", 2331.31938);
        this->declare_parameter("camera.fy", 2321.2793);
        this->declare_parameter("camera.cx", 727.43998);
        this->declare_parameter("camera.cy", 526.88717);

        armors3d_sub_ = this->create_subscription<Armors3D>(
            "/armors_3d", rclcpp::QoS(10),
            [this](const Armors3D::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "接收到 /armors_3d");
                processArmors3D(msg);
            });

        aim_pub_ = this->create_publisher<AimPoints>("/aim_points", rclcpp::QoS(10));
    }

private:
    void processArmors3D(const Armors3D::SharedPtr msg) {
        // 处理装甲板丢失情况
        if (msg->armors_3d.empty()) {
            last_armors_3d_ = std::nullopt;
            filtered_angular_velocities_.clear();
            AimPoints empty_aim;
            aim_pub_->publish(empty_aim);
            RCLCPP_WARN(this->get_logger(), "装甲板丢失，清空历史数据");
            return;
        }

        // 首次接收数据初始化
        if (!last_armors_3d_.has_value()) {
            last_armors_3d_ = msg->armors_3d;
            last_time_ = this->get_clock()->now();
            filtered_angular_velocities_.resize(msg->armors_3d.size(), 0.0);
            return;
        }

        // 数量变化检查
        if (last_armors_3d_->size() != msg->armors_3d.size()) {
            RCLCPP_WARN(this->get_logger(), "装甲板数量变化 %zu->%zu", 
                       last_armors_3d_->size(), msg->armors_3d.size());
            last_armors_3d_ = msg->armors_3d;
            filtered_angular_velocities_.clear();
            filtered_angular_velocities_.resize(msg->armors_3d.size(), 0.0);
            return;
        }

        // 时间差有效性检查
        auto current_time = this->get_clock()->now();
        double delta_time = (current_time - last_time_).seconds();
        if (delta_time < 1e-6) {
            RCLCPP_WARN(this->get_logger(), "时间差过小，跳过计算");
            return;
        }

        AimPoints aim_points_msg;
        double bullet_speed = this->get_parameter("bullet_speed").as_double();

        for (size_t i = 0; i < msg->armors_3d.size(); ++i) {
            const Armor3D& armor = msg->armors_3d[i];
            // 解析顶点坐标
            cv::Point3d p1(armor.coords[0], armor.coords[1], armor.coords[2]);
            cv::Point3d p2(armor.coords[3], armor.coords[4], armor.coords[5]);
            cv::Point3d p3(armor.coords[6], armor.coords[7], armor.coords[8]);
            cv::Point3d p4(armor.coords[9], armor.coords[10], armor.coords[11]);
            cv::Point3d current_center = (p1 + p2 + p3 + p4) / 4.0;

            // 计算距离
            double distance = cv::norm(current_center);
            if (distance < 0.1) continue;

            // 弹丸飞行时间
            double bullet_time = distance / bullet_speed;

            // 历史数据获取
            const Armor3D& last_armor = last_armors_3d_->at(i);
            cv::Point3d last_p1(last_armor.coords[0], last_armor.coords[1], last_armor.coords[2]);
            cv::Point3d last_p2(last_armor.coords[3], last_armor.coords[4], last_armor.coords[5]);
            cv::Point3d last_p3(last_armor.coords[6], last_armor.coords[7], last_armor.coords[8]);
            cv::Point3d last_p4(last_armor.coords[9], last_armor.coords[10], last_armor.coords[11]);
            cv::Point3d last_center = (last_p1 + last_p2 + last_p3 + last_p4) / 4.0;

            // 计算平均角度变化
            double total_delta_angle = 0.0;
            std::array<cv::Point3d, 4> current_points = {p1, p2, p3, p4};
            std::array<cv::Point3d, 4> last_points = {last_p1, last_p2, last_p3, last_p4};
            for (int j = 0; j < 4; ++j) {
                cv::Point3d current_r = current_points[j] - current_center;
                cv::Point3d last_r = last_points[j] - last_center;
                double current_angle = atan2(current_r.y, current_r.x);
                double last_angle = atan2(last_r.y, last_r.x);
                double delta_angle = current_angle - last_angle;
                // 角度跳变修正
                if (delta_angle > M_PI) delta_angle -= 2 * M_PI;
                else if (delta_angle < -M_PI) delta_angle += 2 * M_PI;
                total_delta_angle += delta_angle;
            }
            double delta_angle = total_delta_angle / 4.0;
            double angular_velocity = delta_angle / delta_time;

            // 角速度滤波
            if (i < filtered_angular_velocities_.size()) {
                double alpha = 0.3;
                angular_velocity = alpha * angular_velocity + 
                                 (1 - alpha) * filtered_angular_velocities_[i];
                filtered_angular_velocities_[i] = angular_velocity;
            } else {
                filtered_angular_velocities_.push_back(angular_velocity);
            }

            // 计算预瞄点
            double lead_angle = angular_velocity * bullet_time;
            double armor_radius = (cv::norm(p1 - p3) + cv::norm(p2 - p4)) / 4.0;
            cv::Point3d lead_offset(
                armor_radius * sin(lead_angle),
                -armor_radius * (1 - cos(lead_angle)),
                0
            );

            // 3D投影
            cv::Point3d target_point = current_center + lead_offset;
            double fx = this->get_parameter("camera.fx").as_double();
            double fy = this->get_parameter("camera.fy").as_double();
            double cx = this->get_parameter("camera.cx").as_double();
            double cy = this->get_parameter("camera.cy").as_double();
            double u = (fx * target_point.x / target_point.z) + cx;
            double v = (fy * target_point.y / target_point.z) + cy;

            aim_points_msg.x.push_back(u);
            aim_points_msg.y.push_back(v);
        }

        aim_pub_->publish(aim_points_msg);
        last_armors_3d_ = msg->armors_3d;
        last_time_ = current_time;
    }

    rclcpp::Time last_time_;
    rclcpp::Subscription<Armors3D>::SharedPtr armors3d_sub_;
    rclcpp::Publisher<AimPoints>::SharedPtr aim_pub_;
    std::optional<std::vector<Armor3D>> last_armors_3d_;
    std::vector<double> filtered_angular_velocities_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BallisticCalculator>());
    rclcpp::shutdown();
    return 0;
}