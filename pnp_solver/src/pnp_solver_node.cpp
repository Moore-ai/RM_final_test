#include <iostream>
#include <vector>

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/armor.hpp"
#include "interfaces/msg/armors.hpp"
#include "interfaces/msg/armors3_d.hpp"
#include "interfaces/msg/armor3_d.hpp"
#include "_camera_info.hpp"

using Armors = interfaces::msg::Armors;
using Armor3D = interfaces::msg::Armor3D;
using Armors3D = interfaces::msg::Armors3D;
using json = nlohmann::json;

class PnPSolver : public rclcpp::Node {
    public:
    PnPSolver() : Node("pnp_solver") {
        
        camera_info.read("/home/lenovo/ros2_ws/src/pnp_solver/config/camera_info.yaml");
        
        // 初始化发布器
        armors3d_pub_ = create_publisher<Armors3D>("/armors_3d", rclcpp::QoS(10));

        this->armors_sub_ = create_subscription<Armors>(
            "/armors", rclcpp::QoS(10),
            [this] (const Armors::SharedPtr msg) {
                auto armors_3d_msg = interfaces::msg::Armors3D();
                // 处理每个装甲板
                for (const auto& armor : msg->armors) {
                    // 提取像素坐标
                    std::vector<cv::Point2d> pixels = {
                        {armor.x4, armor.y4}, {armor.x3, armor.y3},
                        {armor.x2, armor.y2}, {armor.x1, armor.y1}
                    };

                    // 定义装甲板物理尺寸 (135,125) (230,127)
                    std::vector<cv::Point3d> object_points_std = {
                        {-67.5, 62.5, 0}, {-67.5, -62.5, 0},
                        {67.5, 62.5, 0}, {67.5, -62.5, 0}
                    };
                    std::vector<cv::Point3d> object_points_hero = {
                        {-115, 63.5, 0}, {-115, -63.5, 0},
                        {115, 63.5, 0}, {115, -63.5, 0}
                    };

                    auto object_points = armor.is_hero ? std::move(object_points_hero) : std::move(object_points_std);

                    cv::Mat rvec, tvec;
                    bool success = cv::solvePnP(
                        object_points,
                        pixels,
                        camera_info.camera_matrix,
                        camera_info.distortion_coefficients,
                        rvec, tvec,
                        false, cv::SOLVEPNP_IPPE);

                    // // 在解算后添加合理性检查 
                    // if (cv::norm(tvec) > 1000.0) {
                    //     RCLCPP_WARN(this->get_logger(), "异常解算结果: tvec=%.2fm  跳过发布", cv::norm(tvec));
                    //     continue;
                    // }
                
                    if (success) {
                        std::cout << "Rotation Vector:\n" << rvec << std::endl;
                        std::cout << "Translation Vector:\n" << tvec << std::endl;

                        Armor3D armor_3d;

                        // 将旋转向量转换为旋转矩阵
                        cv::Mat rotation_matrix;
                        cv::Rodrigues(rvec, rotation_matrix);

                        std::cout << "Rotation Matrix:\n" << rotation_matrix << std::endl;

                        for (size_t i = 0; i < object_points.size(); ++i) {
                            cv::Mat point_3d_local = (cv::Mat_<double>(3,1) << 
                                object_points[i].x, 
                                object_points[i].y, 
                                object_points[i].z);
                          
                            // 转换到世界坐标系: R * local_point + tvec
                            cv::Mat point_3d_global = rotation_matrix * point_3d_local + tvec;
  
                            // 填充坐标到消息
                            armor_3d.coords[3*i] = point_3d_global.at<double>(0);  // x
                            armor_3d.coords[3*i + 1] = point_3d_global.at<double>(1); // y
                            armor_3d.coords[3*i + 2] = point_3d_global.at<double>(2); // z

                            armor_3d.timestamp=std::move(armor.timestamp);
                        }

                        armors_3d_msg.armors_3d.push_back(armor_3d);
                    } else {
                        RCLCPP_WARN(this->get_logger(), "PnP 解算失败！");
                    }
                }

                // 发布消息
                armors3d_pub_->publish(armors_3d_msg);
                RCLCPP_INFO(this->get_logger(),"armors_3d_msg 发送成功");
            }
        );
    }

    private:
    // cv::Mat camera_matrix_;  /* 加载相机内参 */
    // cv::Mat dist_coeffs_;  /* 加载畸变系数 */
    rclcpp::Subscription<Armors>::SharedPtr armors_sub_;
    rclcpp::Publisher<Armors3D>::SharedPtr armors3d_pub_;

    CameraInfo camera_info;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc,argv);
    auto node = std::make_shared<PnPSolver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}