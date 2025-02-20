#pragma once
#include <opencv2/opencv.hpp>

class CameraInfo {
    public:
    int image_width, image_height;
    
    cv::Mat camera_matrix,
            distortion_coefficients,
            projection_matrix;

    void read(const std::string& path) {

        cv::FileStorage fs;

        if (!fs.open(path, cv::FileStorage::READ)) {
            std::cerr << "Error: Could not open YAML file." << std::endl;
            exit(1);
        }

        fs["image_width"]>>image_width;
        fs["image_height"]>>image_height;
        fs["camera_matrix"]>>camera_matrix;
        fs["distortion_coefficients"]>>distortion_coefficients;
        fs["projection_matrix"]>>projection_matrix;

        fs.release();
    }
};