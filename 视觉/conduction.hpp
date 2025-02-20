#pragma once
#define CONDUCTION_HPP

#include <iostream>
#include <vector>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include "config.hpp"

using namespace cv;
using namespace std;

using BGR = tuple<double,double,double>;
using json = nlohmann::json;
using FourPoints = tuple<Point2d,Point2d,Point2d,Point2d>;

#define get_i get<int>()
#define get_d get<double>()

class Conduction {
    public:
    VideoCapture video;
    
    Conduction(int num);
    ~Conduction();
    void work();
    void test_img(const string& path);
    void show_img_BGR(const string& path);
    void show_img_gray(const string& path);
    std::vector<FourPoints> work_and_no_imshow(Mat& image);

    static inline int area(Mat& image);
    static double angle(const Point2d& a,const Point2d& b);
    static double calculate_angle(const Point2d& a,const Point2d& b,const Point2d& c);
    static inline double calculate_dist(const Point2d& a,const Point2d& b);
    static inline bool between(double element,double left,double right=255.0);

    inline bool area_about(Rect& a,Rect& b);
    bool rotated_rect_area_about(RotatedRect& a,RotatedRect& b);
    inline bool rotated_angle_about(RotatedRect& a,RotatedRect& b);
    static inline bool middle_angle_about(RotatedRect& a,RotatedRect& b);
    static double special_count_gray_average(Mat& image);
    static cv::Point2d rect_center(const cv::Rect& rect);

    void read_video_path();
    void read_config(const string& path);

    static void debug_RotatedRect(Mat& img,RotatedRect& rect,Scalar color);
    void show_count_result();

    static double calculateRotationSpeed(const Mat& prevRotation, const Mat& currRotation, double timeInterval);
    static void predictFuturePosition(const Mat& currentRotation, const Mat& currentTranslation, double rotationSpeed, double timeInterval, vector<Point3f>& futurePoints);

    private:
    json content;
    vector<string>video_paths;
    Config config;
    int rate;

    vector<double>count_result;
};
