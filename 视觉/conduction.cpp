#include "conduction.hpp"

static void get_BGR(int event,int x,int y,int flags,void* frame);
static void get_gray(int event,int x,int y,int flags,void* frame);

Conduction::Conduction(int num) {
    read_video_path();

    read_config("video_config_" + to_string(num) + ".json");

    video.open(video_paths[num-1]);

    if (!video.isOpened()) {
        std::cerr<<"Failed to get the video\n";
        exit(1);
    }
}

Conduction::~Conduction() {}

std::vector<FourPoints> Conduction::work_and_no_imshow(Mat& image) {
    // cout<<"-------------"<<endl;
    Mat gray_img,thresed_img,gaussianed_img;

    cvtColor(image,gray_img,COLOR_BGR2GRAY);
    // threshold(gray_img,thresed_img,160,255,THRESH_BINARY);
    // /*
    //     140,255
    //     195,255
    // */

    // // auto kernal=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
    // // cv::morphologyEx(thresed_img,thresed_img,cv::MORPH_OPEN,kernal,cv::Point(-1,-1),3);

    // GaussianBlur(thresed_img,gaussianed_img,Size(5,5),3.0);
    // /*
    //     (5,5),3
    //     (5,5),1
    // */

    // cv::convertScaleAbs(gray_img,gray_img,1.1); // 1.0

    // GaussianBlur(gray_img,gaussianed_img,Size(5,5),1.0);
    int ksize=config.GaussianBlur_ksize;
    double sigmaX=config.GaussianBlur_sigmaX;
    GaussianBlur(gray_img,gaussianed_img,Size(ksize,ksize),sigmaX);

    int thresh=config.threshold_thresh,
        maxval=config.threshold_maxval;
    threshold(gaussianed_img,thresed_img,thresh,maxval,THRESH_BINARY);

    vector<vector<Point>>contours;
    findContours(thresed_img,contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);

    Mat canvas(image.rows,image.cols,image.type(),Scalar(255,255,255));
    drawContours(canvas,contours,-1,Scalar(0,0,0),2);

    vector<Rect>gray_rects;
    vector<RotatedRect>gray_rotated_rects;
    // cout<<contours.size()<<endl;

    double height_width_rate=config.height_width_rate;
    double special_count_min=config.special_count_min,
           special_count_max=config.special_count_max;

    for (auto& contour:contours) {
        Rect rect=boundingRect(contour);

        Mat color_rect_mat,gray_rect_mat;
        
        if ((double)rect.height>(double)rect.width*height_width_rate) {
            /*
                2.2
                1.7
            */
            RotatedRect rotated_rect=minAreaRect(contour);

            gray_rect_mat=gaussianed_img(rect);
            // rectangle(canvas,rect,Scalar(0,0,255),2);

            // cout<<special_count_gray_average(gray_rect_mat)<<endl;
            if (special_count_min<=special_count_gray_average(gray_rect_mat)<=special_count_max) {
                /*
                    180<=   <=255.0
                */
                gray_rects.push_back(rect);
                gray_rotated_rects.push_back(rotated_rect);
                // rectangle(canvas,rect,Scalar(255,0,0),2);
            }
        }
    }

    const double HEIGHT_LIGHT=57.0;

    const double HEIGHT_BOARD=125.0;
    const double WIDTH_BOARD=135.0;

    const double HERO_WIDTH_BOARD=230.0;
    const double HERO_HEIGHT_BOARD=127.0;

    assert(gray_rects.size()==gray_rotated_rects.size());

    cv::Point2d p1,p2,p3,p4;
    std::vector<FourPoints>result;

    if (!gray_rects.empty()) {
        for (int i=0;i<gray_rects.size()-1;++i) {
            for (int j=i+1;j<gray_rects.size();++j) {
                if (area_about(gray_rects[i],gray_rects[j]) && rotated_angle_about(gray_rotated_rects[i],gray_rotated_rects[j])) {
                    // rectangle(canvas,gray_rects[i],Scalar(0,0,255),2);
                    // rectangle(canvas,gray_rects[j],Scalar(0,0,255),2);
                    // auto x=gray_rotated_rects[i].size.height;
                    // auto y=gray_rotated_rects[j].size.height;

                    auto middle_i=Conduction::rect_center(gray_rects[i]),
                         middle_j=Conduction::rect_center(gray_rects[j]);

                    auto i_height=gray_rects[i].height,
                         j_height=gray_rects[j].height;

                    double average_height=static_cast<double>(i_height+j_height)/2,
                           middle_distance=calculate_dist(middle_i,middle_j);
                    // calculate_dist(middle_i,middle_j)
                    // abs(middle_i.x-middle_j.x)
                    // middle_distance=abs(middle_i.x-middle_j.x);

                    const double standard_rate=WIDTH_BOARD/HEIGHT_LIGHT,
                           hero_standard_rate=HERO_HEIGHT_BOARD/HEIGHT_LIGHT;
                    double current_rate=middle_distance/average_height;

                    double height_rate = (double)i_height/(double)j_height;

                    double avai_min=config.height_rate_avai_min,
                           avai_max=config.height_rate_avai_max;
                    bool height_rate_avai = avai_min<=height_rate && height_rate<=avai_max; // 0.88 1.12
                                                                                    // 0.82 1.18
                    // cout<<"rate = "<<height_rate<<endl;

                    double curr_std_min=config.curr_std_rate_min,
                           curr_std_max=config.curr_std_rate_max;
                    double curr_hero_min=config.curr_hero_std_rate_min,
                           curr_hero_max=config.curr_hero_std_rate_max;
                    bool is_target = (between(current_rate/standard_rate,curr_std_min,curr_std_max) // 0.8 1.2
                         || between(current_rate/hero_standard_rate,curr_hero_min,curr_hero_max)) // 0.8 1.2
                                                                                                  // 0.7 1.3
                         && middle_angle_about(gray_rotated_rects[i],gray_rotated_rects[j]);

                    // cout<<current_rate/standard_rate<<" "<<current_rate/hero_standard_rate<<endl;   

                    if (is_target && height_rate_avai) {
                        // rectangle(canvas,gray_rects[i],Scalar(0,255,0),2);
                        // rectangle(canvas,gray_rects[j],Scalar(0,255,0),2);
                        rectangle(image,gray_rects[i],Scalar(0,255,0),2);
                        rectangle(image,gray_rects[j],Scalar(0,255,0),2);

                        double height_i = i_height * HEIGHT_BOARD / HEIGHT_LIGHT,
                               height_j = j_height * HEIGHT_BOARD / HEIGHT_LIGHT,
                               height_i_j = (height_i+height_j) / 4;

                        p1 = cv::Point2d(middle_i.x, middle_j.y - height_i_j);
                        p2 = cv::Point2d(middle_j.x, middle_j.y - height_i_j);
                        p3 = cv::Point2d(middle_j.x, middle_j.y + height_i_j);
                        p4 = cv::Point2d(middle_i.x, middle_i.y + height_i_j);

                        // cv::line(canvas,p1,p2,Scalar(0,255,0),2);
                        // cv::line(canvas,p2,p3,Scalar(0,255,0),2);
                        // cv::line(canvas,p3,p4,Scalar(0,255,0),2);
                        // cv::line(canvas,p4,p1,Scalar(0,255,0),2);

                        cv::line(image,p1,p2,Scalar(0,255,0),2);
                        cv::line(image,p2,p3,Scalar(0,255,0),2);
                        cv::line(image,p3,p4,Scalar(0,255,0),2);
                        cv::line(image,p4,p1,Scalar(0,255,0),2);

                        result.push_back(std::make_tuple(p1,p2,p3,p4));
                    }
                }
            }
        }
    }

    // imshow("image",image);
    // imshow("canvas",canvas);

    return result;
}

void Conduction::test_img(const string& path) {
    Mat image=imread(path,IMREAD_ANYCOLOR);

    if (image.empty()) {
        cerr<<"Failed to imread the image\n";
        exit(1);
    }

    imshow("image",image);
    work_and_no_imshow(image);
    waitKey(0);
}

void Conduction::show_img_BGR(const string& path) {
    Mat image=imread(path,IMREAD_ANYCOLOR);

    if (image.empty()) {
        cerr<<"Failed to imread the image\n";
        exit(1);
    }

    imshow("color_img",image);
    setMouseCallback("color_img",get_BGR,&image);
    waitKey(0);
}

void Conduction::show_img_gray(const string& path) {
    Mat image=imread(path,IMREAD_ANYCOLOR);

    if (image.empty()) {
        cerr<<"Failed to imread the image\n";
        exit(1);
    }

    imshow("gray_img",image);
    setMouseCallback("gray_img",get_gray,&image);
    waitKey(0);
}

static void get_BGR(int event,int x,int y,int flags,void* frame) {
    if (event!=EVENT_LBUTTONDOWN) {
        return;
    } else {
        auto image=static_cast<Mat*>(frame);
        Vec3b pixel=image->at<Vec3b>(y,x);

        cout<<(int)pixel[0]<<" "<<(int)pixel[1]<<" "<<(int)pixel[2]<<endl;
    }
}

static void get_gray(int event,int x,int y,int flags,void* frame) {
    if (event!=EVENT_LBUTTONDOWN) {
        return;
    } else {
        Mat* image=static_cast<Mat*>(frame);
        Mat gray_img;
        cvtColor(*image,gray_img,COLOR_BGR2GRAY);
        int pixel=(int)gray_img.at<uchar>(y,x);

        cout<<pixel<<endl;
    }
}

void Conduction::work() {
    Mat image;
    while (true) {
        video>>image;

        if (image.empty()) break;

        imshow("tmp",image);
        work_and_no_imshow(image);
        waitKey(this->rate);
    }
}

int Conduction::area(Mat& image) {
    return image.rows*image.cols;
}

double Conduction::angle(const Point2d& a,const Point2d& b) {
    double dy=a.y-b.y,
           dx=a.x-b.x;
    
    if (dx<1e-6) {
        return 90.0;
    }

    double slope=dy/dx,
           radian=atan(slope),
           degree=radian*180/CV_PI;

    if (degree<0) {
        degree+=180.0;
    } else if (degree==0 && dy<0) {
        degree=180.0;
    }
    return degree;
}

double Conduction::calculate_angle(const Point2d& a,const Point2d& b,const Point2d& c) {
    double angle_ab=angle(a,b),
           angle_bc=angle(b,c),
           include_angle=abs(angle_ab-angle_bc);

    if (include_angle>180.0) {
        include_angle=360.0-include_angle;
    }

    return include_angle;
}

double Conduction::calculate_dist(const Point2d& a,const Point2d& b) {
    double m=pow(a.x-b.x,2),
           n=pow(a.y-b.y,2);

    return pow(m+n,0.5);
}

bool Conduction::between(double element,double left,double right) {
    return element>=left && element<=right;
}

bool Conduction::area_about(Rect& a,Rect& b) {
    double height_rate=static_cast<double>(a.height)/b.height,
           width_rate=static_cast<double>(a.width)/b.width;

    double height_rate_min=config.area_about_height_rate_min,
           height_rate_max=config.area_about_height_rate_max;
    double width_rate_min=config.area_about_width_rate_min,
           width_rate_max=config.area_about_width_rate_max;

    // cout<<height_rate<<" "<<width_rate<<endl;
    return between(height_rate,height_rate_min,height_rate_max) && between(width_rate,width_rate_min,width_rate_max);  // 0.6 1.40 0.75 1.35
                                                                                                                       // 0.6 1.40 0.55 1.45
    // return true;
}

bool Conduction::rotated_rect_area_about(RotatedRect& a,RotatedRect& b) {
    Size2f a_size=a.size,
           b_size=b.size;
    
    double height_rate=static_cast<double>(a_size.height)/b_size.height,
           width_rate=static_cast<double>(a_size.width)/b_size.width;

    return between(height_rate,0.8,1.2) && between(width_rate,0.8,1.2);    // 0.8 1.2 0.8 1.2
}

bool Conduction::rotated_angle_about(RotatedRect& a,RotatedRect& b) {
    float rotated_angle_rate=a.angle/b.angle;

    // cout<<a.angle<<" "<<b.angle<<endl;

    // cout<<rotated_angle_rate<<endl;

    // return between((double)rotated_angle_rate,0.8,1.2) || (between(abs((a.angle+b.angle)/2-45.0),0,3.0));
    // return between((double)rotated_angle_rate,0.8,1.2);

    double delta_min=config.rotated_angle_about_delta_min,
           delta_max=config.rotated_angle_about_delta_max;
    
    double delta=abs(a.angle-b.angle);
    bool available = (0<=90-a.angle<=5 && 0<=b.angle<=5) || (0<=90-b.angle<=5 && 0<=a.angle<=5);
    return between(delta,delta_min,delta_max) || available;

    // return true;
}

bool Conduction::middle_angle_about(RotatedRect& a,RotatedRect& b) {
    // const double PI=3.1415927;

    // auto middle_a=a.center,
    //      middle_b=b.center;

    // double delta_x=abs(middle_a.x-middle_b.x),
    //        delta_y=abs(middle_a.y-middle_b.y);

    // double middle_angle_rate=std::atan(delta_y/delta_x);
    // // std::cout<<middle_angle_rate<<std::endl;

    // assert(middle_angle_rate>=0);

    // return between(middle_angle_rate,0,0.20);
    return true;
}
double Conduction::special_count_gray_average(Mat& image) {
    assert(image.type()==CV_8UC1);

    double gray_sum=0;
    int gray_amount=0;

    for (int i=0;i<image.rows;++i) {
        for (int j=0;j<image.cols;++j) {
            int gray=(int)image.at<uchar>(i,j);

            if (between(gray,80)) {  // 80
                gray_sum+=gray;
                gray_amount++;
            }
        }
    }

    return gray_sum/gray_amount;
}

void Conduction::read_video_path() {
    fstream in_json("/home/lenovo/桌面/视觉/config/path.json");

    if (!in_json.is_open()) {
        cerr<<"Failed to get config from the path.json\n";
        exit(1);
    }

    in_json>>this->content;

    for (const auto& video_path:this->content["video_paths"]) {
        video_paths.push_back(video_path.get<string>());
    }

    in_json.close();
    this->content.clear();
}

void Conduction::read_config(const string& path) {
    string file_path="/home/lenovo/桌面/视觉/config/";
    file_path.append(path);

    fstream in_json(file_path);

    if (!in_json.is_open()) {
        cerr<<"Failed to get config from the "<<file_path<<endl;
        exit(1);
    }

    in_json>>this->content;

    config.GaussianBlur_ksize=content["GaussianBlur_ksize"].get_i;
    config.GaussianBlur_sigmaX=content["GaussianBlur_sigmaX"].get_d;
    config.threshold_thresh=content["threshold_thresh"].get_i;
    config.threshold_maxval=content["threshold_maxval"].get_i;
    config.height_width_rate=content["height_width_rate"].get_d;
    config.special_count_min=content["special_count_min"].get_d;
    config.special_count_max=content["special_count_max"].get_d;
    config.height_rate_avai_min=content["height_rate_avai_min"].get_d;
    config.height_rate_avai_max=content["height_rate_avai_max"].get_d;
    config.area_about_height_rate_min=content["area_about_height_rate_min"].get_d;
    config.area_about_height_rate_max=content["area_about_height_rate_max"].get_d;
    config.area_about_width_rate_min=content["area_about_width_rate_min"].get_d;
    config.area_about_width_rate_max=content["area_about_width_rate_max"].get_d;
    config.rotated_angle_about_delta_min=content["rotated_angle_about_delta_min"].get_d;
    config.rotated_angle_about_delta_max=content["rotated_angle_about_delta_max"].get_d;
    config.curr_std_rate_min=content["curr_std_rate_min"].get_d;
    config.curr_std_rate_max=content["curr_std_rate_max"].get_d;
    config.curr_hero_std_rate_min=content["curr_hero_std_rate_min"].get_d;
    config.curr_hero_std_rate_max=content["curr_hero_std_rate_max"].get_d;

    in_json.close();
    this->content.clear();

    std::fstream in_transmit("/home/lenovo/桌面/视觉/config/transmit.json");

    if (!in_transmit.is_open()) {
        cerr<<"Failed to get config from the transmit.json"<<endl;
        exit(1);
    }

    in_transmit>>this->content;

    this->rate=content["rate"].get_i;

    in_transmit.close();
    this->content.clear();
}

void Conduction::debug_RotatedRect(Mat& img,RotatedRect& rect,Scalar color) {
    Point2f vertices[4];
    rect.points(vertices);

    for (int i=0;i<4;++i) {
        Point pt((int)vertices[i].x,(int)vertices[i].y);
        vertices[i]=pt;
    }

    line(img,vertices[0],vertices[1],color,2);
    line(img,vertices[1],vertices[2],color,2);
    line(img,vertices[2],vertices[3],color,2);
    line(img,vertices[3],vertices[0],color,2);
}

void Conduction::show_count_result() {
    cout<<"result = ";
    for (const auto& curr : this->count_result) {
        std::cout<<curr<<" ";
    }
    cout<<endl;
    count_result.clear();
}

cv::Point2d Conduction::rect_center(const cv::Rect& rect) {
    return cv::Point2d((double)rect.x+(double)rect.width/2,(double)rect.y+(double)rect.height/2);
}

// 计算旋转速度
double Conduction::calculateRotationSpeed(const Mat& prevRotation, const Mat& currRotation, double timeInterval) {
    // 将旋转向量转换为旋转矩阵
    Mat prevRotationMatrix, currRotationMatrix;
    Rodrigues(prevRotation, prevRotationMatrix);
    Rodrigues(currRotation, currRotationMatrix);

    // 将旋转矩阵转换为四元数
    Mat prevQuaternion = prevRotationMatrix.t() * currRotationMatrix;
    Mat currQuaternion = currRotationMatrix.t() * prevRotationMatrix;

    // 计算四元数的差值
    Mat qDiff = currQuaternion - prevQuaternion;

    // 计算旋转速度（弧度/秒）
    double rotationSpeed = norm(qDiff) / timeInterval;
    return rotationSpeed;
}

// 预测未来位置
void Conduction::predictFuturePosition(const Mat& currentRotation, const Mat& currentTranslation, double rotationSpeed, double timeInterval, vector<Point3f>& futurePoints) {
    // 计算未来旋转角度
    double futureAngle = rotationSpeed * timeInterval;

    // 假设绕 Z 轴旋转
    Vec3d rotationAxis = {0, 0, 1};

    // 计算未来旋转矩阵
    Mat futureRotation = getRotationMatrix2D(Point2f(0, 0), futureAngle, 1.0);

    // 计算未来位置
    for (auto& point : futurePoints) {
        Mat point3d = (Mat_<double>(4, 1) <<
            point.x, point.y, point.z, 1);
        Mat transformedPoint = futureRotation * point3d + currentTranslation;
        point.x = transformedPoint.at<double>(0);
        point.y = transformedPoint.at<double>(1);
        point.z = transformedPoint.at<double>(2);
    }
}