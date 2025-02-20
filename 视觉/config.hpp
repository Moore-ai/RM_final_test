#pragma once
#define CONFIG_HPP

class Config {
    public:
    int GaussianBlur_ksize;
    double GaussianBlur_sigmaX;

    int threshold_thresh, threshold_maxval;

    double height_width_rate;

    double special_count_min, special_count_max;

    double height_rate_avai_min, height_rate_avai_max;

    double area_about_height_rate_min, area_about_height_rate_max;
    double area_about_width_rate_min, area_about_width_rate_max;

    double rotated_angle_about_delta_min, rotated_angle_about_delta_max;

    double curr_std_rate_min, curr_std_rate_max;
    double curr_hero_std_rate_min, curr_hero_std_rate_max;
};