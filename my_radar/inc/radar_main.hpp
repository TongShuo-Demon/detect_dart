//
// Created by demon on 2019/11/11.
//
#ifndef RADAR_RADAR_MAIN_HPP
#define RADAR_RADAR_MAIN_HPP

#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include "stdio.h"
namespace radar{

    extern  cv::Mat cv_left_camera_matrix;                               // 相机内参
    extern  cv::Mat cv_left_dist_coeffs;                                 // 畸变系数
    extern cv::Mat cv_right_camera_matrix;
    extern cv::Mat cv_right_dist_coeffs;                                 // 畸变系数

    extern cv::Mat cv_left_rotation;                                    //左相机与左相机关系
    extern cv::Mat cv_left_translation;
    extern cv::Mat cv_right_rotation;                                    //右相机与左相机（世界坐标系）关系
    extern cv::Mat cv_right_translation;


    extern int enemy_color;                                //敌方颜色
    extern int dart_min_area;                                  //小车最小面积
    extern int dart_max_area;                                  //小车最大面积

    extern cv::Ptr<cv::BackgroundSubtractor> KNN;                 //KNN Background subtractor
    extern float dart_heigth_div_width_min;                   //高度比上宽度
    extern float dart_heigth_div_width_max;
    extern float dart_area_ratio;
    extern float dart_heigth_min;
    extern float dart_heigth_max;

    extern float blob_area_min;
    extern float blob_area_max;
    extern float blob_heigth_div_width_min;
    extern float blob_heigth_div_width_max;

    extern std::string apriltag_family;
    extern float tag_size;
    extern cv::Mat transition_matrix;                                 //卡尔曼滤波器的转移矩阵

    void radar_init();
}






#endif //RADAR_RADAR_MAIN_HPP
