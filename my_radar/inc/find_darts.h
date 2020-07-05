//
// Created by demon on 20-5-23.
//

#ifndef RADAR_FIND_DARTS_H
#define RADAR_FIND_DARTS_H

#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include "radar_main.hpp"
#include "switch_function.h"


#define RED 1
#define BLUE 0


class Dart{

    public:
    cv::Point2f dart_center;
    int dart_color;

};
typedef std::vector<Dart>  darts;




typedef std::vector<cv::Rect2d>  RectDarts;

class DartsDetect{

public:

    bool findDarts(cv::Mat src,darts &move_darts,int color_type); //寻找飞镖
    bool extractionDart(const cv::Mat src, RectDarts &move_darts); //提取运动物体
    cv::Point2f findOneDetect(cv::Mat src,std::string,float &S );     //最后检测

    bool matchDoubleDarts(cv::Mat src_left,cv::Mat src_right,cv::Point2f &uv_left,cv::Point2f &uv_right,float &K);     //最后检测


private:
    cv::Mat imagePreProcess(cv::Mat &src,uint8_t enemy_color);
    cv::Mat processVideoKNN(const cv::Mat &src);

};





void findDartsTest(cv::Mat src,int color_type,cv::Point2f &uv_pixel);









#endif //RADAR_FIND_DARTS_H
