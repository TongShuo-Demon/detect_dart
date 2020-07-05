//
// Created by demon on 20-5-23.
//

#ifndef RADAR_DOUBLE_MEASUREMENT_H
#define RADAR_DOUBLE_MEASUREMENT_H

#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>  // Eigen 几何模块
#include <iostream>
#include <Eigen/Dense>

#define PI 3.1415926

/* *****************************************
 *
 *双目测距类
 *
 *
 *******************************************/
class DoubleMeasurement
{

public:
    DoubleMeasurement(Eigen::Matrix3f left_intrinsic,Eigen::Matrix<float, 1, 5> left_distortion,
                      Eigen::Matrix3f right_intrinsic,Eigen::Matrix<float, 1, 5> right_distortion,
                      Eigen::Matrix3f left_rotation, Eigen::Vector3f left_translation,
                      Eigen::Matrix3f right_rotation,Eigen::Vector3f right_translation);

    cv::Point3f pixelToWorldCoordinate(cv::Point2f uvLeft,cv::Point2f uvRight);

    cv::Point2f pixelDistortionCorrection(cv::Point2f &connor,Eigen::Matrix3f cameraMatrix,Eigen::Matrix<float, 5, 1> distCoeffs);

    cv::Point2f worldTopixelCoordinate(cv::Point3f world_point,float intrinsic[3][3],float translation[1][3],float rotation[3][3]);

    void cameraToWorld(Eigen::Vector3d camera_point,Eigen::Vector3d &world_point);


private:


    Eigen::Matrix3f left_intrinsic_,left_rotation_,right_intrinsic_,right_rotation_;
    Eigen::Matrix<float, 1, 5>  left_distortion_, right_distortion_;     //一行五列
    Eigen::Vector3f  left_translation_ ,right_translation_;               //一行3列
    Eigen::Matrix<float, 3, 4> m_left_transform_;  //变换矩阵
    Eigen::Matrix<float, 3, 4> m_right_transform_;

};

#endif //RADAR_DOUBLE_MEASUREMENT_H



