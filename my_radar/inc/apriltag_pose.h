//
// Created by demon on 20-6-7.
//

#ifndef RADAR_APRILTAG_POSE_H
#define RADAR_APRILTAG_POSE_H

#include <iostream>
#include "opencv2/opencv.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "radar_main.hpp"
#include "switch_function.h"

extern "C" {
#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/tag25h9.h"
#include "apriltag/tag16h5.h"
#include "apriltag/tagCircle21h7.h"
#include "apriltag/tagCircle49h12.h"
#include "apriltag/tagCustom48h12.h"
#include "apriltag/tagStandard41h12.h"
#include "apriltag/tagStandard52h13.h"
#include "apriltag/common/getopt.h"
#include "apriltag/apriltag_pose.h"
#include "apriltag/common/homography.h"
}



#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;




class UsingApriltag{

public:

    UsingApriltag(std::string family, double left_fx,double left_fy,double left_cx,double left_cy);   //初始化apriltag各种参数


    void positionEstimation(cv::Mat image, Eigen::Matrix3f &rvec,Eigen::Vector3f  &tvec);
    void camera2world(Eigen::Vector3f P_cam,Eigen::Matrix3f R_cam_tag, Eigen::Vector3f T_cam_tag,Eigen::Vector3f &P_world);


    ~UsingApriltag();

private:
    apriltag_detection_info_t info;
    apriltag_detector_t *td ;
    const char *famname;
    getopt_t *getopt;
    apriltag_family_t *tf;


    inline double standardRad(double t) {  //定义了角度归一化函数，角度范围统一输出范围是[-pi,pi].
        if (t >= 0.) {
            t = fmod(t+PI, TWOPI) - PI;
        } else {
            t = fmod(t-PI, -TWOPI) + PI;
        }
        return t;
    }

};














#endif //RADAR_APRILTAG_POSE_H
