//
// Created by demon on 2019/11/11.
//
#include <radar_main.hpp>

namespace radar{

    cv::Mat cv_left_camera_matrix;                               // 相机内参
    cv::Mat cv_left_dist_coeffs;                                 // 畸变系数

    cv::Mat cv_right_camera_matrix;
    cv::Mat cv_right_dist_coeffs;                                 // 畸变系数

    cv::Mat cv_left_rotation;                                    //左相机与左相机关系
    cv::Mat cv_left_translation;

    cv::Mat cv_right_rotation;                                    //右相机与左相机（世界坐标系）关系
    cv::Mat cv_right_translation;

    cv::Mat transition_matrix;                                 //卡尔曼滤波器的转移矩阵

    int enemy_color;                                //敌方颜色
     int dart_min_area;                           //小车最小面积
     int dart_max_area;                           //小车最大面积
    cv::Ptr<cv::BackgroundSubtractor> KNN;         //MOG2 Background subtractor
    float dart_heigth_div_width_min;
    float dart_heigth_div_width_max;
    float dart_area_ratio;
    float dart_heigth_min;
    float dart_heigth_max;

    float blob_area_min;
    float blob_area_max;
    float blob_heigth_div_width_min;
    float blob_heigth_div_width_max;

    std::string apriltag_family;
    float tag_size;



}

void radar::radar_init()
{

    cv::FileStorage file_1("../cfg/radar_cam_para_cfg.yml", cv::FileStorage::READ);
    if(file_1.isOpened())
    {
        std::cout << "try to read radar camera configuration parameter" << std::endl;

        file_1["leftCameraMatrix"] >> radar::cv_left_camera_matrix;
        file_1["leftDistCoeffs"] >> radar::cv_left_dist_coeffs;

        file_1["rightCameraMatrix"] >> radar::cv_right_camera_matrix;
        file_1["rightDistCoeffs"] >> radar::cv_right_dist_coeffs;

        file_1["left_rotation"] >> radar::cv_left_rotation;
        file_1["left_translation"] >> radar::cv_left_translation;

        file_1["right_rotation"] >> radar::cv_right_rotation;
        file_1["right_translation"] >> radar::cv_right_translation;

        file_1["transition_matrix"] >> radar::transition_matrix;  //卡尔曼的转移矩阵

        file_1.release();
    }
    else
    {
        std::cerr << "fail to read armor camera configuration parameter" << std::endl;
        return;
    }
    std::cout << "succed to read armor camera configuration parameter" << std::endl;


    cv::FileStorage file_2("../cfg/radar.yml", cv::FileStorage::READ);
    if (file_2.isOpened())
    {

        //file读取有两种方法
        std::cout << "try to read radar detect configuration parameter" << std::endl;
        radar::enemy_color = (int)file_2["enemy_color"];
        radar::dart_min_area=(int)file_2["dart_min_area"];
        radar::dart_max_area=(int)file_2["dart_max_area"];
        radar::dart_heigth_div_width_min=(float)file_2["dart_heigth_div_width_min"];
        radar::dart_heigth_div_width_max=(float)file_2["dart_heigth_div_width_max"];
        radar::dart_area_ratio=(float)file_2["dart_area_ratio"];
        radar::dart_heigth_min=(float)file_2["dart_heigth_min"];
        radar::dart_heigth_max=(float)file_2["dart_heigth_max"];

        radar::blob_area_min=(int)file_2["blob_area_min"];
        radar::blob_area_max=(int)file_2["blob_area_max"];
        radar::blob_heigth_div_width_min=(float)file_2["blob_heigth_div_width_min"];
        radar::blob_heigth_div_width_max=(float)file_2["blob_heigth_div_width_max"];

        radar::apriltag_family = (std::string)file_2["apriltag_family"];
        radar::tag_size = (float)file_2["tag_size"];



        file_2.release();

        radar::KNN = cv::createBackgroundSubtractorKNN();   //创建 Background Subtractor objects

    }else
    {
        std::cerr << "fail to read radar detect configuration parameter" << std::endl;
        return;
    }
    std::cout << "succed to read radar detect configuration parameter" << std::endl;

    std::cout << radar::apriltag_family  << std::endl;
}