#include <radar_main.hpp>
#include <shm.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "double_measurement.h"
#include "find_darts.h"
#include "switch_function.h"
#include "radar_main.hpp"
#include "apriltag_pose.h"
#include "math.h"
using namespace Eigen;
using namespace cv;


#ifdef CAMERA_GET
subscriber camera_sub("camera_pub");  //定义共享内存，共享内存名称需要与预设的相同
#endif

#ifdef VIDEO_WRITE
cv::Size VideoSize(1280*2,1024);

cv::VideoWriter SaveDartsVideo;            //类

int FRAME_DARTS_FREQ = 1;                 //常量

int g_cnt_Of_dart = 0;                   //全局变量
#endif

double run_time, v_time;
float FPS;
int flag_v=0;



int main()
{
    //相机内参、畸变系数、相机之间的关系
    Eigen::Matrix3f left_intrinsic,left_rotation,right_rotation;
    Eigen::Matrix<float, 5, 1> left_distCoeffs,right_distCoeffs;

    Eigen::Matrix3f right_intrinsic;
    Eigen::Matrix<float, 3, 1> left_translation,right_translation;

    left_intrinsic << 1738.9,0,592,              //左相机 内参
            0,1738.4,504.5,
            0, 0, 1;
    left_distCoeffs<<-0.1539, 0.2329, 0, 0, 0;   //畸变系数


    right_intrinsic << 1716.3,0,708.6,           //右相机内参
            0,1711.9,463.7,
            0, 0, 1;
    right_distCoeffs<<-0.16, 0.1698, 0, 0, 0;


//左相机与世界坐标系的关系，旋转矩阵与平移矩阵，在这里是以左相机的坐标系为世界坐标系
    left_rotation << 1, 0,0,
            0, 1,0,
            0, 0,1;
    left_translation << 0, 0, 0;

//右相机与世界坐标系的关系，即旋转矩阵与平移矩阵
    right_rotation << 0.9935,-0.0179,-0.1123,
                      0.0094,0.9971,-0.0756,
                      0.1133,0.0741,0.9908;
    right_translation << -758.7666,-13.4515,-59.1288;

 //卡尔曼滤波部分
    //1.kalman filter setup
    const int stateNum=6;                                      //状态值5×1向量(x,y,z,v,a)
    const int measureNum=3;                                    //测量值3×1向量(x,y,z)
    KalmanFilter KF(stateNum, measureNum, 0);

//    KF.transitionMatrix = (Mat_<float>(5, 5) << 1,0,0,3,2,
//                                                0,1,0,1,10,
//                                                0,0,1,2,3,
//                                                0,0,0,1,0,
//                                                0,0,0,0,1);  //转移矩阵A
    KF.transitionMatrix = (Mat_<float>(6, 6) << 1,0,0,1,0,0,
                                                0,1,0,0,5,0,
                                                0,0,1,0,0,5,
                                                0,0,0,1,0,0,
                                                0,0,0,0,1,0,
                                                0,0,0,0,0,1);  //转移矩阵A
    setIdentity(KF.measurementMatrix);                                             //测量矩阵H
    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));                            //系统噪声方差矩阵Q
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));                        //测量噪声方差矩阵R
    setIdentity(KF.errorCovPost, Scalar::all(1));                                  //后验错误估计协方差矩阵P
    randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));                      //x(0)初始化
    Mat measurement = Mat::zeros(measureNum, 1, CV_32F);                           //初始测量值x'(0)，因为后面要更新这个值，所以必须先定义
    Mat state(stateNum, 1, CV_32F);                                     //state



    DoubleMeasurement doubleMeasurement(left_intrinsic,left_distCoeffs,right_intrinsic,right_distCoeffs,
            left_rotation,left_translation,right_rotation,right_translation);   //双目测距

    DartsDetect  dartsDetect;              //飞镖检测
    UsingApriltag usingApriltag;           //相机位姿


#ifdef VIDEO_TEST
    cv::VideoCapture cap; //创建VideoCapture对象
    cap.open("/home/demon/CLionProjects/two_camera/video/20200624T212306/realtime_dart.avi"); //20200618T210050
    if(!cap.isOpened()){   //检查是否能正常打开视频文件
        std::cout<<"fail to open video"<<std::endl;
    }
#endif
    cv::Mat frame;
    cv::Point2f  uv_left,uv_right;

    boost::posix_time::ptime time_now(boost::posix_time::second_clock::local_time());
    std::string now_iso_str(to_iso_string(time_now));


#ifdef VIDEO_WRITE
    int video_cnt=0;
    std::string videodirname = "../video/"+now_iso_str+"/";
    const char * videodirnamestr = videodirname.c_str();
    mkdir(videodirnamestr,S_IRUSR | S_IWUSR | S_IXUSR);
    std::string video_dart_name = videodirname+"realtime_dart.avi";

    SaveDartsVideo.open(video_dart_name, cv::VideoWriter::fourcc('M', 'P', '4', '2'), 20, VideoSize);


    while(!SaveDartsVideo.isOpened()){
        video_cnt++;
        SaveDartsVideo.open(video_dart_name,cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 20, VideoSize);
        if(video_cnt>=10){
            break;
        }
    }
    video_cnt=0;
#endif

    radar::radar_init();

    int number_img=0;
    float K,pre_V=0,  acceleration=0;;


    Point2f pre_uv_left; // 计算速度使用



    while(1)
    {
        number_img++;

        run_time = (double)cv::getTickCount();
//        double v_time=run_time;

#ifdef VIDEO_TEST
        cap >>frame;
        if (frame.empty())//如果某帧为空则退出循环
            break;
#endif
        run_time = ((double)cv::getTickCount() - run_time) / cv::getTickFrequency();

#ifdef CAMERA_GET
        camera_sub.get(frame);
#endif

//       cv::imshow("ddd",frame);

#ifdef VIDEO_WRITE
        g_cnt_Of_dart++;
        if(g_cnt_Of_dart%FRAME_DARTS_FREQ == 0)
        {
            g_cnt_Of_dart = 0;
            SaveDartsVideo << frame;
        }
#endif


        cv::Mat right_frame = frame(cv::Rect(0,0,1280,1024));
        cv::Mat left_frame = frame(cv::Rect(1280,0,1280,1024));   //获得左相机图片




//        uv_left=dartsDetect.findOneDetect(left_frame,"left",a);
//        if(uv_left.x==0 && uv_left.y==0){
////            std::cout << "左图未检测到" << std::endl;
//            continue;
//        }
//        uv_right=dartsDetect.findOneDetect(right_frame,"right",b);
//        if(uv_right.x==0 && uv_right.y==0){
////            std::cout << "右图未检测到" << std::endl;
//            continue;
//        }
//
//        if((abs(uv_left.x-uv_right.x) > 300) || (abs(uv_right.y-uv_left.y) > 300))
//        {
////            std::cout << "匹配点不对" << std::endl;
//
//            continue;
//        }
//        if(uv_left.x < 600 && uv_left.y > 600)
//        {
////            std::cout << "位置不对" << std::endl;
//
//            continue;
//        }




         Point3f cv_camera_point;
        if(dartsDetect.matchDoubleDarts(left_frame,right_frame, uv_left,uv_right,K)) {

            std::cout << "一帧图片结束" << std::endl<< std::endl<< std::endl<< std::endl;


           if(flag_v == 0){
              v_time=run_time;
               flag_v++;
           }
            Point2f shift = uv_left-pre_uv_left;
            pre_uv_left=uv_left;
            double radius = cv::sqrt(shift.x*shift.x + shift.y*shift.y);
            float V =  radius/((cv::getTickCount()-v_time)/ cv::getTickFrequency())/K;
            if(V > 17){
                V=18;
            }
            acceleration=(V-pre_V)/((cv::getTickCount()-v_time)/ cv::getTickFrequency());
            if(acceleration > 17){
                acceleration=18;
            }
            v_time = cv::getTickCount();

            pre_V=V;
            std::cout <<"速度：" << V << std::endl<<"加速度："<< acceleration << std::endl;


            //2.预测
            Mat prediction = KF.predict();                       //计算预测值，返回x'
            Point3f predict_pt = Point3f(prediction.at<float>(0),prediction.at<float>(1),prediction.at<float>(2));   //预测值(x',y')


            cv_camera_point = doubleMeasurement.pixelToWorldCoordinate(
                    doubleMeasurement.pixelDistortionCorrection(uv_left, left_intrinsic, left_distCoeffs),
                    doubleMeasurement.pixelDistortionCorrection(uv_right, right_intrinsic, right_distCoeffs));

            Eigen::Vector3f left_camera_point;
            Eigen::Vector3f world_point;
            left_camera_point << cv_camera_point.x / 1000, cv_camera_point.y / 1000, cv_camera_point.z / 1000; //相机坐标系下面的坐标

            //cout << "左相机系坐标为:" << endl << left_camera_point << endl<< endl;

            Eigen::Matrix3f left_rvec;
            Eigen::Vector3f  left_tvec,world_uwb;
            usingApriltag.positionEstimation(left_frame,left_rvec,left_tvec);  //确定相机与标签之间的关系
            usingApriltag.camera2world(left_camera_point,left_rvec,left_tvec,world_uwb);

            //3.update measurement
            measurement.at<float>(0) = (float)world_uwb[0];
            measurement.at<float>(1) = (float)world_uwb[1];
            measurement.at<float>(2) = (float)world_uwb[2];
            measurement.at<float>(3) = (float)V;
            measurement.at<float>(4) = (float)acceleration;

            //4更新
            KF.correct(measurement);


//            cv::putText(frame, std::to_string(KF.statePost.at<float>(0))+" "+std::to_string(KF.statePost.at<float>(1)+2)+" "+std::to_string(KF.statePost.at<float>(2))
//                    +" "+std::to_string(number_img),cv::Point(100,100),cv::FONT_HERSHEY_PLAIN,2,cv::Scalar(0, 255, 255), 2, 8, 0);

            cv::putText(frame, std::to_string(world_uwb[0])+" "+std::to_string(world_uwb[1]+3.4)+" "+std::to_string(world_uwb[2])
                               +" "+std::to_string(number_img),cv::Point(100,100),cv::FONT_HERSHEY_PLAIN,2,cv::Scalar(0, 255, 255), 2, 8, 0);

            cout << "measurement:" << measurement <<endl;
            cout << "predict_pt:" << predict_pt<<endl;
            cout << "KF.state:" << KF.statePost <<endl;

            uv_left.x=0; uv_left.y=0; uv_right.x=0;uv_right.y=0;
        }
        imshow("sss",frame);




        FPS  = 1.0 / run_time;
        cout<<"FPS:"<<FPS<<endl;


        if(10>=0 && waitKey (10)>=32){
            waitKey(0);}

    }

}








