//#include <shm.hpp>
//#include <boost/date_time/posix_time/posix_time.hpp>
//#include "double_measurement.h"
//#include "find_darts.h"
//#include "switch_function.h"
//#include "radar_main.hpp"
//#include "apriltag_pose.h"
//#include "math.h"
//#include "serial_port.h"
//#include <opencv2/core/eigen.hpp>
//
//using namespace Eigen;
//using namespace cv;
//
//
//#ifdef CAMERA_GET
//subscriber camera_sub("camera_pub");  //定义共享内存，共享内存名称需要与预设的相同
//#endif
//
//#ifdef VIDEO_WRITE
//cv::Size VideoSize(1280*2,1024);
//
//cv::VideoWriter SaveDartsVideo;            //类
//
//int FRAME_DARTS_FREQ = 1;                 //常量
//
//int g_cnt_Of_dart = 0;                   //全局变量
//#endif
//
////!串口通讯
//#ifdef SERIAL_RUN
//void sp_callback(const uint8_t *recv_buf){}
//cubot::SerialPort sp(sp_callback, "/dev/ttyUSB0", 115200);
//#endif
//
////!日志文件管理
//#ifdef log_output
//std::ofstream FileOut;
//#endif
//
//double run_time, v_time;
//float FPS;
//int flag_v=0;
//int apriltag_flag= 0;
//Eigen::Matrix3f left_rvec;
//Eigen::Vector3f  left_tvec,world_uwb;
//
//
//int main()
//{
//
//    cv::Mat frame;
//    cv::Point2f  uv_left,uv_right;
//    int number_img=0;
//    float K,pre_V=0,  acceleration=0;
//    Point2f pre_uv_left; // 计算速度使用
//
//
//    boost::posix_time::ptime time_now(boost::posix_time::second_clock::local_time());
//    std::string now_iso_str(to_iso_string(time_now));
//
//
//    radar::radar_init();
//
//
//    //相机内参、畸变系数、相机之间的关系
//    Eigen::Matrix3f left_intrinsic,left_rotation,right_rotation;
//    Eigen::Matrix<float, 5, 1> left_distCoeffs,right_distCoeffs;
//
//    Eigen::Matrix3f right_intrinsic;
//    Eigen::Matrix<float, 3, 1> left_translation,right_translation;
//
//    cv2eigen(radar::cv_left_camera_matrix,left_intrinsic);
//    cv2eigen(radar::cv_left_dist_coeffs,left_distCoeffs);
//
//    cv2eigen(radar::cv_right_camera_matrix,right_intrinsic);
//    cv2eigen(radar::cv_right_dist_coeffs,right_distCoeffs);
//    //左相机与世界坐标系的关系，旋转矩阵与平移矩阵，在这里是以左相机的坐标系为世界坐标系
//    cv2eigen(radar::cv_left_rotation,left_rotation);
//    cv2eigen(radar::cv_left_translation,left_translation);
//    //右相机与世界坐标系的关系，即旋转矩阵与平移矩阵
//    cv2eigen(radar::cv_right_rotation,right_rotation);
//    cv2eigen(radar::cv_right_translation,right_translation);
//
//
//    //卡尔曼滤波部分
//    //1.kalman filter setup
//    const int stateNum=6;                                      //状态值5×1向量(x,y,z,dx,dy,dz)
//    const int measureNum=3;                                    //测量值3×1向量(x,y,z)
//    KalmanFilter KF(stateNum, measureNum, 0);                  //创建卡尔曼滤波器对象KF  //状态维数stateNum，测量维数measureNum，没有控制量
//
//    KF.transitionMatrix = radar::transition_matrix;           //转移矩阵A
//
//    setIdentity(KF.measurementMatrix);                        //测量矩阵H
//    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));        //系统噪声方差矩阵Q
//    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));    //测量噪声方差矩阵R
//    setIdentity(KF.errorCovPost, Scalar::all(1));               //后验错误估计协方差矩阵P
//    randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));      //x(0)初始化
//    Mat measurement = Mat::zeros(measureNum, 1, CV_32F);       //初始测量值x'(0)，因为后面要更新这个值，所以必须先定义
//    Mat state(stateNum, 1, CV_32F);                             //state
//
//
//
//    DoubleMeasurement doubleMeasurement(left_intrinsic,left_distCoeffs,right_intrinsic,right_distCoeffs,
//            left_rotation,left_translation,right_rotation,right_translation);   //双目测距
//
//    DartsDetect  dartsDetect;              //飞镖检测
//    UsingApriltag usingApriltag(radar::apriltag_family,left_intrinsic(0,0),left_intrinsic(1,1),
//                                left_intrinsic(0,2),left_intrinsic(1,2));           //相机位姿
//
//#ifdef log_output
//    //创建日志文件
//    std::string LogName = "../log/"+now_iso_str+".log";
//    const char * lognamestr = LogName.c_str();
//    FileOut.open( lognamestr);
//#endif
//
//#ifdef VIDEO_TEST
//    cv::VideoCapture cap; //创建VideoCapture对象
//    cap.open("/home/demon/CLionProjects/two_camera/video/20200624T212306/realtime_dart.avi"); //20200618T210050
//    if(!cap.isOpened()){   //检查是否能正常打开视频文件
//        std::cout<<"fail to open video"<<std::endl;
//    }
//#endif
//
//
//
//#ifdef VIDEO_WRITE
//    int video_cnt=0;
//    std::string videodirname = "../video/"+now_iso_str+"/";
//    const char * videodirnamestr = videodirname.c_str();
//    mkdir(videodirnamestr,S_IRUSR | S_IWUSR | S_IXUSR);
//    std::string video_dart_name = videodirname+"realtime_dart.avi";
//
//    SaveDartsVideo.open(video_dart_name, cv::VideoWriter::fourcc('M', 'P', '4', '2'), 20, VideoSize);
//
//
//    while(!SaveDartsVideo.isOpened()){
//        video_cnt++;
//        SaveDartsVideo.open(video_dart_name,cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 20, VideoSize);
//        if(video_cnt>=10){
//            break;
//        }
//    }
//    video_cnt=0;
//#endif
//
//
//    while(1)
//    {
//
//#ifdef log_output
//        FileOut << "===============come into while====================" << std::endl;
//#endif
//        number_img++;
//        run_time = (double)cv::getTickCount();
//
//
//#ifdef VIDEO_TEST
//        cap >>frame;
//        if (frame.empty())//如果某帧为空则退出循环
//            break;
//#endif
//        run_time = ((double)cv::getTickCount() - run_time) / cv::getTickFrequency();
//
//#ifdef CAMERA_GET
//        camera_sub.get(frame);
//#endif
//
//
//
//#ifdef VIDEO_WRITE
//        g_cnt_Of_dart++;
//        if(g_cnt_Of_dart%FRAME_DARTS_FREQ == 0)
//        {
//            g_cnt_Of_dart = 0;
//            SaveDartsVideo << frame;
//        }
//#endif
//
//
//        cv::Mat right_frame = frame(cv::Rect(0,0,1280,1024));
//        cv::Mat left_frame = frame(cv::Rect(1280,0,1280,1024));   //获得左相机图片
//
//        cv::Mat right_frame_qu = right_frame(cv::Rect(0,0,640,512));
//        cv::Mat left_frame_qu = left_frame(cv::Rect(0,0,640,512));   //获得左相机图片
//         Point3f cv_camera_point;
//
////计算三维坐标
//        if(dartsDetect.matchDoubleDarts(left_frame,right_frame, uv_left,uv_right)) {
//
//            std::cout << "一帧图片结束" << std::endl<< std::endl<< std::endl<< std::endl;
//
//           if(flag_v == 0){
//              v_time=run_time;
//               flag_v++;
//           }
//            Point2f shift = uv_left-pre_uv_left;
//            pre_uv_left=uv_left;
//            double radius = cv::sqrt(shift.x*shift.x + shift.y*shift.y);
//            float V =  radius/((cv::getTickCount()-v_time)/ cv::getTickFrequency())/dartsDetect.K_pixel_physics;
//            if(V > 17){
//                V=18;
//            }
//            acceleration=(V-pre_V)/((cv::getTickCount()-v_time)/ cv::getTickFrequency());
//            if(acceleration > 17){
//                acceleration=18;
//            }
//            v_time = cv::getTickCount();
//
//            pre_V=V;
////            std::cout <<"速度：" << V << std::endl<<"加速度："<< acceleration << std::endl;
//
//            //2.预测
//            Mat prediction = KF.predict();                       //计算预测值，返回x'
//            Point3f predict_pt = Point3f(prediction.at<float>(0),prediction.at<float>(1),prediction.at<float>(2));   //预测值(x',y')
//
//
//            cv_camera_point = doubleMeasurement.pixelToWorldCoordinate(
//                    doubleMeasurement.pixelDistortionCorrection(uv_left, left_intrinsic, left_distCoeffs),
//                    doubleMeasurement.pixelDistortionCorrection(uv_right, right_intrinsic, right_distCoeffs));
//
//            Eigen::Vector3f left_camera_point;
//            Eigen::Vector3f world_point;
//            left_camera_point << cv_camera_point.x / 1000, cv_camera_point.y / 1000, cv_camera_point.z / 1000; //相机坐标系下面的坐标
//
////            cout << "左相机系坐标为:" << endl << left_camera_point << endl<< endl;
//
//            if(apriltag_flag /10==0) {
//                usingApriltag.positionEstimation(left_frame, left_rvec, left_tvec);  //确定相机与标签之间的关系
//                apriltag_flag=0;
//            }
//            apriltag_flag++;
//
//            usingApriltag.camera2world(left_camera_point,left_rvec,left_tvec,world_uwb);
//
//            //3.update measurement
//            measurement.at<float>(0) = (float)world_uwb[0];
//            measurement.at<float>(1) = (float)world_uwb[1]+3;
//            measurement.at<float>(2) = (float)world_uwb[2];
////            measurement.at<float>(3) = (float)V;
////            measurement.at<float>(4) = (float)acceleration;
//
//            //4更新
//            KF.correct(measurement);
//
//
//            cv::putText(frame, std::to_string(KF.statePost.at<float>(0))+" "+std::to_string(KF.statePost.at<float>(1))+" "+std::to_string(KF.statePost.at<float>(2))
//                    +" "+std::to_string(number_img),cv::Point(100,100),cv::FONT_HERSHEY_PLAIN,2,cv::Scalar(0, 255, 255), 2, 8, 0);
//
////            cv::putText(frame, std::to_string(world_uwb[0])+" "+std::to_string(world_uwb[1])+" "+std::to_string(world_uwb[2])
////                               +" "+std::to_string(number_img),cv::Point(100,100),cv::FONT_HERSHEY_PLAIN,2,cv::Scalar(0, 255, 255), 2, 8, 0);
//
////            cout << "measurement:" << measurement <<endl;
////            cout << "predict_pt:" << predict_pt<<endl;
//            cout << "KF.state:" << KF.statePost <<endl;
//
//            cv::Point3f point_3d;
//
//            point_3d.x=KF.statePost.at<float>(0);
//            point_3d.y=KF.statePost.at<float>(1);
//            point_3d.z=KF.statePost.at<float>(2);
//#ifdef SERIAL_RUN
//            //将三维坐标发送至下位机
//
//            int isSpSucss = sp.SerialPublish(point_3d);
//
//            std::cout<<" 已经发送三维坐标:"<<std::endl;
//            cout<<" SerialPort publish "<<isSpSucss<<" bytes sucessfully."<<std::endl;
//#endif
//
//            uv_left.x=0; uv_left.y=0; uv_right.x=0;uv_right.y=0;
//        }
//        imshow("sss",frame);
//
//        FPS  = 1.0 / run_time;
//        cout<<"FPS:"<<FPS<<endl;
//
//
//        if(10>=0 && waitKey (10)>=32){
//            waitKey(0);}
//    }
//
//}



#include <shm.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "double_measurement.h"
#include "find_darts.h"
#include "switch_function.h"
#include "radar_main.hpp"
#include "apriltag_pose.h"
#include "math.h"
#include "serial_port.h"
#include <opencv2/core/eigen.hpp>

using namespace Eigen;
using namespace cv;


int main()
{

    cv::Mat frame;
    cv::Point2f  uv_left,uv_right;
    int number_img=0;
    float K,pre_V=0,  acceleration=0;
    Point2f pre_uv_left; // 计算速度使用


    boost::posix_time::ptime time_now(boost::posix_time::second_clock::local_time());
    std::string now_iso_str(to_iso_string(time_now));


    radar::radar_init();


    //相机内参、畸变系数、相机之间的关系
    Eigen::Matrix3f left_intrinsic,left_rotation,right_rotation;
    Eigen::Matrix<float, 5, 1> left_distCoeffs,right_distCoeffs;

    Eigen::Matrix3f right_intrinsic;
    Eigen::Matrix<float, 3, 1> left_translation,right_translation;

    cv2eigen(radar::cv_left_camera_matrix,left_intrinsic);
    cv2eigen(radar::cv_left_dist_coeffs,left_distCoeffs);

    cv2eigen(radar::cv_right_camera_matrix,right_intrinsic);
    cv2eigen(radar::cv_right_dist_coeffs,right_distCoeffs);
    //左相机与世界坐标系的关系，旋转矩阵与平移矩阵，在这里是以左相机的坐标系为世界坐标系
    cv2eigen(radar::cv_left_rotation,left_rotation);
    cv2eigen(radar::cv_left_translation,left_translation);
    //右相机与世界坐标系的关系，即旋转矩阵与平移矩阵
    cv2eigen(radar::cv_right_rotation,right_rotation);
    cv2eigen(radar::cv_right_translation,right_translation);


    DoubleMeasurement doubleMeasurement(left_intrinsic,left_distCoeffs,right_intrinsic,right_distCoeffs,
            left_rotation,left_translation,right_rotation,right_translation);   //双目测距

    DartsDetect  dartsDetect;              //飞镖检测
    UsingApriltag usingApriltag(radar::apriltag_family,left_intrinsic(0,0),left_intrinsic(1,1),
                                left_intrinsic(0,2),left_intrinsic(1,2));           //相机位姿




    cv::VideoCapture cap; //创建VideoCapture对象
    cap.open("/home/demon/CLionProjects/two_camera/video/20200624T212306/realtime_dart.avi"); //20200618T210050
    if(!cap.isOpened()){   //检查是否能正常打开视频文件
        std::cout<<"fail to open video"<<std::endl;
    }


    while(1) {
        cap >> frame;
        if (frame.empty())//如果某帧为空则退出循环
            break;

        cv::Mat right_frame = frame(cv::Rect(0,0,1280,1024));
        cv::Mat left_frame = frame(cv::Rect(1280,0,1280,1024));   //获得左相机图片

        cv::Mat right_frame_qu = right_frame(cv::Rect(0,0,640,512));
        cv::Mat left_frame_qu = left_frame(cv::Rect(0,0,640,512));   //获得左相机图片
         Point3f cv_camera_point;



        dartsDetect.frameDifferenceMethod(frame);

        waitKey(20);



    }
}


