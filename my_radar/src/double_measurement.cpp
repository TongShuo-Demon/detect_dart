//
// Created by demon on 20-5-23.
//
#include "double_measurement.h"
using namespace cv;


DoubleMeasurement::DoubleMeasurement(Eigen::Matrix3f left_intrinsic,Eigen::Matrix<float, 1, 5>  left_distortion,
                                     Eigen::Matrix3f right_intrinsic,Eigen::Matrix<float, 1, 5> right_distortion,
                                     Eigen::Matrix3f left_rotation, Eigen::Vector3f left_translation,
                                     Eigen::Matrix3f right_rotation,Eigen::Vector3f right_translation)
{

    //左相机的内参矩阵与畸变系数
    left_intrinsic_ = left_intrinsic;

    left_distortion_ = left_distortion;   //畸变系数

    //右相机的内参矩阵与畸变系数
    right_intrinsic_ = right_intrinsic;

    right_distortion_ = right_distortion;

    //左相机与世界坐标系的关系，旋转矩阵与平移矩阵，在这里是以左相机的坐标系为世界坐标系
    left_rotation_ = left_rotation;

    left_translation_ = left_translation;

    //右相机与世界坐标系的关系，即旋转矩阵与平移矩阵
    right_rotation_ = right_rotation;
    right_translation_ = right_translation;

    m_left_transform_ << left_rotation_, left_translation_;  //拼接变换矩阵
    m_right_transform_ << right_rotation_,right_translation_;


}



//畸变矫正分为两种，分别是图像整体矫正和单个点矫正
//去畸变
cv::Point2f DoubleMeasurement::pixelDistortionCorrection(cv::Point2f &connor,Eigen::Matrix3f cameraMatrix,Eigen::Matrix<float, 5, 1> distCoeffs)
{
    Eigen::Vector3f pixel;
    Eigen::Vector3f P_cam, P_distortion;  //3行1列，distortion畸变

    pixel << connor.x, connor.y, 1;  //像素坐标
    P_cam = cameraMatrix.inverse() * pixel; //inverse是进行逆变换，求出Xc/Zc,即归一化后的坐标,内参矩阵

//    pixel << connor.x, connor.y, 1;
//    P_cam << (connor.x-cameraMatrix(0,2))/cameraMatrix(0,0),(connor.y-cameraMatrix(1,2))/cameraMatrix(1,1),1;

    double r2 = pow(P_cam.x(), 2) + pow(P_cam.y(), 2);//pow是求x的y次方
//公式求出畸变坐标点
    P_distortion.x() = P_cam.x() * ( 1 + distCoeffs[0] * pow( r2, 1 ) + distCoeffs[1] * pow( r2, 2 ) + distCoeffs[2] * pow( r2, 3 ) )
                       + 2 * distCoeffs[3] * P_cam.x() * P_cam.y() + distCoeffs[4] * (r2 + 2 * pow(P_cam.x(), 2 ) );

    P_distortion.y() = P_cam.y() * ( 1 + distCoeffs[0] * pow( r2, 1 ) + distCoeffs[1] * pow( r2, 2 ) + distCoeffs[2] * pow( r2, 3 ) )
                       + distCoeffs[3] * ( r2 + 2 * pow( P_cam.y(), 2 ) ) + 2 * distCoeffs[4] * P_cam.x() * P_cam.y();
    P_distortion.z() = 1;

    cv::Point2f P;  //矫正后的点
//    P.x=P_distortion.x()*cameraMatrix(0,0)+cameraMatrix(0,2);
//    P.y=P_distortion.y()*cameraMatrix(1,1)+cameraMatrix(1,2);
    P.x=P_distortion.x()+connor.x;
    P.y=P_distortion.y()+connor.y;

    return P;
}



//输入左右相机检测到的像素点;
//输出世界坐标（相机下坐标）;
//采用原理就是最小二乘法求解AX=B;
Point3f DoubleMeasurement::pixelToWorldCoordinate(Point2f pixeLeft,Point2f pixelRight)
{
    Eigen::Matrix<float, 4, 3> A;//最小二乘法A矩阵
    Eigen::Matrix<float, 4, 1> B;//最小二乘法B矩阵
    Eigen::Matrix<float, 3, 1> XYZ;//最小二乘法求解XYZ

    Eigen::MatrixXf m_leftM,m_rightM;


    m_leftM = left_intrinsic_ * m_left_transform_;
    m_rightM = right_intrinsic_ * m_right_transform_;


    A(0,0) = pixeLeft.x * m_leftM(2,0) - m_leftM(0,0);
    A(0,1) = pixeLeft.x * m_leftM(2,1) - m_leftM(0,1);
    A(0,2) = pixeLeft.x * m_leftM(2,2) - m_leftM(0,2);

    A(1,0) = pixeLeft.y * m_leftM(2,0) - m_leftM(1,0);
    A(1,1) = pixeLeft.y * m_leftM(2,1) - m_leftM(1,1);
    A(1,2) = pixeLeft.y * m_leftM(2,2) - m_leftM(1,2);

    A(2,0) = pixelRight.x * m_rightM(2,0) - m_rightM(0,0);
    A(2,1) = pixelRight.x * m_rightM(2,1) - m_rightM(0,1);
    A(2,2) = pixelRight.x * m_rightM(2,2) - m_rightM(0,2);

    A(3,0) = pixelRight.y * m_rightM(2,0) - m_rightM(1,0);
    A(3,1) = pixelRight.y * m_rightM(2,1) - m_rightM(1,1);
    A(3,2) = pixelRight.y * m_rightM(2,2) - m_rightM(1,2);


    B(0,0) = m_leftM(0,3) - pixeLeft.x * m_leftM(2,3);
    B(1,0) = m_leftM(1,3) - pixeLeft.y * m_leftM(2,3);
    B(2,0) = m_rightM(0,3) - pixelRight.x * m_rightM(2,3);
    B(3,0) = m_rightM(1,3) - pixelRight.y * m_rightM(2,3);

    XYZ=A.colPivHouseholderQr().solve(B);  //例子中colPivHouseholderQr()方法返回一个类ColPivHouseholderQR的对象
//    XYZ= A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);

    Point3f world;
    world.x = XYZ(0,0);
    world.y = XYZ(1,0);
    world.z = XYZ(2,0);

    return world;

}



//*********************************************以下代码已经废弃*************************************
/*
 * 世界坐标系到图像坐标系
 *
 */
Point2f worldTopixelCoordinate(Point3f world_point,float intrinsic[3][3],float translation[1][3],float rotation[3][3])
{
    //    [fx s x0]                    [Xc]      [Xw]      [u]      1       [Xc]
    //K = |0 fy y0|       TEMP = [R T]      |Yc| = TEMP*|Yw|      | | = —*K *|Yc|
    //    [ 0 0 1 ]                    [Zc]      |Zw|      [v]      Zc   [Zc]
    //                                       [1 ]
    Point3f c;
    c.x = rotation[0][0]*world_point.x + rotation[0][1]*world_point.y + rotation[0][2]*world_point.z + translation[0][0]*1;
    c.y = rotation[1][0]*world_point.x + rotation[1][1]*world_point.y + rotation[1][2]*world_point.z + translation[0][1]*1;
    c.z = rotation[2][0]*world_point.x + rotation[2][1]*world_point.y + rotation[2][2]*world_point.z + translation[0][2]*1;

    Point2f uv;
    uv.x = (intrinsic[0][0]*c.x + intrinsic[0][1]*c.y + intrinsic[0][2]*c.z)/c.z;
    uv.y = (intrinsic[1][0]*c.x + intrinsic[1][1]*c.y + intrinsic[1][2]*c.z)/c.z;

    return uv;
}

// Calculates rotation matrix given euler angles.
Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d &theta) {
    Eigen::Matrix3d R_x, R_y, R_z, R;
    theta << theta[0] * PI / 180, theta[1] * PI / 180, theta[2] * PI / 180;

    R_x << 1, 0, 0,
            0, cos(theta[0]), -sin(theta[0]),
            0, sin(theta[0]), cos(theta[0]);

    R_y << cos(theta[1]), 0, sin(theta[1]),
            0, 1, 0,
            -sin(theta[1]), 0, cos(theta[1]);

    R_z << cos(theta[2]), -sin(theta[2]), 0,
            sin(theta[2]), cos(theta[2]), 0,
            0, 0, 1;

    R = R_z * R_y * R_x;
    return R;
}

//像素与世界坐标系关系
//参考链接：https://www.jianshu.com/p/4566a1281066
Eigen::Vector3d pixelToWorld(Eigen::Matrix3d cameraMatrix, Eigen::Matrix3d rv, Eigen::Vector3d tv,cv::Point connor)
{
    Eigen::Vector3d pixel;
    pixel << connor.x,connor.y,1.0;

    //计算 invR * T
    Eigen::Matrix3d invR;
    Eigen::Vector3d transPlaneToCam,worldPtCam,worldPtPlane,scale_worldPtPlane,world;
    invR=rv.inverse();
    transPlaneToCam =  invR* tv ;
    //[x,y,z] = invK * [u,v,1]
    worldPtCam = cameraMatrix.inverse()*pixel;  //相机下面坐标
    //[x,y,1] * invR
    worldPtPlane = invR * worldPtCam;
    //zc,存在疑问
    float scale = transPlaneToCam[2] / worldPtPlane[2];
    //zc * [x,y,1] * invR
    scale_worldPtPlane = scale * worldPtPlane;
    //[X,Y,Z]=zc*[x,y,1]*invR - invR*T
    world = scale_worldPtPlane - transPlaneToCam;
    world << world[0]+2,world[1],0;           //补偿结果
//    std::cout << "world:"<<world;
//    std::cout << "pixel:"<<pixel<<std::endl<<std::endl;
    return world;
}


//相机与世界坐标系关系
//假定相机90度平行于地面
void DoubleMeasurement::cameraToWorld(Eigen::Vector3d camera_point,Eigen::Vector3d &world_point)
{
    Eigen::Matrix3d R_x;
    Eigen::Vector3d t;
    world_point<< 0,0,0;

    R_x <<  1, 0, 0,
            0, cos(-PI/2), -sin(-PI/2),
            0, sin(-PI/2), cos(-PI/2);

    std::cout <<"为:"<<std::endl<<camera_point<<std::endl;
    std::cout <<" R_x:"<<std::endl<< R_x<<std::endl;
    t << 2,0,3;

    world_point  = R_x*camera_point+t;

//    world_point=R_x.inverse()*(camera_point-t);

}





