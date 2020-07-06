//
// Created by demon on 20-6-7.
//


#include "apriltag_pose.h"


 UsingApriltag :: UsingApriltag(std::string family,double left_fx,double left_fy,double left_cx,double left_cy)
{
    const char *c_family = family.c_str();  //c_str()返回const char *类型
    getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", c_family, "Tag family to use");
    getopt_add_int(getopt, 't', "threads", "1", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "2.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");

    // Initialize tag detector with options
    tf = NULL;
    famname = getopt_get_string(getopt, "family");   //通过哈希表的对应关系
    if (!strcmp(famname, "tag36h11")) {
        tf = tag36h11_create();
    } else if (!strcmp(famname, "tag25h9")) {
        tf = tag25h9_create();
    } else if (!strcmp(famname, "tag16h5")) {
        tf = tag16h5_create();
    } else if (!strcmp(famname, "tagCircle21h7")) {
        tf = tagCircle21h7_create();
    } else if (!strcmp(famname, "tagCircle49h12")) {
        tf = tagCircle49h12_create();
    } else if (!strcmp(famname, "tagStandard41h12")) {
        tf = tagStandard41h12_create();
    } else if (!strcmp(famname, "tagStandard52h13")) {
        tf = tagStandard52h13_create();
    } else if (!strcmp(famname, "tagCustom48h12")) {
        tf = tagCustom48h12_create();
    } else {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }

    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = getopt_get_int(getopt, "threads");
    td->debug = getopt_get_bool(getopt, "debug");
    td->refine_edges = getopt_get_bool(getopt, "refine-edges");

/***********输入标定的相机参数*************/

    info.tagsize = radar::tag_size; //标识的实际尺寸
    info.fx = 1738.9;
    info.fy = 1738.4;
    info.cx = 592;
    info.cy = 504.5;
//
//     info.fx = 605.0183;
//     info.fy = 604.6563;
//     info.cx = 325.8806;
//     info.cy = 255.1074;

}

 UsingApriltag :: ~UsingApriltag()
{

    apriltag_detector_destroy(td);

    if (!strcmp(famname, "tag36h11")) {
        tag36h11_destroy(tf);
    } else if (!strcmp(famname, "tag25h9")) {
        tag25h9_destroy(tf);
    } else if (!strcmp(famname, "tag16h5")) {
        tag16h5_destroy(tf);
    } else if (!strcmp(famname, "tagCircle21h7")) {
        tagCircle21h7_destroy(tf);
    } else if (!strcmp(famname, "tagCircle49h12")) {
        tagCircle49h12_destroy(tf);
    } else if (!strcmp(famname, "tagStandard41h12")) {
        tagStandard41h12_destroy(tf);
    } else if (!strcmp(famname, "tagStandard52h13")) {
        tagStandard52h13_destroy(tf);
    } else if (!strcmp(famname, "tagCustom48h12")) {
        tagCustom48h12_destroy(tf);
    }
std::cout << "free"<<std::endl;

    getopt_destroy(getopt);

}


//说明：
//输出的是tag坐标系到世界坐标系的旋转矩阵和平移矩阵
void UsingApriltag :: positionEstimation(cv::Mat frame, Eigen::Matrix3f &rvec, Eigen::Matrix<float, 3, 1>  &tvec){

    using namespace std;
    using namespace cv;

    cv::Mat gray;
    cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    // Make an image_u8_t header for the Mat data
    image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
    };

    zarray_t *detections = apriltag_detector_detect(td, &im);

#ifdef apriltag_show
    std::cout << zarray_size(detections) << " tags detected" << std::endl;
#endif

    // Draw detection outlines
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        line(frame, Point(det->p[0][0], det->p[0][1]),
             Point(det->p[1][0], det->p[1][1]),
             Scalar(0, 0xff, 0), 2);
        line(frame, Point(det->p[0][0], det->p[0][1]),
             Point(det->p[3][0], det->p[3][1]),
             Scalar(0, 0, 0xff), 2);
        line(frame, Point(det->p[1][0], det->p[1][1]),
             Point(det->p[2][0], det->p[2][1]),
             Scalar(0xff, 0, 0), 2);
        line(frame, Point(det->p[2][0], det->p[2][1]),
             Point(det->p[3][0], det->p[3][1]),
             Scalar(0xff, 0, 0), 2);

//*********加入位姿估计函数**********

        info.det = det;  //det-->info
        apriltag_pose_t pose;
        //  estimate_pose_for_tag_homography(&info, &pose); //Estimate pose of the tag using the homography method.
        double err = estimate_tag_pose(&info, &pose);

        tvec <<pose.t->data[0],pose.t->data[1],pose.t->data[2];                 //tag相对于相机的位姿

        rvec <<pose.R->data[0],pose.R->data[1],pose.R->data[2],
                pose.R->data[3],pose.R->data[4],pose.R->data[5],
                pose.R->data[6],pose.R->data[7],pose.R->data[8];



#ifdef apriltag_show
        std::cout <<"tvec:"<< std::endl<< tvec << std::endl<<std::endl;
        std::cout <<"rvec:"<< std::endl<< rvec << std::endl<<std::endl;
        stringstream ss;      //转成流
        ss << det->id;
        String text = ss.str();
        int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
        double fontscale = 1.0;
        int baseline;
        Size textsize = getTextSize(text, fontface, fontscale, 2, &baseline);
        putText(frame, text, Point(det->c[0]-textsize.width/2,
                                   det->c[1]+textsize.height/2),
                fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
#endif
    }


}



//输入：相机下面的坐标、变换矩阵
//输出：世界坐标
void UsingApriltag :: camera2world(Eigen::Vector3f P_cam,Eigen::Matrix3f R_cam_tag, Eigen::Vector3f T_cam_tag,Eigen::Vector3f &P_world)
{

    Eigen::Matrix3f R_tag_cam,R_world_tag;
    Eigen::Matrix<float, 3, 1> P_tag,T_tag_cam,T_world_tag;

    //二维码所在坐标系到相机坐标系得到相机位姿  ---->>>>用于作为相机的位姿估计
    R_tag_cam = R_cam_tag.transpose();
    T_tag_cam = -R_cam_tag.transpose()*T_cam_tag;

    //相机坐标系下面的坐标变换到tag下面
    P_tag = R_tag_cam * P_cam + T_tag_cam;

//tag下面坐标变换到世界（uwb）下面坐标
//   R_world_tag << 1,0,0,0,0,-1,0,1,0; //右手系下，绕着x轴正向旋转90度
//    T_world_tag << 6.0648,8.786,0.3675;
//    T_world_tag << 1.6048 , 1.91859,  9.23666;

    R_world_tag << 1,0,0,0,0,1,0,-1,0;
    T_world_tag << 2.0648,8,0.7;

    P_world = R_world_tag*P_tag+T_world_tag;


//#ifdef apriltag_show
    std::cout << "uwb_world：" << P_world << std::endl;
//#endif

//    std::cout << "R：" << R_world_tag* R_tag_cam<< std::endl;
//    std::cout << "T：" << R_world_tag* T_tag_cam+T_world_tag<< std::endl;



}










