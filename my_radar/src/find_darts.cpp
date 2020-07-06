//
// Created by demon on 20-5-23.
//
#include"find_darts.h"

// 腐蚀膨胀二值化
cv::Mat DartsDetect::imagePreProcess(cv::Mat &src,uint8_t enemy_color) {

    std::vector<cv::Mat> channels;       // 通道拆分
    cv::Mat color_channel;
    cv::split(src, channels);               //************************
    equalizeHist(channels[0],channels[0]);
    equalizeHist(channels[2],channels[2]);  //直方图均衡化,增加对比度

    if (enemy_color == 0) {
        color_channel = channels[0];        // 根据目标颜色进行通道提取
    } else if (enemy_color == 1) {          //      1，红色
        color_channel = channels[2];        //************************
    }

    int light_threshold;
    if(enemy_color == 0){
        light_threshold = 220;
    }else{
        light_threshold = 200;
    }
    cv::threshold(color_channel,color_channel,light_threshold, 255,cv::THRESH_BINARY); // 二值化对应通道

//    cv::Mat kernel_erode = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
//    erode(color_channel, color_channel, kernel_erode);
//
//    cv::Mat kernel_dilate = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
//    dilate(color_channel, color_channel, kernel_dilate);
//
//    cv::Mat kernel_dilate2 = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
//    dilate(color_channel, color_channel, kernel_dilate2);
//
//    cv::Mat kernel_erode2 = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
//    erode(color_channel, color_channel, kernel_erode2);
//    cv::imshow("erode",color_channel);

    return color_channel;
}

// 旋转矩形的长宽比,因为程序里面采用拟合椭圆的原因，所以长的就是长的
static double lengthDivWidth(const cv::RotatedRect &rect) {

    return rect.size.height / rect.size.width;
}

static bool judge_pxl_persent(cv::Mat bgr_roi,double contours,int color_type)
{

    //对轮廓颜色面积比例进行约束，首先对rgb进行约束，进而对hsv进行约束
    cv::Mat hsv_roi, color_mask_roi;
    cv::cvtColor(bgr_roi, hsv_roi, cv::COLOR_BGR2HSV);

    if(color_type == RED)
    {
        cv::Mat hsv1, hsv2;
        cv::inRange(hsv_roi, cv::Scalar(0, 43, 46), cv::Scalar(50, 255, 255), hsv1);
        cv::inRange(hsv_roi, cv::Scalar(156, 43, 46), cv::Scalar(180, 255, 255), hsv2);
        color_mask_roi = hsv1 + hsv2;
    } else if(color_type == BLUE){

        cv::inRange(hsv_roi, cv::Scalar(100, 100, 100), cv::Scalar(124, 255, 255), color_mask_roi);
    }

    int correct_pxl=0;
    for (int j=0; j < color_mask_roi.rows; j++)
    {
        auto *hsv_ptr = color_mask_roi.ptr<uchar>(j);
        auto *bgr_ptr = bgr_roi.ptr<uchar>(j);

        for (int k = 0; k < color_mask_roi.cols; k++)
        {
            auto hsv_val = hsv_ptr[k];             //第j行，第k列
            auto b = bgr_ptr[k * 3];
            auto r = bgr_ptr[k * 3 + 2];

            if(color_type == RED) {
                if (hsv_val && r - b > 80)
                    correct_pxl++;
            } else if(color_type == BLUE)
            {
                if(hsv_val)
                    correct_pxl++;

            }

        }
    }
    float pxl_persent = (float)correct_pxl /contours;
//    std::cout <<"像素比约束："<< pxl_persent<<std::endl;
    return pxl_persent < 0.09 ;

}

//1、描述：
//判断矩形是否在图像内
//2、输入：
//rect：输入矩形
//rows：行界限
//cols：列界限
//3、输出：
//返回：如果在范围内返回true，否则返回flase
bool check_rect(cv::Rect &rect, int rows, int cols)
{
    cv::Rect big_rect(0, 0, cols, rows);
    cv::Rect and_rect;
    and_rect = big_rect & rect;
    if (and_rect.area() < 1)
        return false;

    return true;
}

//1、描述：
//检查待检测值是否在给定范围内，否则更新待值到给定范围内
//2、输入：
//a：待检测值
//hight：范围上限
//low：范围下限
float limit_border(float a, float hight, float low)
{
    if (a > hight)
        return hight;
    if (a < low)
        return low;

    return a;
}

//1、描述：
//首先检查矩形是否在规定范围内
//如果超出范围则去除超出的范围的矩形
//2、输入：
//rect：输入矩形
//rows：行界限
//cols：列界限
//3、输出：
//返回：成功更新矩阵返回true，否则返回false
bool limit_rect(cv::Rect &rect, int rows, int cols)
{
    if (!check_rect(rect, rows, cols))
        return false;
    else
    {
        int x1, y1, x2, y2;
        x1 = limit_border(rect.x, cols - 1, 0);
        y1 = limit_border(rect.y, rows -1 , 0);
        x2 = limit_border(rect.x + rect.width, cols - 1, 0);
        y2 = limit_border(rect.y + rect.height, rows - 1, 0);

        if ((y2 <= y1) || (x2 <= x1))
            return false;

        rect.x = x1;
        rect.y = y1;
        rect.width = x2 - x1;
        rect.height = y2 - y1;

        if ((rect.width < 0) || (rect.height < 0))
            return false;
    }
    return true;
}

cv::Mat DartsDetect::processVideoKNN(const cv::Mat &src){

    cv::Mat mask_knn,src_gray,thresh_knn; //通过knn方法得到的掩码图像fgmask

    cvtColor(src, src_gray, cv::COLOR_BGR2GRAY);

//    equalizeHist(src_gray,src_gray);          //直方图均衡化


    radar::KNN->apply(src_gray, mask_knn);    //更新背景模型


    threshold(mask_knn, thresh_knn, 50, 255, cv::THRESH_OTSU);//THRESH_BINARY,二值化

    cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::Mat element2 = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(21, 21));
//    morphologyEx(thresh_knn, thresh_knn, cv::MORPH_ERODE, element);//腐蚀、
//    morphologyEx(thresh_knn, thresh_knn, cv::MORPH_DILATE, element2);//膨胀

    morphologyEx(thresh_knn, thresh_knn, cv::MORPH_TOPHAT, element2);//顶帽

//    medianBlur(thresh_knn, thresh_knn, 9);  //中值滤波
    cv::GaussianBlur(thresh_knn,thresh_knn,cv::Size(5,5),0,0);  //高斯滤波
    
    return thresh_knn;
}



//输入的是
//得到所有疑似车辆
bool DartsDetect::extractionDart(const cv::Mat src, RectDarts &move_darts){

    cv::Mat two_value_image,src_copy;
    move_darts.clear();
    src_copy=src.clone();

    std::vector<std::vector<cv::Point> > contours;  //轮廓
    std::vector<cv::Vec4i> hierarchy;               //轮廓等级关系

    two_value_image=processVideoKNN(src_copy);    //预处理

    findContours(two_value_image, contours, hierarchy,
                 cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE,cv::Point(0,0)); //检测最外围轮廓

    cv::RotatedRect min_ellipse;
#ifdef write_car
    //筛选所有疑似车辆
    static int tt=0;
#endif
    if (contours.size() > 0)
    {

        for (int i = 0; i < contours.size(); i++) {

            min_ellipse = fitEllipse(contours[i]);                  //椭圆拟合

            //!面积筛选
            if (radar::dart_min_area > contourArea(contours[i]) ||
                contourArea(contours[i]) > radar::dart_max_area ){
#ifdef show_move
                std::cout << "运动筛选：面积筛选未通过:"<< contourArea(contours[i]) << std::endl;
#endif
                continue;
            }

            //!轮廓面积和外接矩形面积筛选
            float areaRatio = contourArea(contours[i])/min_ellipse.size.area();
            if (areaRatio < radar::dart_area_ratio){
#ifdef show_move
                std::cout << "运动筛选：面积比值筛选未通过:" << areaRatio<< std::endl;
#endif
                continue;
            }

            //!长宽比值筛选
            float dart_heigth_div_width = (float)min_ellipse.size.height / min_ellipse.size.width;
            if (radar::dart_heigth_div_width_min >  dart_heigth_div_width ||
                dart_heigth_div_width > radar::dart_heigth_div_width_max ){
#ifdef show_move
                std::cout << "运动筛选：长宽比值筛选未通过:"<< vehicle_heigth_div_width << std::endl;
#endif
                continue;
            }
            //!长和宽长度
            if ( min_ellipse.size.height < radar::dart_heigth_min || min_ellipse.size.height > radar::dart_heigth_max){
#ifdef show_move
                std::cout << "运动筛选：长宽的长度未通过:"<< vehicle_heigth_div_width << std::endl;
#endif
                continue;
            }
            cv::Rect min_rect;
            min_rect = min_ellipse.boundingRect();
            //!限制轮廓外接矩形roi在图像内部
            if (!limit_rect(min_rect, src_copy.rows, src_copy.cols )) {
#ifdef show_move
                std::cout << "运动筛选：超出界限:" << std::endl;
#endif
                continue;
            }
            //裁剪感兴趣区域
            cv::Mat bgr_roi;
            bgr_roi = src_copy(min_rect);
#ifdef write_car
            imwrite(intToString(tt)+"_"+intToString(i)+"_.png",bgr_roi);//采集图片
            tt++;
#endif
            move_darts.emplace_back(min_rect);
#ifdef show_move
            cv::rectangle(src_copy,min_rect,cv::Scalar(0, 0, 255), 2, 8, 0);//绘制矩形框
#endif
        }
    }

#ifdef show_move
    cv::imshow("二值化后图像",two_value_image);
    cv::imshow("相机原图",src_copy);
#endif

    return !move_darts.empty(); //如果为空返回1

}

//输入图片
//判断颜色,输出1为红色，否则蓝色
static int usingHSV(const cv::Mat &bgr_roi)
{
    //对轮廓颜色面积比例进行约束，首先对rgb进行约束，进而对hsv进行约束
    cv::Mat hsv_roi, color_mask_roi1,color_mask_roi2;
    cv::cvtColor(bgr_roi, hsv_roi, cv::COLOR_BGR2HSV);

    cv::Mat hsv1, hsv2;
    cv::inRange(hsv_roi, cv::Scalar(0, 43, 46), cv::Scalar(10, 255, 255), hsv1);  //红色
    cv::inRange(hsv_roi, cv::Scalar(156, 43, 46), cv::Scalar(180, 255, 255), hsv2);
    color_mask_roi1 = hsv1 + hsv2;

    cv::inRange(hsv_roi, cv::Scalar(100, 43, 46), cv::Scalar(124, 255, 255), color_mask_roi2);//蓝色

    int correct_pxl1=0,correct_pxl2=0;

    for (int j=0; j < color_mask_roi1.rows; j++)
    {
        auto *hsv_ptr1 = color_mask_roi1.ptr<uchar>(j);
        auto *bgr_ptr1 = bgr_roi.ptr<uchar>(j);
        for (int k = 0; k < color_mask_roi1.cols; k++)
        {
            auto hsv_val1 = hsv_ptr1[k];             //第j行，第k列
            auto b1 = bgr_ptr1[k * 3];
            auto r1 = bgr_ptr1[k * 3 + 2];

            if (hsv_val1 && r1 - b1 > 0)
                correct_pxl1++;

        }
    }

    for (int j=0; j < color_mask_roi2.rows; j++)
    {
        auto *hsv_ptr2 = color_mask_roi2.ptr<uchar>(j);
        auto *bgr_ptr2 = bgr_roi.ptr<uchar>(j);
        for (int k = 0; k < color_mask_roi2.cols; k++)
        {
            auto hsv_val2 = hsv_ptr2[k];             //第j行，第k列
            auto b2 = bgr_ptr2[k * 3];
            auto r2 = bgr_ptr2[k * 3 + 2];
            if(hsv_val2 && r2-b2 < 0)
                correct_pxl2++;


        }
    }
 //   std::cout << "correct_pxl1:"<<correct_pxl1 << " _correct_pxl2:"<<correct_pxl2 << std::endl;
    if(correct_pxl1 < 40 && correct_pxl2 < 40)
    { return -1;}
    return correct_pxl1 > correct_pxl2;
}

//输入原图
//
//输出图像坐标系
//
bool DartsDetect::findDarts(cv::Mat src,darts &move_darts,int color_type)
{

    cv::Mat src_gray;

    src_gray=imagePreProcess(src,color_type);

    cv::Mat contours_img(src.size() , CV_8UC1, cv::Scalar(0));
    cv::Mat temp_contours_img(src.size() , CV_8UC1, cv::Scalar(0));
    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point>> contours;
//    std::vector<std::vector<cv::Point>> temp_contours;
    //只检测最外围轮廓
    cv::findContours(src_gray, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));

    if (contours.size() > 0) {

        std::vector<Dart> temp_darts(contours.size());

        for (int i = 0; i < contours.size(); i++) {

            cv::RotatedRect min_ellipse;
            cv::Rect min_rect;
            cv::Mat min_rect_roi;

            if (contours[i].size() <= 6)         //第i个轮廓的所有像素点数目
                continue;
            else if (contours[i].size() > 6) {

                min_ellipse = fitEllipse(contours[i]);                  //椭圆拟合
                //长和宽的比值进项限制
                double ellipse_h_w_div=lengthDivWidth(min_ellipse);
                if(ellipse_h_w_div > radar::blob_heigth_div_width_max ||
                   ellipse_h_w_div < radar::blob_heigth_div_width_min){
#ifdef show_blob
                    std::cout << "筛选灯条：changkuan:" << ellipse_h_w_div << std::endl;
#endif
                    continue;
                }
                //!面积筛选
                if (radar::blob_area_min > contourArea(contours[i]) ||
                    contourArea(contours[i]) > radar::blob_area_max ){
#ifdef show_blob
                    std::cout << "筛选灯条：mianji"<< contourArea(contours[i]) << std::endl;
#endif
                    continue;
                }

                cv::Mat bgr_roi;
                min_rect = min_ellipse.boundingRect();                  //最小外接矩形

                //判断是否超出边界
                if (!limit_rect( min_rect, src.rows,src.cols ))
                {
#ifdef show_blob
                    std::cout << "筛选灯条：超出边界"<< std::endl;
#endif
                    continue;
                }

                bgr_roi = src(min_rect);

                if(judge_pxl_persent(bgr_roi,contourArea(contours[i]),color_type))
                {
#ifdef show_blob
                    std::cout << "像素筛选未通过" << std::endl;
#endif
                    continue;
                }
            }

//            temp_contours.emplace_back(contours[i]);

            temp_darts[i].dart_center=min_ellipse.center;
            temp_darts[i].dart_color=color_type;

        }
        for (int i = 0; i < temp_darts.size(); i++)
        {
            if(temp_darts[i].dart_center.x!=0)
                move_darts.emplace_back(temp_darts[i]);
        }
#ifdef show_blob
 //       cv::drawContours(src, contours, -1, cv::Scalar(255), 1, 8);
//      cv::drawContours(temp_contours_img, temp_contours, -1, cv::Scalar(255), 1, 8);
//      cv::imshow("筛选灯条", temp_contours_img);
//      cv::imshow("未筛选灯条", src);
//       cv::drawContours(temp_contours_img, temp_contours, -1, cv::Scalar(255), 1, 8);
//        cv::imshow("筛选灯条", temp_contours_img);
#endif
    }

return move_darts.size() == 1;
}


cv::Point2f DartsDetect::findOneDetect(cv::Mat src,std::string camera,float &S)
{


    RectDarts darts_;
    int color_type=-1;
    cv::Mat temp_car,point_src;
    darts move_darts;

    int  number=0;
    point_src=src.clone();

    if(extractionDart(point_src,darts_))       //检测运动物体
    {


        for (int i = 0; i < darts_.size(); i++)
        {
            cv::Mat src3=point_src(darts_[i]);
            color_type=usingHSV(src3);         //得到颜色

            if(color_type!=radar::enemy_color)              //敌方颜色
            {
//                std::cout << "不是敌方颜色" << std::endl;
                color_type=-1;
                continue;
            }
            int judge_number_darts =findDarts(src3,move_darts,color_type);
//            cv::rectangle(point_src,darts_[i],cv::Scalar(0, 255, 0), 2, 8, 0);//绘制矩形框
//            cv::imshow("dddd",point_src);

            number =number+judge_number_darts;

        }
//        std::cout << darts_.size() << camera+"：" << number << std::endl <<number <<std::endl ;
        if(number==1)
        {
            number=0;
            cv::rectangle(src,darts_[0],cv::Scalar(0, 0, 255), 2, 8, 0);//绘制矩形框
//            imshow("srcc"+camera,src);
//                cv::putText(src3, std::to_string(color_type),cv::Point(15,15),cv::FONT_HERSHEY_PLAIN,2,cv::Scalar(0, 255, 255), 2, 8, 0);
//                imshow("src3"+camera,src3);

            cv::Point2f Pixel;
            Pixel.x=darts_[0].tl().x+move_darts[0].dart_center.x;
            Pixel.y=darts_[0].tl().y+move_darts[0].dart_center.y;

            int length = darts_[0].height >= darts_[0].width ? darts_[0].width:darts_[0].height;
            S = length/0.12;

//            std::cout << "检测到飞镖头"<< Pixel <<std::endl;
            return Pixel;
        }

    }


    return cv::Point2f(0,0);   //没有检索到运动物体

}

bool DartsDetect::matchDoubleDarts(cv::Mat src_left,cv::Mat src_right, cv::Point2f &uv_left,cv::Point2f &uv_right,float &K)
{

//    cv::Point2f uv_left,uv_right;
    float left_a,right_b;


    uv_left=findOneDetect(src_left,"left",left_a);
    if(uv_left.x==0 && uv_left.y==0){
//            std::cout << "左图未检测到" << std::endl;
        return 0;
    }
    uv_right=findOneDetect(src_right,"right",right_b);
    if(uv_right.x==0 && uv_right.y==0){
//            std::cout << "右图未检测到" << std::endl;
        return 0;
    }

    if((abs(uv_left.x-uv_right.x) > 300) || (abs(uv_right.y-uv_left.y) > 300))
    {
//            std::cout << "匹配点不对" << std::endl;

        return 0;
    }
    if(uv_left.x < 600 && uv_left.y > 600)
    {
//            std::cout << "位置不对" << std::endl;

        return 0;
    }

    K=(left_a+right_b)/2;
    return 1;
}









//
//以下部分是为了应对没有实际飞镖飞行的视频的情况下，验证双目的坐标准不准问题的
//

cv::Mat imagePreProcessTest(cv::Mat &src,uint8_t enemy_color) {

    std::vector<cv::Mat> channels;       // 通道拆分
    cv::Mat color_channel;
    cv::split(src, channels);               /************************/
    if (enemy_color == 0) {         /*                      */
        color_channel = channels[0];        /* 根据目标颜色进行通道提取 */
    } else if (enemy_color == 1) {    /*        1，红色              */
        color_channel = channels[2];        /************************/
    }

    int light_threshold;
    if(enemy_color == 0){
        light_threshold = 220;
    }else{
        light_threshold = 200;
    }
    cv::threshold(color_channel,color_channel,light_threshold, 255,cv::THRESH_BINARY); // 二值化对应通道

//    static cv::Mat kernel_erode = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
//    erode(color_channel, color_channel, kernel_erode);
//
//    static cv::Mat kernel_dilate = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
//    dilate(color_channel, color_channel, kernel_dilate);
//
//    static cv::Mat kernel_dilate2 = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
//    dilate(color_channel, color_channel, kernel_dilate2);
//
//    static cv::Mat kernel_erode2 = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
//    erode(color_channel, color_channel, kernel_erode2);

    return color_channel;
}

void findDartsTest(cv::Mat src,int color_type,cv::Point2f &uv_pixel)
{

    cv::Mat src_gray;


    src_gray=imagePreProcessTest(src,color_type);

    cv::Mat contours_img(src.size() , CV_8UC1, cv::Scalar(0));
    cv::Mat temp_contours_img(src.size() , CV_8UC1, cv::Scalar(0));
    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<std::vector<cv::Point>> temp_contours;
    //只检测最外围轮廓
    cv::findContours(src_gray, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));

    if (contours.size() > 0) {

        std::vector<Dart> temp_darts(contours.size());

        for (int i = 0; i < contours.size(); i++) {

            cv::RotatedRect min_ellipse;
            cv::Rect min_rect;
            cv::Mat min_rect_roi;

            if (contours[i].size() <= 6)         //第i个轮廓的所有像素点数目
                continue;
            else if (contours[i].size() > 6) {

                min_ellipse = fitEllipse(contours[i]);                  //椭圆拟合


                //长和宽的比值进项限制
                double ellipse_h_w_div=min_ellipse.size.height / min_ellipse.size.width;
                if(ellipse_h_w_div > 2 ||
                   ellipse_h_w_div < 0.5){

                    continue;
                }
                //!面积筛选
                if (300> contourArea(contours[i]) ||
                    contourArea(contours[i]) > 4000 ){

                    continue;
                }

                cv::Mat bgr_roi;
                min_rect = min_ellipse.boundingRect();                  //最小外接矩形

                //判断是否超出边界
                if (!limit_rect( min_rect, src.rows,src.cols ))
                {

//                    std::cout << "筛选灯条：超出边界"<<std::endl;

                    continue;
                }

                bgr_roi = src(min_rect);

                if(judge_pxl_persent(bgr_roi,contourArea(contours[i]),color_type))
                {
//                    std::cout << "像素筛选未通过" << std::endl;
                    continue;
                }
            }


            temp_contours.emplace_back(contours[i]);

            temp_darts[i].dart_center=min_ellipse.center;
            temp_darts[i].dart_color=color_type;

        }
        for (int i = 0; i < temp_darts.size(); i++)
        {
            if(temp_darts[i].dart_center.x!=0)
                uv_pixel=temp_darts[i].dart_center;
        }

        cv::drawContours(src, contours, -1, cv::Scalar(255), 1, 8);
//      cv::drawContours(temp_contours_img, temp_contours, -1, cv::Scalar(255), 1, 8);
//      cv::imshow("筛选灯条", temp_contours_img);
//      cv::imshow("未筛选灯条", src);
        cv::drawContours(temp_contours_img, temp_contours, -1, cv::Scalar(255), 1, 8);
        cv::imshow("筛选灯条", temp_contours_img);

    }


}