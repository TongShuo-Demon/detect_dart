#include "startFetch.hpp"

using namespace std;


boost::mutex camera_mutex2,camera_mutex1;
cv::Mat boost_img1,boost_img2,merge_picture;

void myMerge(Mat &img_left,Mat &img_right,Mat &img_merge)
{
    if (img_left.empty() || img_right.empty())
    {
        printf("open error");
        return ;
    }

    Size size(img_left.cols + img_right.cols, MAX(img_left.rows, img_right.rows));


    Mat outImg_left, outImg_right;

    img_merge.create(size, CV_MAKETYPE(img_left.depth(), 3));
    img_merge = Scalar::all(0);
    outImg_left = img_merge(Rect(0, 0, img_left.cols, img_left.rows));
    outImg_right = img_merge(Rect(img_left.cols, 0, img_right.cols, img_right.rows));
//3.将待拷贝图像拷贝到感性趣区域中
    if(img_left.type() == CV_8U)
    {
        cvtColor(img_left, outImg_left, COLOR_BGR2GRAY);
    }else{
        img_left.copyTo(outImg_left);
    }

    if(img_right.type() == CV_8U)
    {
        cvtColor(img_right, outImg_right, COLOR_BGR2GRAY);
    }else{
        img_right.copyTo(outImg_right);
    }


//    imshow("merge", img_merge);
}




bool DahuaCamera::connectCamera(std::string &filename,
                                preProcessFunction preProcess)
{

    //这一部分为读取配置参数
    getCamParaFromYml(filename, DahuaCamera::daHuaPara);
    if (daHuaPara.IsReadYmlSucc != true)
    {
        std::cerr<<"Failed to get the configur file "<<std::endl;
        return false;
    }

    /* 发现设备 */
    CSystem &systemObj = CSystem::getInstance();//获取实例
    TVector<ICameraPtr> vCameraPtrList;
    bool isDiscoverySuccess = systemObj.discovery(vCameraPtrList);
    if (!isDiscoverySuccess)
    {
        std::cout<<"[error-001]: find device fail."<<endl;
        return false;
    }

    printf(" camera number:%d\n", vCameraPtrList.size());

    if (vCameraPtrList.size() == 0 || vCameraPtrList.size() == 1)
    {
        std::cout<<"[error-002]: no or one devices."<<endl;
        return false;
    }


    /* 连接相机 */
    if (!vCameraPtrList[0]->connect())
    {
        std::cout<<"[error-003]: connect camera failed."<<endl;
        return false;
    } else{
        std::cout<<"connect camera success! "<<endl;
    }

    /* 连接相机 2*/
    if (!vCameraPtrList[1]->connect())
    {
        std::cout<<"[error-003]: connect camera2 failed."<<endl;
        return false;
    } else{
        std::cout<<"connect camera2 success! "<<endl;
    }


    //获得相机指针0、2
    DahuaCamera::cameraSptr = vCameraPtrList[0];
    DahuaCamera::cameraSptr2 = vCameraPtrList[1];

    /* 按照配置文件进行设置相机参数 */
    int IsSetCamParaSucc = 0;
    camParaConfig(DahuaCamera::cameraSptr, DahuaCamera::daHuaPara, IsSetCamParaSucc);
    int IsSetCamParaSucc2 = 0;
    camParaConfig(DahuaCamera::cameraSptr2, DahuaCamera::daHuaPara, IsSetCamParaSucc2);

    if(IsSetCamParaSucc != 0 || IsSetCamParaSucc2 != 0  )
    {
        std::cerr<<"[error-004]: failed to set camera para"<<endl;
        DahuaCamera::cameraSptr->disConnect();

        return false;
    }
    else{
        std::cout<<"succeed to set camera para"<<endl;
    }

    setLineTriggerMode(cameraSptr, true);   //上升沿触发
    setLineTriggerMode(cameraSptr2, true);
    setLineDebouncerTimeAbs(cameraSptr, 1);
    setLineDebouncerTimeAbs(cameraSptr2, 1);


    /* 创建流对象 */
    IStreamSourcePtr streamPtr = systemObj.createStreamSource(DahuaCamera::cameraSptr);

    if (NULL == streamPtr)
    {
        std::cerr<<"create stream obj fail."<<endl;
        DahuaCamera::cameraSptr->disConnect();

        return false;
    }

    /* 创建流对象 */
    IStreamSourcePtr streamPtr2 = systemObj.createStreamSource(DahuaCamera::cameraSptr2);

    if (NULL == streamPtr2)
    {
        std::cerr<<"create stream2 obj  fail."<<endl;
        DahuaCamera::cameraSptr2->disConnect();

        return false;
    }

    /* 停止抓图 并休眠1ms */
    streamPtr->stopGrabbing();
    streamPtr2->stopGrabbing();
    usleep(500);
    usleep(500);
    /* 开始取图 */
    bool isStartGrabbingSuccess = streamPtr->startGrabbing();
    if (!isStartGrabbingSuccess)
    {
        std::cerr<<"Start Grabbing fail."<<endl;
        DahuaCamera::cameraSptr->disConnect();
        return false;
    }

   /* 开始取图 */
    bool isStartGrabbingSuccess2 = streamPtr2->startGrabbing();
    if (!isStartGrabbingSuccess2)
    {
        std::cerr<<"Start Grabbing2 fail."<<endl;
        DahuaCamera::cameraSptr2->disConnect();
        return false;
    }

    /*创建取流线程*/
    Dahua::Memory::TSharedPtr<StreamRetrieve> streamThreadSptr(new StreamRetrieve(cameraSptr,    //相机指针
                                                                                   daHuaPara,    //大华相机参数
                                                                                   streamPtr,
                                                                                   boost::bind(DahuaCamera::callback,_1),
                                                                                   preProcess));

    /*创建取流线程2*/
    Dahua::Memory::TSharedPtr<StreamRetrieve> streamThreadSptr2(new StreamRetrieve(cameraSptr2, //相机指针
                                                                                  daHuaPara,    //大华相机参数
                                                                                  streamPtr2,
                                                                                  boost::bind(DahuaCamera::callback2,_1),
                                                                                  preProcess));

    if (NULL == streamThreadSptr || NULL == streamThreadSptr2 )
    {
        printf("create thread obj failed.\n");
        streamPtr->stopGrabbing();
        DahuaCamera::cameraSptr->disConnect();
        streamPtr2->stopGrabbing();
        DahuaCamera::cameraSptr2->disConnect();
        return false;
    }
    while(1){

        camera_mutex1.lock();

        if(boost_img1.empty()==0 && boost_img2.empty()==0) {
            myMerge(boost_img1, boost_img2, merge_picture);  //我自己（demon）写的拼接图像
            static publisher pub("camera_pub",merge_picture);
            pub.braodcast(merge_picture);

            boost_img1.release();
            boost_img2.release();
        }

        camera_mutex1.unlock();
    }

    streamThreadSptr->join();             //等待该线程完成
    streamThreadSptr2->join();             //等待该线程完成
    return true;
}



void DahuaCamera::callback2(cv::Mat& img2)
{

//   camera_mutex2.lock();
    if(img2.empty() )
    {
        return;
    }
    printf("get image2.\n");

    boost_img2= img2.clone();
//    std::cout << boost_img2.empty()<<std::endl;
//    camera_mutex2.unlock();
}

void DahuaCamera::callback(cv::Mat& img)
{

//    camera_mutex2.lock();
    if(img.empty() )
    {
        return;
    }
    printf("get image.\n");

    boost_img1= img.clone();
//    camera_mutex2.unlock();
}




/*

void DahuaCamera::callback(cv::Mat& img)
{
//    camera_mutex2.lock();

    if(img.empty() )
    {

        return;
    }
    printf("get image.\n");
    static publisher pub("camera_pub",img);
    pub.braodcast(img);
//    camera_mutex2.unlock();
}

void DahuaCamera::callback2(cv::Mat& img2)
{
//    camera_mutex2.lock();

    if(img2.empty() )
    {

        return;
    }
    printf("get image2.\n");

    static publisher pub2("camera_pub2",img2);
    pub2.braodcast(img2);

//    camera_mutex2.unlock();
}

*/




//默认的预处理函数,勿动
void DahuaCamera::nothing(cv::Mat &in, cv::Mat &out) {
    out = in;
}