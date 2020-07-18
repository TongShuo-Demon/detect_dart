//
// Created by demon on 20-7-7.
//
#include "serial_port.h"


namespace cubot{

    SerialPort::SerialPort(sp_callback_ cb, const std::string& sp_dev, const int &baud_rate)
            :sp_callback(cb),
             sp(io_sp, sp_dev)         //创建串口对象，传入io_service对象，打开串口
    {
        sp.set_option(boost::asio::serial_port::baud_rate(baud_rate));
        sp.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
        sp.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
        sp.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
        sp.set_option(boost::asio::serial_port::character_size(8));

        t0 = boost::thread(boost::bind(&SerialPort::read,this));         //在此开始读取串口数据

        io_sp.run();
    }
    SerialPort::~SerialPort(){
    }

    //描述
    //向串口写数据
    std::size_t SerialPort::write_sync(uint8_t *send_buf, int send_size)
    {
        return sp.write_some(boost::asio::buffer(send_buf, send_size), error_sp);
    }

    void SerialPort::spJoin()
    {
        t0.join();
    }




    //描述
    //通过串口读取数据
    //
    void SerialPort::read() {

//        static shm_publisher_open<int> mode_pub("car_mode");


        int size_recv = 0;

        while (1) {
            size_recv = sp.read_some(boost::asio::buffer(recv_buf, 128), error_sp);

            if (size_recv > 0) {


            }


        }

    }




    //描述
    //向串口发送数据
    //约定格式 0xaa yaw高八位 yaw底八位 pitch高八位 pitch底八位 0xdd
    int SerialPort::SerialPublish(const cv::Point3f &connors)
    {
        if((connors.x==0)&&(connors.y==0)&&(connors.z==0))
            return 0;


        //将角度放大100倍（在下位机解算是需要缩小300倍）
        int  x = int(connors.x*100);
        int  y = int(connors.y*100);
        int  z = int(connors.z*100);

        uint8_t x_h = x>>8;
        uint8_t x_l = x;
        uint8_t y_h = y>>8;
        uint8_t y_l = y;
        uint8_t z_h = y>>8;
        uint8_t z_l = y;

        uint8_t send_buf[8]={0};
        send_buf[0]=0xaa;
        send_buf[1]=x_h;
        send_buf[2]=x_l;
        send_buf[3]=y_h;
        send_buf[4]=y_l;
        send_buf[5]=z_h;
        send_buf[6]=z_l;
        send_buf[7]=0xdd;

        int isSpSucss = write_sync(send_buf,8);
        if(isSpSucss>0)
            std::cout<<" SerialPort publish "<<isSpSucss<<" bytes sucessfully."<<std::endl;
        else
            std::cerr<<"serial port connects failed...\n";
        return isSpSucss;

    }
}