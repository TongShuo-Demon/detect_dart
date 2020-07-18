//
// Created by demon on 20-7-7.
//

#ifndef RADAR_SERIAL_PORT_H
#define RADAR_SERIAL_PORT_H

#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <opencv2/opencv.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <opencv2/opencv.hpp>

namespace cubot{

    typedef boost::function<void(const uint8_t *)> sp_callback_;       //指向一个有效函数

    class SerialPort{
    public:

        SerialPort(sp_callback_ cb, const std::string& sp_dev, const int &baud_rate);
        ~SerialPort();

        int SerialPublish(const cv::Point3f &connors);
        void spJoin();

    private:
        boost::system::error_code  error_sp;
        boost::asio::io_service    io_sp;  //io_service对象是使用boost::asio库的必需要有的对象。
        boost::asio::serial_port   sp;
        boost::thread              t0;
        sp_callback_               sp_callback;

        uint8_t                    recv_buf[128] = {0};
        void read();
        std::size_t write_sync(uint8_t *send_buf, int send_size);
    };
}




#endif //RADAR_SERIAL_PORT_H
