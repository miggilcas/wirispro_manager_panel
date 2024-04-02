/**
 * visiblethread.cpp
 * 
 * This file will be used to implement VideoStreaming in its final purpose.
 * 
 * At first, we will implement a simple opencv video viewer.
*/

#include "wirispro_manager_panel/visiblethread.h"

namespace wirispro_manager_panel {



void VisibleThread::receiveStartStreaming(void)
{
    bool thread_running = false;

    ROS_INFO("StartStreaming signal received");
    if (!thread_running) {
        _visible_thread = std::thread(&VisibleThread::run, this);
        *_stream = true;
    } else
        ROS_WARN("VisibleThread is already running!");
}

void VisibleThread::receiveStopStreaming(void)
{
    ROS_INFO("StopStreaming signal received");
    *_stream = false;
    _visible_thread.join();
    ROS_INFO("VisibleThread stopped");

}

void VisibleThread::run()
{
    ROS_INFO("VisibleThread running :)");
    //cv::VideoCapture cap(_ssrc.toStdString());
    std ::string addr("rtspsrc location=rtsp://" + this->_ssrc.toStdString() + ":8554/visible latency=300 caps = \"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96\" ! rtph264depay ! decodebin ! videoconvert ! appsink");
    ROS_INFO("Address: %s", addr.c_str());
    cv::VideoCapture cap(addr, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        ROS_ERROR("Error opening video stream or file");
        *_stream = false;
        return;
    }
    // we will use a QLabel to show the video
    cv::Mat frame;
    cv::Mat frame2;
    while(*_stream) {
        cap >> frame;
        //cap.read(frame);
        // std::cout << frame.size() << std::endl; 
        if (frame.empty()) {
            ROS_ERROR("Empty frame");
            break;
        }
        
        
        // // print properties 
        // // ROS_INFO("Frame size: %d x %d", frame.cols, frame.rows);
        // // ROS_INFO("Frame channels: %d", frame.channels());
        // // ROS_INFO("Frame type: %d", frame.type());
        // // cv::imshow("Frame", frame);
        // // cv::waitKey(0);
        // resize the image to 640x480
        cv::resize(frame, frame, cv::Size(640, 480));
        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
        // std::cout << "1" << std::endl;            
        // std::cout << "2" << std::endl; 

        static uchar *data = nullptr;
        if (!data) {
            data = new uchar[frame.rows * frame.step];
        }

        memcpy(data, frame.data, frame.rows * frame.step);

        streamFrameVisible = QImage(data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888).copy();
        _label->setPixmap(QPixmap::fromImage(streamFrameVisible));
        _label->resize(_label->pixmap()->size());

        
    }
    
    cap.release();
    *_stream = false;
    ROS_INFO("VisibleThread stop running :(");

}
// TBD: Destructor
// TBD: _stream management 
} // namespace wirispro_manager_panel