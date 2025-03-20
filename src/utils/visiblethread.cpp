/**
 * visiblethread.cpp
 *
 * This file will be used to implement VideoStreaming in its final purpose.
 *
 * At first, we will implement a simple opencv video viewer.
 */

#include "wirispro_manager_panel/visiblethread.h"

namespace wirispro_manager_panel {

void VisibleThread::receiveStartStreaming(void) {
  bool thread_running = false; // TBD: check if thread is running properly

  ROS_INFO("StartStreaming signal received");
  if (!thread_running) {
    _visible_thread = std::thread(&VisibleThread::run, this);
    *_stream = true;
  } else
    ROS_WARN("VisibleThread is already running!");
}

void VisibleThread::receiveStopStreaming(void) {
  ROS_INFO("StopStreaming signal received");
  *_stream = false;
  _visible_thread.join();
  ROS_INFO("VisibleThread stopped");
}

void VisibleThread::run() {
  ROS_INFO("VisibleThread running :)");

  // Clear QLabel at the start
  QMetaObject::invokeMethod(_label, "clear");

  std::string addr =
      "rtspsrc location=rtsp://" + this->_ssrc.toStdString() +
      ":8554/visible latency=300 caps=\"application/x-rtp, "
      "media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, "
      "payload=(int)96\" ! rtph264depay ! decodebin ! videoconvert ! appsink";

  ROS_INFO("Address: %s", addr.c_str());

  cv::VideoCapture cap(addr, cv::CAP_GSTREAMER);
  if (!cap.isOpened()) {
    ROS_ERROR("Error opening video stream or file");
    *_stream = false;
    return;
  }

  cv::Mat frame;
  bool process = true;
  while (*_stream) {
    if (!cap.read(frame) || frame.empty()) {
      ROS_ERROR("Empty frame or read error");
      break;
    }
    if (!process) {
      process = true;
      continue;
    }

    // Resize and convert color
    cv::resize(frame, frame, cv::Size(640, 480));
    cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);

    // Using a unique_ptr to manage dynamic memory
    auto data = std::make_unique<uchar[]>(frame.rows * frame.step);
    std::memcpy(data.get(), frame.data, frame.rows * frame.step);

    streamFrameVisible = QImage(data.get(), frame.cols, frame.rows, frame.step,
                                QImage::Format_RGB888);

    // Update QLabel on the main thread
    QMetaObject::invokeMethod(_label,
                              [this, img = std::move(streamFrameVisible)]() {
                                _label->setPixmap(QPixmap::fromImage(img));
                                _label->resize(_label->pixmap()->size());
                              });
    process = false;
  }

  cap.release();
  *_stream = false;
  ROS_INFO("VisibleThread stop running :(");
}

// TBD: Destructor
// TBD: _stream management
} // namespace wirispro_manager_panel
