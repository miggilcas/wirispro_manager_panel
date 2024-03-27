/*
Adaptation of the solution provided by: 
https://stackoverflow.com/questions/21246766/how-to-efficiently-display-opencv-video-in-qt


*/


#pragma once

#include <QObject>
#include <qthread.h>
#include <QString>
#include <qimage.h>
#include <qmutex.h>
#include <QtWidgets>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
// we include ros for debugging purposes
#include <ros/ros.h>

#include <mutex>
#include <thread>



namespace wirispro_manager_panel {
/**
 * @brief AddressTracker.
 * 
 * The AddressTracker is used to track memory reallocations for debugging purposes.
*/
struct AddressTracker {
   const void *address = {};
   int reallocs = 0;
   void track(const cv::Mat &m) { track(m.data); }
   void track(const QImage &img) { track(img.bits()); }
   void track(const void *data) {
      if (data && data != address) {
         address = data;
         reallocs ++;
      }
   }
};
/**
 * @brief QStreamer.
 *
 * This custom QOBject act as a client of a RTSP streaming server
 * onboard the Wiris Pro camera.
 *
 */
class QStreamer : public QObject {
    Q_OBJECT
    Q_PROPERTY(QImage image READ image WRITE setImage USER true)
    // We need a property to specify the QLabel from a ui file to display the image
    Q_PROPERTY(QLabel *label READ label WRITE setLabel)
    AddressTracker m_track;
public:
    QStreamer(QObject *parent = nullptr);
    ~QStreamer();

    Q_SLOT void setImage(const QImage &img);
    Q_SLOT void setLabel(QLabel *lbl);
    QImage image() const;
    QLabel *label() const;

private:
    QImage m_img;
    QLabel *m_label;
     
};



/**
 * @brief Capture
 * 
 * The Capture class fills the internal frame buffer with the captured frame. 
 * It notifies of a frame change. The frame is the user property of the class.
 * 
*/
class Capture : public QObject {
    Q_OBJECT
    Q_PROPERTY(cv::Mat frame READ frame NOTIFY frameReady USER true)
    AddressTracker m_track;
public:
    Capture(QObject *parent = nullptr);
    ~Capture();

    Q_SIGNAL void started();
    Q_SLOT void start(int cam = 0);
    Q_SLOT void stop();
    Q_SIGNAL void frameReady(const cv::Mat &);
    cv::Mat frame() const;

protected:
    void timerEvent(QTimerEvent *ev);

private:
    cv::Mat m_frame;
    QBasicTimer m_timer;
    QScopedPointer<cv::VideoCapture> m_videoCapture;
};

/**
 * @brief Converter
*/

class Converter : public QObject {
    Q_OBJECT
    Q_PROPERTY(QImage image READ image NOTIFY imageReady USER true)
    Q_PROPERTY(bool processAll READ processAll WRITE setProcessAll)
    AddressTracker m_track;
public:
    explicit Converter(QObject *parent = nullptr);
    ~Converter();

    bool processAll() const;
    void setProcessAll(bool all);
    QImage image() const;

    Q_SIGNAL void imageReady(const QImage &);
    Q_SLOT void processFrame(const cv::Mat &frame);

protected:
    void timerEvent(QTimerEvent *ev);

private:
    QBasicTimer m_timer;
    cv::Mat m_frame;
    QImage m_image;
    bool m_processAll = true;

    void queue(const cv::Mat &frame);
    void process(const cv::Mat &frame);
};

} // namespace wirispro_manager_panel