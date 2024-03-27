#include "wirispro_manager_panel/QStreamer.h"

#include <QDateTime>


namespace wirispro_manager_panel {

/// QStreamer class implementation
QStreamer::QStreamer(QObject *parent) : QObject(parent) {
    bool* _stream;
    // Showing a video in a QLabel
    cv::VideoCapture cap("/home/user/cat.gif");
    if(!cap.isOpened()){
        QMessageBox::information(this, "", "error: Video not loaded "); // show error message
        *_stream = false;
    }
    ROS_INFO("Video loaded successfully");
    cv::Mat cvframe;
    QImage Qframe;
    
    while(*_stream){
        ROS_INFO("Showing video");
        cap >> cvframe;
        if(cvframe.empty())
        {   
            qDebug() << "visible: empty frame occurance, stopping stream!";
            break;}
        cv::cvtColor(cvframe, cvframe,cv::ColorConversionCodes::COLOR_BGR2RGB);
        static uchar *data = nullptr;
        if (!data) {
            data = new uchar[cvframe.rows * cvframe.step];
        }
        memcpy(data, cvframe.data, cvframe.rows * cvframe.step);
        Qframe = QImage(data, cvframe.cols, cvframe.rows, cvframe.step, QImage::Format_RGB888).copy();
        _ui->stream_label->setPixmap(QPixmap::fromImage(Qframe));
        //_ui->stream_label->setScaledContents(true);


        }
        cap.release();
        *_stream = false;
    

    
}

QStreamer::~QStreamer() {
    qDebug() << __FUNCTION__ << "deallocations" << m_track.reallocs;
}

void QStreamer::setImage(const QImage &img) {
    m_img = img;
    // Set the image pixmap to the label
    m_label->setPixmap(QPixmap::fromImage(m_img));
    ROS_INFO("Image set");
}

void QStreamer::setLabel(QLabel *lbl) {
    m_label = lbl;
}
QImage QStreamer::image() const {
    return m_img;
}



/// Capture class implementation
Capture::Capture(QObject *parent) : QObject(parent) {
    // Constructor implementation (if any)
}

Capture::~Capture() {
    qDebug() << __FUNCTION__ << "reallocations" << m_track.reallocs;
    // Destructor implementation (if any)
}

void Capture::start(int cam) {
    cv::VideoCapture cap(cam);

    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open camera %d", cam);
        return;
    }
    if (cap.isOpened()) {
        m_timer.start(0, this);
        // Print if everything is ok
    

        ROS_WARN("Capture started with camera %d", cam);
        

        Q_EMIT started();
    }

    // if (!m_videoCapture)
    //     //m_videoCapture.reset(new cv::VideoCapture("v4l2src device=/dev/video0 ! video/x-raw,framerate=30/1,width=1280,height=720 ! xvimagesink", cv::CAP_GSTREAMER));
    //     m_videoCapture.reset(new cv::VideoCapture("/home/user/meme.mp4"));
    // if (!m_videoCapture->isOpened()) {
    //     ROS_ERROR("Failed to open camera %d", cam);
    //     return;
    // }
    // if (m_videoCapture->isOpened()) {
    //     m_timer.start(0, this);
    //     // Print if everything is ok
    

    //     ROS_WARN("Capture started with camera %d", cam);
        

    //     Q_EMIT started();
    // }
}

void Capture::stop() {
    ROS_INFO("Capture stopped");
    m_timer.stop();
}

cv::Mat Capture::frame() const {
    return m_frame;
}

void Capture::timerEvent(QTimerEvent *ev) {
    if (ev->timerId() != m_timer.timerId()) return;
    if (!m_videoCapture->read(m_frame)) { // Blocks until a new frame is ready
        ROS_INFO("Capture timer blocking");
        m_timer.stop();
        return;
    }
    m_track.track(m_frame);
    ROS_WARN("Frame ready my man");
    Q_EMIT frameReady(m_frame);
}

/// Converter class implementation
Converter::Converter(QObject *parent) : QObject(parent) {
    // Constructor implementation (if any)
}

Converter::~Converter() {
    qDebug() << __FUNCTION__ << "reallocations" << m_track.reallocs;
    // Destructor implementation (if any)
}

bool Converter::processAll() const {
    return m_processAll;
}

void Converter::setProcessAll(bool all) {
    m_processAll = all;
}

QImage Converter::image() const {
    return m_image;
}

void Converter::processFrame(const cv::Mat &frame) {
    if (m_processAll)
        process(frame);
    else
        queue(frame);
}

void Converter::timerEvent(QTimerEvent *ev) {
    if (ev->timerId() != m_timer.timerId()) return;
    process(m_frame);
    m_frame.release();
    // m_track.track(m_frame);
    m_timer.stop();
}

void Converter::queue(const cv::Mat &frame) {
    if (!m_frame.empty()) qDebug() << "Converter dropped frame!";
    m_frame = frame;
    if (!m_timer.isActive()) m_timer.start(0, this);
}

void Converter::process(const cv::Mat &frame) {
    Q_ASSERT(frame.type() == CV_8UC3);
    int w = frame.cols / 3.0, h = frame.rows / 3.0;
    if (m_image.size() != QSize{w, h})
        m_image = QImage(w, h, QImage::Format_RGB888);
    cv::Mat mat(h, w, CV_8UC3, m_image.bits(), m_image.bytesPerLine());
    cv::resize(frame, mat, mat.size(), 0, 0, cv::INTER_AREA);
    cv::cvtColor(mat, mat, cv::COLOR_BGR2RGB);
    ROS_WARN("Image processed and ready buddy");
    Q_EMIT imageReady(m_image);
}

} // namespace wirispro_manager_panel
