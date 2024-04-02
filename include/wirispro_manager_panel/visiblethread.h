#ifndef CUSTOMTHREAD_H
#define CUSTOMTHREAD_H

#include <QObject>
#include <QThread>
#include <QString>
#include <QImage>
#include <QMutex>
#include <QLabel>
#include <QtGlobal>

#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <memory>
#include "streamthread.h"


namespace wirispro_manager_panel {

class VisibleThread: public StreamThread
{
    Q_OBJECT

    public:
        VisibleThread(bool* stream, const QString & ssrc, QLabel* label) : StreamThread(stream, ssrc), _label(label) 
        {}
    public Q_SLOTS:

        /**
         * @brief Q_SLOT to start streamin the video from the rtsp.
         */
        void receiveStartStreaming(void);

        /**
         * @brief Q_SLOT to stop streaming the video from the rtsp.
         */
        void receiveStopStreaming(void);

    protected:
        virtual void run() override;

        QImage streamFrameVisible;
        std::shared_ptr<QMutex> visibleMutex;
        std::thread _visible_thread;
        // defining the label to show the video, it will be passed as a parameter as _ui->stream_label
        QLabel* _label;

};
} // namespace wirispro_manager_panel

#endif // CUSTOMTHREAD_H