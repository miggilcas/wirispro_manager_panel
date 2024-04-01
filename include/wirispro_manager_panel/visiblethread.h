#ifndef CUSTOMTHREAD_H
#define CUSTOMTHREAD_H

#include <QObject>
#include <QThread>
#include <QString>
#include <QImage>
#include <QMutex>

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
        VisibleThread(bool* stream, const QString & ssrc) :  StreamThread (stream, ssrc)
        {}
    public Q_SLOTS:

        /**
         * @brief Q_SLOT to start streamin the video from the rtsp.
         */
        void receiveStartStreaming(void);

    protected:
        virtual void run() override;

        QImage streamFrameVisible;
        std::shared_ptr<QMutex> visibleMutex;

};
} // namespace wirispro_manager_panel

#endif // CUSTOMTHREAD_H