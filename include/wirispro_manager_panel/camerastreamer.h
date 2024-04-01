// NOT USEFUL FOR NOW

#ifndef CAMERASTREAMER_H
#define CAMERASTREAMER_H

#include <QObject>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "visiblethread.h"

namespace wirispro_manager_panel {
 
extern QImage streamFrame;
extern QImage streamFrameVisible;
extern std::shared_ptr<QMutex> thermalMutex;
extern std::shared_ptr<QMutex> visibleMutex;

// class CameraStreamer : public QObject
// {
//     Q_OBJECT
//     //threads for recieving, they are started after successful connection and setup
//     std::unique_ptr<VisibleThread>      _visibleThread = nullptr;
//     //std::unique_ptr<ThermalThread>      _thermalThread = nullptr;
//     // camera parameters and connection state
//     //std::shared_ptr<CameraParameters>   _params = nullptr;
//     //std::shared_ptr<CameraConnection>   _connection = nullptr;
// public:
//     CameraStreamer();
//     //void toggleStreamThermal();
//     void toggleStreamVisible();
//     //void killStreamThermal();
//     void stopStream();
//     //void mainCameraSnapshot();
//     //void secondaryCameraSnapshot();
// Q_SIGNALS:
//     void streamFrameChanged();
//     //void streamFrameVisibleChanged();
//     //void snapshotSaved(QString path);
// };
} // namespace wirispro_manager_panel
#endif // CAMERASTREAMER_H