#pragma once

#include <QList>
#include <QThread>
#include <QWidget>
#include <QPixmap>
#include <QImage>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "visiblethread.h"
//#include "QCustomProgressBar.h"

namespace Ui {
class WirisProManagerWidget;
}

namespace wirispro_manager_panel {

/**
 * @brief WirisProManagerWidget.
 *
 * This custom QWidget controls the Qt-based user interface,
 * connecting all signals and slots.
 *
 */
class WirisProManagerWidget : public QWidget
{
    Q_OBJECT

  public:
    /**
     * @brief Constructor of the WirisProManagerWidget class.
     *
     * @param parent A parent QWidget, if there is one.
     */
    explicit WirisProManagerWidget(QWidget* parent = nullptr);

    /**
     * @brief Destructor of the WirisProManagerWidget class.
     */
    virtual ~WirisProManagerWidget();

  private: // modify from here to do whatever we want
    /**
     * @brief Function that emits the signal to start
     * recording video.
     */
    void startRecording(void);

    /**
     * @brief Function that emits the signal to stop
     * recording video.
     */
    void stopRecording(void);

    /**
     * @brief Function that emits the signal to capture
     * an image.
     */
    void captureImg(void);


    /**
     * @brief Function to connect all the necessary
     * signals and slots between this class and
     * the QBagPlayer class.
     */
    void connectSignals(void);

  Q_SIGNALS:
    // Not sure if they are necessary, but we can add them later
    /**
     * @brief Q_SIGNAL that starts the rosbag playing.
     */
    void sendStartRecording(void);

    /**
     * @brief Q_SIGNAL that stops the rosbag playing.
     */
    void sendStopRecording(void);

    /**
     * @brief Q_SIGNAL that captures the current frame.
     */ 
    void sendCapture(void);

    /**
     * @brief Q_SIGNAL that starts the display of the stream
    */
    void sendStartStream(void);

    /**
     * @brief Q_SIGNAL that stops the display of the stream
    */
    void sendStopStream(void);

    

  private Q_SLOTS:
    /**
     * @brief Q_SLOT that handles actions for when
     * the start button has been clicked.
     *
     * @param checked Bool set to true if the button was
     *        pressed, and false if it was released.
     */
    void handleStartClicked(const bool checked);

    /**
     * @brief Q_SLOT that handles actions for when
     * the stop button has been clicked.
     *
     * @param checked Bool set to true if the button was
     *        pressed, and false if it was released.
     */
    void handleStopRClicked(const bool checked);

    /**
     * @brief Q_SLOT that handles actions for when
     * the capture button has been clicked.
     * 
     * @param checked Bool set to true if the button was
     *       pressed, and false if it was released.
     */
    void handleCaptureClicked(const bool checked);

    /**
     * @brief Q_SLOT that handles actions for when
     * the check box for ETH stream has been clicked.
     * 
     * @param state Int that represents the state of the
     *       check box.
    */
    void handleEthChecked(int state);

    /**
     * @brief Q_SLOT that handles actions for when
     * the ZoomSlider (QSlider) has been moved. 
     * 
     * @param value Int that represents the value of the
     *      QSlider.
    */
    void handleZoomSliderMoved(int value);
    
    /// GIMBAL CONTROL

    /**
     * @brief Q_SLOT that handles actions for when
     * the combo box for the gimbal control mode has been modified.
     * 
     * @param index Int that represents the index of the
     *     combo box.
     * 
    */
    void handleGimbalModeChanged(int index);

    /**
     * @brief Q_SLOT that handles actions for when
     * the QDialogButtonBox for the gimbal angle 
     * control has been clicked in the apply button.
     * 
    */
    void handleGimbalAngleControlApply(void); 

    /**
     * @brief Q_SLOT that handles actions for when
     * the QDialogButtonBox for the gimbal angle
     * control has been clicked in the reset button.
    */
    void handleGimbalAngleControlReset(void);

    /**
     * @brief Q_SLOT that helps to visualize the gimbal angles
     * 
    */    
    void handleGimbalAnglesTracker(void);
    

  private:
    std::unique_ptr<Ui::WirisProManagerWidget> _ui;

    ros::NodeHandle _nh;

    // As we are gonna use a ros driver to control the camera that will act as a server, we need to create clients
    // to send the commands to the server
    ros::ServiceClient _start_recording_client;
    ros::ServiceClient _stop_recording_client;
    ros::ServiceClient _capture_client;
    ros::ServiceClient _eth_stream_client;

    // Thread management
    std::unique_ptr<VisibleThread>         _visible_stream;
    std::unique_ptr<QThread>            _visible_stream_thread;

};
} // namespace wirispro_manager_panel