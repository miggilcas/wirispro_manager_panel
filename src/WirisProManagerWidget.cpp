#include "wirispro_manager_panel/WirisProManagerWidget.h"

#include <QFileDialog>
#include <QIcon>
#include <QMessageBox>
#include <QPushButton>
#include <std_srvs/Trigger.h>          // trigger srv from std srv

#include "ui_WirisProManagerWidget.h"
#include "wirispro_manager/CameraEthStreamService.h"
#include "gremsy_base/GimbalPose.h"  
#include "gremsy_base/GimbalMode.h"
#include "geometry_msgs/Vector3.h"
#include <std_msgs/Bool.h

namespace wirispro_manager_panel {
// Constructor
WirisProManagerWidget::WirisProManagerWidget(QWidget* parent) : QWidget(parent), _ui(std::make_unique<Ui::WirisProManagerWidget>()),_nh("~")
{
    _ui->setupUi(this);
    // ros clients initialization, TBD: take a look into the relative/absolute path of the service
    _start_recording_client = _nh.serviceClient<std_srvs::Trigger>("/recording_start");
    _stop_recording_client = _nh.serviceClient<std_srvs::Trigger>("/recording_stop");
    _capture_client = _nh.serviceClient<std_srvs::Trigger>("/capture");
    _eth_stream_client = _nh.serviceClient<wirispro_manager::CameraEthStreamService>("/set_eth_stream");


    // TBD: include gimbal subscribers or services
    _set_gimbal_goal_client = _nh.serviceClient<gremsy_base::GimbalPose>("/ros_gremsy/goal");
    _set_gimbal_mode_client = _nh.serviceClient<gremsy_base::GimbalMode>("/ros_gremsy/mode");
    _gimbal_angles_sub = _nh.subscribe("/ros_gremsy/angle", 10, &WirisProManagerWidget::gimbalAnglesCB, this);

    // for debugging purposes, we set the angles to 10, 20, 30
    // _ui->lcdNumber_pitch->display(10);
    // _ui->lcdNumber_roll->display(20);
    // _ui->lcdNumber_yaw->display(30);
    // make the stream_label (QLabel) invisible
    //_ui->stream_label->setVisible(false);

    //------------------ Image testing -----------------------------------------------------
    QString filename = "/home/user/simar_ws/src/wirispro_manager_panel/docs/init.png";
    
    /** set content to show center in label */
    _ui->stream_label->setAlignment(Qt::AlignCenter);
    QPixmap pix;

    /** to check wether load ok */
    if(pix.load(filename)){
        /** scale pixmap to fit in label'size and keep ratio of pixmap */
        pix = pix.scaled(_ui->stream_label->size(),Qt::KeepAspectRatio);
        _ui->stream_label->setPixmap(pix);
    }
    // Trying to print some video file features to make sure opencv can be used
    // cv::VideoCapture cap("/home/user/meme.mp4");
    // if(!cap.isOpened()){
    //     ROS_ERROR("Error opening video file");
    // }
    // else{
    //     ROS_WARN("Video file opened");
    //     ROS_INFO("Frame width: %d", cap.get(cv::CAP_PROP_FRAME_WIDTH));
    //     ROS_INFO("Frame height: %d", cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    //     ROS_INFO("Frame count: %d", cap.get(cv::CAP_PROP_FRAME_COUNT));
    //     ROS_INFO("Frame rate: %f", cap.get(cv::CAP_PROP_FPS));
    // }
    //------------------ It works      -----------------------------------------------------
    
    // Visible thread management
    // remember that the constructor of the VisibleThread class is: VisibleThread(bool* stream, const QString & ssrc, QLabel *label)
    _visible_stream = std::make_unique<VisibleThread>(new bool(true), "10.42.0.230", _ui->stream_label);
    _visible_stream_thread = std::make_unique<QThread>(this);

    // Move the thread to the QThread
    _visible_stream->moveToThread(_visible_stream_thread.get());
    // Make a connection to the finished signal of the thread
    connect(_visible_stream_thread.get(), &QThread::finished, _visible_stream.get(), &QObject::deleteLater, Qt::QueuedConnection);
    // Connect the signals to the slot
    connect(this, &WirisProManagerWidget::sendStartStream, _visible_stream.get(), &VisibleThread::receiveStartStreaming);
    connect(this, &WirisProManagerWidget::sendStopStream, _visible_stream.get(), &VisibleThread::receiveStopStreaming);
    // Start the thread
    _visible_stream_thread->start();

    
    

    connectSignals();

    QIcon::setThemeName("Yaru");
    _ui->start_button->setIcon(QIcon::fromTheme("media-playback-start"));
    _ui->stop_button->setIcon(QIcon::fromTheme("media-playback-stop"));
    //_ui->capture_button->setIcon(QIcon::fromTheme("camera-photo"));
    //_ui->eth_checkBox->setIcon(QIcon::fromTheme("media-seek-backward"));
    
    // Button connections from the ui generated file with Qt Designer
    connect(_ui->start_button, &QPushButton::clicked, this, &WirisProManagerWidget::handleStartClicked);
    connect(_ui->stop_button, &QPushButton::clicked, this, &WirisProManagerWidget::handleStopRClicked);
    connect(_ui->capture_button, &QPushButton::clicked, this, &WirisProManagerWidget::handleCaptureClicked);

    // Checkbox connections from the ui generated file with Qt Designer
    connect(_ui->eth_checkBox, &QCheckBox::stateChanged, this, &WirisProManagerWidget::handleEthChecked);

    // Slider connections from the ui generated file with Qt Designer
    connect(_ui->zoom_slider, &QSlider::valueChanged, this, &WirisProManagerWidget::handleZoomSliderMoved);

    // Combo box connections from the ui generated file with Qt Designer
    connect(_ui->mode_comboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &WirisProManagerWidget::handleGimbalModeChanged);
    // More Gimbal control connections
    // buttonBox
    // when the apply button is clicked
    connect(_ui->angles_buttonBox, &QDialogButtonBox::accepted, this, &WirisProManagerWidget::handleGimbalAngleControlApply);
    // when the reset button is clicked
    connect(_ui->angles_buttonBox, &QDialogButtonBox::rejected, this, &WirisProManagerWidget::handleGimbalAngleControlReset);




}
// Destructor
WirisProManagerWidget::~WirisProManagerWidget()
{  
}

// Function to connect all the necessary signals and slots between this class and the QBagPlayer class.
void WirisProManagerWidget::connectSignals(void)
{   
    //TBD: Make the proper connections between the signals and slots


    // connect(this, &WirisProManagerWidget::sendStartRecording, this, &WirisProManagerWidget::startRecording);
    // connect(this, &WirisProManagerWidget::sendStopRecording, this, &WirisProManagerWidget::stopRecording);
    // connect(this, &WirisProManagerWidget::sendCaptureImg, this, &WirisProManagerWidget::captureImg);

}

// Handling functions:
void WirisProManagerWidget::handleStartClicked(const bool checked){
    // TBD: implement the service call to start recording
    //Q_EMIT sendStartRecording();
    std_srvs::Trigger srv;
    // Debugging:
    if (checked) {
    ROS_INFO("Start recording service called");
    }
    else {  
        ROS_INFO("Start recording WTF");
        if (_start_recording_client.call(srv))
        {
            ROS_INFO("Start recording service called");
        }
        else
        {
            ROS_ERROR("Failed to call start recording service");
        }
    }
    
}

void WirisProManagerWidget::handleStopRClicked(const bool checked){
    // TBD: implement the service call to stop recording
    //Q_EMIT sendStopRecording();
    std_srvs::Trigger srv;
    // Debugging:
    
    if(checked){
        ROS_INFO("Stop recording service called");
    }
    else{
        ROS_INFO("Stop recording WTF");
        if (_stop_recording_client.call(srv))
        {
            ROS_INFO("Stop recording service called");
        }
        else
        {
            ROS_ERROR("Failed to call stop recording service");
        }
    }
    
}
void WirisProManagerWidget::handleCaptureClicked(const bool checked){
    // TBD: implement the service call to capture image
    //Q_EMIT sendCaptureImg();
    std_srvs::Trigger srv;
    // Debugging:
    if(checked){
        ROS_INFO("Capture image service called");
    }
    else{
        ROS_INFO("Capture image WTF");
        if (_capture_client.call(srv))
        {
            ROS_INFO("Capture image service called");
        }
        else
        {
            ROS_ERROR("Failed to call capture image service");
        }
    }

    // 
}
void WirisProManagerWidget::handleEthChecked(int state){
    // Here we implement the service call to start/stop the eth stream
    
    wirispro_manager::CameraEthStreamService srv;
    // Debugging:
    if(state){
        // we need to access to the enable variable of the srv request, string type
        srv.request.enable = "TRUE";

        ROS_INFO("ETH stream checked");
        if(_eth_stream_client.call(srv)){
            ROS_INFO("ETH set stream service called with enable %s", srv.request.enable.c_str());
        }
        else{
            ROS_ERROR("Failed to call ETH stream service");
        }
        
        // make the qLabel visible
        _ui->stream_label->setVisible(true);
        // Emit the signal to start the stream after 1 second to be sure the streaming is available
        // wait for 1 second
        std::this_thread::sleep_for(std::chrono::seconds(1));
        Q_EMIT sendStartStream();
    }
    else{
        srv.request.enable = "FALSE";

        ROS_INFO("ETH stream unchecked");
        if(_eth_stream_client.call(srv)){
            ROS_INFO("ETH set stream service called with enable %s", srv.request.enable.c_str());
        }
        else{
            ROS_ERROR("Failed to call ETH stream service");
        }
        // make the qLabel invisible
        _ui->stream_label->setVisible(false);
        // TBD: Implement a way to destruct the thread when the checkbox is unchecked
        // Emit the signal to stop the stream
        Q_EMIT sendStopStream();

    }
    
}
void WirisProManagerWidget::handleZoomSliderMoved(int value){
    // TBD: implement the service call to change the zoom level and map the value to the zoom level

    
    // Debugging:
    ROS_INFO("Zoom level changed to %d", value);



}

void WirisProManagerWidget::handleGimbalModeChanged(int index){
    // TBD: implement the service call to change the gimbal mode
    gremsy_base::GimbalMode srv;
    srv.request.mode.data = index;
    // Debugging:
    switch (index)
    {
    case 0:
        ROS_INFO("Gimbal mode changed to LOCKED");
        
        break;

    case 1:
        ROS_INFO("Gimbal mode changed to FOLLOW");
        
        break;
    }
    
    if(_set_gimbal_goal_client.call(srv)){
        ROS_INFO("Mode changed successfuly");
    }
    else{
        ROS_ERROR("Failed to call gimbal mode service")
    }


}

void WirisProManagerWidget::handleGimbalAngleControlApply(void){
    
    gremsy_base::GimbalPose srv;
    // Debugging: print the apply button clicked
    ROS_INFO("Apply button clicked");
    srv.request.pos.x =_ui->doubleSpinBox_roll->value();
    srv.request.pos.y =_ui->doubleSpinBox_pitch->value();
    srv.request.pos.z =_ui->doubleSpinBox_yaw->value();
    //Calling the service
    if(_set_gimbal_goal_client.call(srv)){
        // Showing the values from the QDoubleSpinBoxes
        ROS_INFO(" Angles selected: %f, %f, %f", _ui->doubleSpinBox_roll->value(), _ui->doubleSpinBox_pitch->value(), _ui->doubleSpinBox_yaw->value());
        
    }else{
        ROS_ERROR("Failed to call gimbal goal service");

    }
    
}
void WirisProManagerWidget::handleGimbalAngleControlReset(void){
    // TBD: implement the service call to reset the gimbal angles
    
    // Debugging: print the reset button clicked
    ROS_INFO("Reset button clicked");
    // Reset the values of the QDoubleSpinBoxes
    _ui->doubleSpinBox_pitch->setValue(0);
    _ui->doubleSpinBox_roll->setValue(0);
    _ui->doubleSpinBox_yaw->setValue(0);

}
void WirisProManagerWidget::gimbalAnglesCB(const geometry_msgs::Vector3::ConstPtr& msg){
    // TBD: implement the service call to visualize the gimbal angles
    
    // Debugging: Manual display of the angles in the QLcdNumbers
    _ui->lcdNumber_pitch->display(msg.data.x);
    _ui->lcdNumber_roll->display(msg.y);
    _ui->lcdNumber_yaw->display(msg.z);

    


}

// Callbacks
void WirisProManager

// TBD: Implement temperature display
}  // namespace wirispro_manager_panel