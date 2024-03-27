#include "wirispro_manager_panel/WirisProManagerWidget.h"

#include <QFileDialog>
#include <QIcon>
#include <QMessageBox>
#include <QPushButton>
#include <std_srvs/Trigger.h>          // trigger srv from std srv

#include "ui_WirisProManagerWidget.h"
#include "wirispro_manager/CameraEthStreamService.h"


namespace wirispro_manager_panel {
// Constructor
WirisProManagerWidget::WirisProManagerWidget(QWidget* parent) : QWidget(parent), _ui(std::make_unique<Ui::WirisProManagerWidget>()),_nh("~")
{
    _ui->setupUi(this);
    // /// THREADS Declaration
    // // At first we declare the QObjects for capture and convert
    QStreamer streamer;
    // Capture capture;
    // Converter converter;

    // // We need to specify that the QLabel which will contain the image is _ui->stream_label
    streamer.setLabel(_ui->stream_label);

    // // We need a Capture and Convert threads, type Thread  
    // Thread captureThread, converterThread;

    // converter.setProcessAll(false);
    // captureThread.start();
    // converterThread.start();

    // capture.moveToThread(&captureThread);
    // converter.moveToThread(&converterThread);
    // connect(&capture, &Capture::frameReady, &converter, &Converter::processFrame);
    // connect(&converter, &Converter::imageReady, &streamer, &QStreamer::setImage);
    // connect(&capture, &Capture::started, [](){ qDebug() << "Capture started."; });
    // QMetaObject::invokeMethod(&capture, "start");

    

    /// ROS STUFF
    // ros clients initialization, TBD: take a look into the relative/absolute path of the service
    _start_recording_client = _nh.serviceClient<std_srvs::Trigger>("/recording_start");
    _stop_recording_client = _nh.serviceClient<std_srvs::Trigger>("/recording_stop");
    _capture_client = _nh.serviceClient<std_srvs::Trigger>("/capture");
    _eth_stream_client = _nh.serviceClient<wirispro_manager::CameraEthStreamService>("/set_eth_stream");

    // TBD: include gimbal subscribers

    // DEBUGGING, we set the angles to 10, 20, 30
    _ui->lcdNumber_pitch->display(10);
    _ui->lcdNumber_roll->display(20);
    _ui->lcdNumber_yaw->display(30);
    // make the stream_label (QLabel) invisible
    //_ui->stream_label->setVisible(false);

    
    

    connectSignals();

    /// BUTTONS RELATED
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
   // TBD: connect the signals and slots between the QStreamer and the Capture and Converter classes    

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

    }
    
}
void WirisProManagerWidget::handleZoomSliderMoved(int value){
    // TBD: implement the service call to change the zoom level and map the value to the zoom level

    
    // Debugging:
    ROS_INFO("Zoom level changed to %d", value);



}

void WirisProManagerWidget::handleGimbalModeChanged(int index){
    // TBD: implement the service call to change the gimbal mode
   
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
    


}

void WirisProManagerWidget::handleGimbalAngleControlApply(void){
    // TBD: implement the service call to change the gimbal angles
    
    // Debugging: print the apply button clicked
    ROS_INFO("Apply button clicked");

    // Showing the values from the QDoubleSpinBoxes
    ROS_INFO(" Angles selected: %f, %f, %f", _ui->doubleSpinBox_roll->value(), _ui->doubleSpinBox_pitch->value(), _ui->doubleSpinBox_yaw->value());

    
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
void WirisProManagerWidget::handleGimbalAnglesTracker(void){
    // TBD: implement the service call to visualize the gimbal angles
    
    // Debugging: Manual display of the angles in the QLcdNumbers
    _ui->lcdNumber_pitch->display(10);
    _ui->lcdNumber_roll->display(20);
    _ui->lcdNumber_yaw->display(30);

    


}
// Testing purposes
QImage WirisProManagerWidget::convertOpenCVMatToQtQImage(cv::Mat mat)
{
    if(mat.channels() == 1) { // if 1 channel (grayscale or black and white) image
        return QImage((uchar*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Indexed8);// return QImage
    }
    else if(mat.channels() == 3) { // if 3 channel color image
        cv::cvtColor(mat, mat, cv::COLOR_BGR2RGB);// flip colors
        return QImage((uchar*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);// return QImage
    }
    else {
        qDebug() << "in convertOpenCVMatToQtQImage, image was not 1 channel or 3 channel, should never get here";
    }
    return QImage();// return a blank QImage if the above did not work
}

// TBD: Implement temperature display
}  // namespace wirispro_manager_panel