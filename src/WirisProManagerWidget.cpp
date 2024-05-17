#include "wirispro_manager_panel/WirisProManagerWidget.h"

#include <QFileDialog>
#include <QIcon>
#include <QMessageBox>
#include <QPushButton>
#include <std_srvs/Trigger.h> // trigger srv from std srv

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "gremsy_base/GimbalMode.h"
#include "gremsy_base/GimbalPos.h"
#include "ui_WirisProManagerWidget.h"
#include "wirispro_manager/CameraEthStreamService.h"

#include <std_msgs/Bool.h>

namespace wirispro_manager_panel {
// Constructor
WirisProManagerWidget::WirisProManagerWidget(QWidget *parent)
    : QWidget(parent), _ui(std::make_unique<Ui::WirisProManagerWidget>()),
      _nh("~") {
  _ui->setupUi(this);
  QIcon::setThemeName("Yaru");
  _ui->start_button->setIcon(QIcon::fromTheme("media-playback-start"));
  _ui->stop_button->setIcon(QIcon::fromTheme("media-playback-stop"));
  _ui->capture_button->setIcon(QIcon::fromTheme("camera-photo"));

  // ros clients initialization, TBD: take a look into the relative/absolute
  // path of the service
  _start_recording_client =
      _nh.serviceClient<std_srvs::Trigger>("/recording_start");
  _stop_recording_client =
      _nh.serviceClient<std_srvs::Trigger>("/recording_stop");
  _capture_client = _nh.serviceClient<std_srvs::Trigger>("/capture");
  _eth_stream_client =
      _nh.serviceClient<wirispro_manager::CameraEthStreamService>(
          "/set_eth_stream");

  // TBD: include gimbal subscribers or services
  _set_gimbal_goal_client =
      _nh.serviceClient<gremsy_base::GimbalPos>("/ros_gremsy/goal");
  _set_gimbal_mode_client =
      _nh.serviceClient<gremsy_base::GimbalMode>("/ros_gremsy/mode");
  _gimbal_angles_sub = _nh.subscribe(
      "/ros_gremsy/angle", 1000, &WirisProManagerWidget::gimbalAnglesCB, this);

  // make the stream_label (QLabel) invisible
  _ui->stream_label->setVisible(false);
  _ui->thermal_label->setVisible(false);

  //------------------ Image testing
  //-----------------------------------------------------
  // QString filename =
  // "/home/user/simar_ws/src/wirispro_manager_panel/docs/init.png";

  // /** set content to show center in label */
  // _ui->stream_label->setAlignment(Qt::AlignCenter);
  // QPixmap pix;

  // /** to check wether load ok */
  // if(pix.load(filename)){
  //     /** scale pixmap to fit in label'size and keep ratio of pixmap */
  //     pix = pix.scaled(_ui->stream_label->size(),Qt::KeepAspectRatio);
  //     _ui->stream_label->setPixmap(pix);
  // }
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
  //------------------ It works
  //-----------------------------------------------------

  /// Thread management

  // Visible Thread
  // remember that the constructor of the VisibleThread class is:
  // VisibleThread(bool* stream, const QString & ssrc, QLabel *label)
  _visible_stream = std::make_unique<VisibleThread>(
      new bool(true), "10.42.0.230", _ui->stream_label);
  _visible_stream_thread = std::make_unique<QThread>(this);

  // Move the thread to the QThread
  _visible_stream->moveToThread(_visible_stream_thread.get());

  // Thermal Thread
  _thermal_stream = std::make_unique<ThermalThread>(
      new bool(true), "10.42.0.230", _ui->thermal_label);
  _thermal_stream_thread = std::make_unique<QThread>(this);

  // Move the thread to the QThread
  _thermal_stream->moveToThread(_thermal_stream_thread.get());

  /// CONNECTIONS

  connectSignals(); // TBD: check if all the connections could be done inside
                    // the function
  // Make a connection to the finished signal of the visible thread
  connect(_visible_stream_thread.get(), &QThread::finished,
          _visible_stream.get(), &QObject::deleteLater, Qt::QueuedConnection);
  // Connect the signals to the slot
  connect(this, &WirisProManagerWidget::sendStartVisibleStream,
          _visible_stream.get(), &VisibleThread::receiveStartStreaming);
  connect(this, &WirisProManagerWidget::sendStopStream, _visible_stream.get(),
          &VisibleThread::receiveStopStreaming);
  // Start the thread
  _visible_stream_thread->start();

  // Make a connection to the finished signal of the visible thread
  connect(_thermal_stream_thread.get(), &QThread::finished,
          _thermal_stream.get(), &QObject::deleteLater, Qt::QueuedConnection);
  // Connect the signals to the slot
  connect(this, &WirisProManagerWidget::sendStartThermalStream,
          _thermal_stream.get(), &ThermalThread::receiveStartStreaming);
  _thermal_stream_thread->start();

  // Button connections from the ui generated file with Qt Designer
  connect(_ui->start_button, &QPushButton::clicked, this,
          &WirisProManagerWidget::handleStartClicked);
  connect(_ui->stop_button, &QPushButton::clicked, this,
          &WirisProManagerWidget::handleStopRClicked);
  connect(_ui->capture_button, &QPushButton::clicked, this,
          &WirisProManagerWidget::handleCaptureClicked);

  // Checkbox connections from the ui generated file with Qt Designer
  connect(_ui->eth_checkBox, &QCheckBox::stateChanged, this,
          &WirisProManagerWidget::handleEthChecked);

  // Slider connections from the ui generated file with Qt Designer
  connect(_ui->zoom_slider, &QSlider::valueChanged, this,
          &WirisProManagerWidget::handleZoomSliderMoved);

  // Combo box connections from the ui generated file with Qt Designer
  connect(_ui->mode_comboBox,
          QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &WirisProManagerWidget::handleGimbalModeChanged);
  connect(_ui->stream_comboBox,
          QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &WirisProManagerWidget::handleStreamModeChanged);
  // More Gimbal control connections
  // buttonBox
  // when the apply button is clicked
  connect(_ui->angles_buttonBox, &QDialogButtonBox::accepted, this,
          &WirisProManagerWidget::handleGimbalAngleControlApply);
  // when the reset button is clicked
  connect(_ui->angles_buttonBox, &QDialogButtonBox::rejected, this,
          &WirisProManagerWidget::handleGimbalAngleControlReset);
}
// Destructor
WirisProManagerWidget::~WirisProManagerWidget() {}

// Function to connect all the necessary signals and slots between this class
// and the QBagPlayer class.
void WirisProManagerWidget::connectSignals(void) {
  // TBD: Make the proper connections between the signals and slots
}

// Handling functions:
void WirisProManagerWidget::handleStartClicked(const bool checked) {
  // TBD: implement the service call to start recording
  // Q_EMIT sendStartRecording();
  std_srvs::Trigger srv;
  // Debugging:
  if (checked) {
    ROS_INFO("Start recording service called");
  } else {
    ROS_INFO("Start recording WTF");
    if (_start_recording_client.call(srv)) {
      ROS_INFO("Start recording service called");
    } else {
      ROS_ERROR("Failed to call start recording service");
    }
  }
}

void WirisProManagerWidget::handleStopRClicked(const bool checked) {
  // TBD: implement the service call to stop recording
  // Q_EMIT sendStopRecording();
  std_srvs::Trigger srv;
  // Debugging:

  if (checked) {
    ROS_INFO("Stop recording service called");
  } else {
    ROS_INFO("Stop recording WTF");
    if (_stop_recording_client.call(srv)) {
      ROS_INFO("Stop recording service called");
    } else {
      ROS_ERROR("Failed to call stop recording service");
    }
  }
}
void WirisProManagerWidget::handleCaptureClicked(const bool checked) {
  // TBD: implement the service call to capture image
  // Q_EMIT sendCaptureImg();
  std_srvs::Trigger srv;
  // Debugging:
  if (checked) {
    ROS_INFO("Capture image service called");
  } else {
    ROS_INFO("Capture image WTF");
    if (_capture_client.call(srv)) {
      ROS_INFO("Capture image service called");
    } else {
      ROS_ERROR("Failed to call capture image service");
    }
  }

  //
}
void WirisProManagerWidget::handleEthChecked(int state) {
  // Here we implement the service call to start/stop the eth stream

  wirispro_manager::CameraEthStreamService srv;
  // Debugging:
  if (state) {
    // we need to access to the enable variable of the srv request, string type
    srv.request.enable = "TRUE";

    ROS_INFO("ETH stream checked");
    if (_eth_stream_client.call(srv)) {
      ROS_INFO("ETH set stream service called with enable %s",
               srv.request.enable.c_str());
    } else {
      ROS_ERROR("Failed to call ETH stream service");
    }

    // make the qLabel visible
    _ui->stream_label->setVisible(true);
    _ui->thermal_label->setVisible(true);
    // Emit the signal to start the stream  1 second after the thread start
    // running to be sure the streaming is available
    //_visible_stream_thread->start();
    // wait for 1 second
    std::this_thread::sleep_for(std::chrono::seconds(1));
    Q_EMIT sendStartVisibleStream();
    Q_EMIT sendStartThermalStream();
  } else {
    srv.request.enable = "FALSE";

    ROS_INFO("ETH stream unchecked");
    if (_eth_stream_client.call(srv)) {
      ROS_INFO("ETH set stream service called with enable %s",
               srv.request.enable.c_str());
    } else {
      ROS_ERROR("Failed to call ETH stream service");
    }
    // make the qLabel invisible
    _ui->stream_label->setVisible(false);
    // TBD: Implement a way to destruct the thread when the checkbox is
    // unchecked Emit the signal to stop the stream
    // Q_EMIT sendStopStream();
    //_visible_stream_thread->quit();
    //_visible_stream_thread->wait();
  }
}
void WirisProManagerWidget::handleZoomSliderMoved(int value) {
  // TBD: implement the service call to change the zoom level and map the value
  // to the zoom level

  // Debugging:
  ROS_INFO("Zoom level changed to %d", value);
}

void WirisProManagerWidget::handleGimbalModeChanged(int index) {
  // TBD: implement the service call to change the gimbal mode
  gremsy_base::GimbalMode srv;
  srv.request.mode = index;
  // Debugging:
  switch (index) {
  case 0:
    ROS_INFO("Gimbal mode changed to LOCKED");

    break;

  case 1:
    ROS_INFO("Gimbal mode changed to FOLLOW");

    break;
  }

  if (_set_gimbal_mode_client.call(srv)) {
    ROS_INFO("Mode changed successfuly");
  } else {
    ROS_ERROR("Failed to call gimbal mode service");
  }
}

void WirisProManagerWidget::handleGimbalAngleControlApply(void) {

  gremsy_base::GimbalPos srv;
  // Debugging: print the apply button clicked
  ROS_INFO("Apply button clicked");
  srv.request.pos.x = _ui->doubleSpinBox_pitch->value();
  srv.request.pos.y = _ui->doubleSpinBox_roll->value();
  srv.request.pos.z = _ui->doubleSpinBox_yaw->value();
  // Calling the service
  if (_set_gimbal_goal_client.call(srv)) {
    // Showing the values from the QDoubleSpinBoxes
    ROS_INFO(" Angles selected: %f, %f, %f", _ui->doubleSpinBox_roll->value(),
             _ui->doubleSpinBox_pitch->value(),
             _ui->doubleSpinBox_yaw->value());

  } else {
    ROS_ERROR("Failed to call gimbal goal service");
  }
}
void WirisProManagerWidget::handleGimbalAngleControlReset(void) {
  // TBD: implement the service call to reset the gimbal angles

  // Debugging: print the reset button clicked
  ROS_INFO("Reset button clicked");
  // Reset the values of the QDoubleSpinBoxes
  _ui->doubleSpinBox_pitch->setValue(0);
  _ui->doubleSpinBox_roll->setValue(0);
  _ui->doubleSpinBox_yaw->setValue(0);
}
void WirisProManagerWidget::gimbalAnglesCB(
    const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
  // TBD: Modify it to be actually useful, it does not update the values once
  // the callback is called

  // Debugging: Manual display and update of the angles in the QLcdNumbers
  ROS_INFO("Angles received: %f, %f, %f", msg->vector.x, msg->vector.y,
           msg->vector.z);
  _ui->lcdNumber_pitch->display(msg->vector.x);
  _ui->lcdNumber_roll->display(msg->vector.y);
  _ui->lcdNumber_yaw->display(msg->vector.z);
}

void WirisProManagerWidget::handleStreamModeChanged(int index) {
  // TBD: implement the service call to change the stream mode
  // Debugging:
  switch (index) {
  case 0:
    ROS_INFO("Stream mode changed to VISIBLE");
    _ui->stream_label->clear();
    _ui->thermal_label->clear();
    _ui->thermal_label->setVisible(false);
    _ui->stream_label->setVisible(true);

    // if(_visible_stream_thread.get()->isRunning()){
    //     ROS_INFO("Visible thread is already running ");

    // }else{

    //     if(_thermal_stream_thread.get()->isRunning()){
    //         ROS_INFO("Thermal thread is running, STOPPING it and STARTING the
    //         VISIBLE thread");
    //         //Q_EMIT sendStopStream();
    //         _thermal_stream_thread->quit();
    //         _thermal_stream_thread->wait();

    //     ROS_WARN("Visible thread is not running, Starting it now");
    //     _visible_stream_thread->start();

    //     Q_EMIT sendStartVisibleStream();
    // }
    break;
  case 1:
    ROS_INFO("Stream mode changed to THERMAL");
    _ui->stream_label->clear();
    _ui->thermal_label->clear();
    _ui->stream_label->setVisible(false);
    _ui->thermal_label->setVisible(true);
    // if(_thermal_stream_thread.get()->isRunning()){
    //     ROS_INFO("Thermal thread is already running ");
    // }else{
    //     if(_visible_stream_thread.get()->isRunning()){
    //         ROS_INFO("Visible thread is running, STOPPING it and STARTING the
    //         THERMAL thread");
    //         //Q_EMIT sendStopStream();
    //         _visible_stream_thread->quit();
    //         _visible_stream_thread->wait();
    //     ROS_WARN("Thermal thread is not running, Starting it now");
    //     _thermal_stream_thread->start();

    //     Q_EMIT sendStartThermalStream();
    // }
    // }
    break;
  }
}

// TBD: Implement temperature display
} // namespace wirispro_manager_panel
