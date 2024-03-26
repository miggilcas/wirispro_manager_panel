#include "wirispro_manager_panel/WirisProRvizPanel.h"

#include <QFrame>
#include <QHBoxLayout>
#include <QPushButton>
#include <QVBoxLayout>

#include "wirispro_manager_panel/WirisProManagerWidget.h"

namespace wirispro_manager_panel {
WirisProRvizPanel::WirisProRvizPanel(QWidget* parent) : rviz::Panel(parent)
{
    _widget = std::make_unique<WirisProManagerWidget>();
    //
    setupPanelLayout();
}

WirisProRvizPanel::~WirisProRvizPanel() {}

void WirisProRvizPanel::setupPanelLayout()
{
    if (!_widget) {
        ROS_WARN("Widget is null");
        return;
    }
    //_widget = new WirisProManagerWidget;
    QVBoxLayout* layout = new QVBoxLayout;
    layout->addWidget(_widget.get());

    setLayout(layout);
}

void WirisProRvizPanel::load(const rviz::Config& config)
{
    rviz::Panel::load(config);
}

void WirisProRvizPanel::save(rviz::Config config) const
{
    rviz::Panel::save(config);
}
} // namespace wirispro_manager_panel

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(wirispro_manager_panel::WirisProRvizPanel, rviz::Panel)