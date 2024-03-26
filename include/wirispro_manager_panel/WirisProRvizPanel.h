#pragma once

#include <ros/ros.h>
#include <rviz/panel.h>

namespace wirispro_manager_panel {

class WirisProManagerWidget;

/**
 * @brief WirisProRvizPanel.
 *
 * This custom rviz::Panel implements all necessary
 * functions to work as an independent rviz plugin.
 *
 */
class WirisProRvizPanel : public rviz::Panel
{
    Q_OBJECT

  public:
    /**
     * @brief Constructor of the WirisProRvizPanel class.
     *
     * @param parent A parent QWidget, if there is one.
     */
    WirisProRvizPanel(QWidget* parent = nullptr);

    /**
     * @brief Destructor of the WirisProRvizPanel class.
     */
    ~WirisProRvizPanel();

    /**
     * @brief Load a rviz configuration.
     *
     * @param config rviz::Config with the configuration
     *               to load from.
     */
    virtual void load(const rviz::Config& config);

    /**
     * @brief Save the actual rviz configuration.
     *
     * @param config rviz::Config to save.
     */
    virtual void save(rviz::Config config) const;

  private:
    /**
     * @brief Sets up the main panel with the widget
     * object.
     */
    void setupPanelLayout();

  protected:
    std::unique_ptr<WirisProManagerWidget> _widget;
    //WirisProManagerWidget* _widget;
};
} // namespace rosbag_rviz_panel