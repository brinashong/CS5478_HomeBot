#ifndef RVIZ_HOMEBOT_PANEL_HOMEBOT_PANEL_HPP
#define RVIZ_HOMEBOT_PANEL_HOMEBOT_PANEL_HPP

#include <ros/ros.h>
#include <rviz/panel.h>

#include <ui_homebot_panel.h>

namespace rviz_panel
{
  class HomebotPanel : public rviz::Panel
  {
    Q_OBJECT

    public:
      explicit HomebotPanel(QWidget *parent = 0);
      ~HomebotPanel() {}

      virtual void save(rviz::Config config) const;
      virtual void load(const rviz::Config &config);

      public Q_SLOTS:

      private Q_SLOTS:
      // void button_one();
      // void button_two();

    protected:
      std::shared_ptr<Ui::HomebotPanel> ui_;
      // ROS declaration
      ros::NodeHandle nh_;
      // ros::Publisher button_1_pub_;
      // ros::Publisher button_2_pub_;
  };
} // rviz_panel

#endif // RVIZ_HOMEBOT_PANEL_HOMEBOT_PANEL_HPP
