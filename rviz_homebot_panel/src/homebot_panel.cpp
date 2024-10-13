#include "rviz_homebot_panel/homebot_panel.hpp"
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rviz_panel::HomebotPanel, rviz::Panel)

namespace rviz_panel
{
  HomebotPanel::HomebotPanel(QWidget *parent)
    : rviz::Panel(parent)
    , ui_{std::make_shared<Ui::HomebotPanel>()}
  {
    // Extend the widget with all attributes and children from UI file
    ui_->setupUi(this);

    // Define ROS publisher
    // button_1_pub_ = nh_.advertise<std_msgs::Bool>("button_1_topic", 1);
    // button_2_pub_ = nh_.advertise<std_msgs::Bool>("button_2_topic", 1);

    // Declare ROS msg_
    // msg_.data = true;

    // connect(ui_->pushButton_1, SIGNAL(clicked()), this, SLOT(button_one()));
    // connect(ui_->pushButton_2, SIGNAL(clicked()), this, SLOT(button_two()));
  }

  // void simplePanel::button_one()
  // {
  //   ROS_INFO_STREAM("Button one pressed.");
  //   this->button_1_pub_.publish(this->msg_);
  // }
  //
  // void simplePanel::button_two()
  // {
  //   ROS_INFO_STREAM("Button two pressed.");
  //   this->button_2_pub_.publish(this->msg_);
  // }

  void HomebotPanel::save(rviz::Config config) const
  {
    rviz::Panel::save(config);
  }

    void HomebotPanel::load(const rviz::Config &config)
  {
    rviz::Panel::load(config);
  }

} // rviz_panel

