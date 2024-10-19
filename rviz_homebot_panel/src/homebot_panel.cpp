#include "rviz_homebot_panel/homebot_panel.hpp"
#include "std_srvs/SetBoolRequest.h"
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

    nh_ = ros::NodeHandle();
    goal_srv_client_ = nh_.serviceClient<std_srvs::SetBool>("/homebot/goal_task");

    state_sub_ = nh_.subscribe<std_msgs::String>("/homebot/state", 1, [this](const std_msgs::String::ConstPtr& input){ this->stateCallback(input); });
    task_sub_ = nh_.subscribe<std_msgs::String>("/homebot/task", 1, [this](const std_msgs::String::ConstPtr& input){ this->taskCallback(input); });
    goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, [this](const geometry_msgs::PoseStamped::ConstPtr& input){ this->goalCallback(input); });

    reset_srv_server_ = nh_.advertiseService("/homebot/reset", &HomebotPanel::resetCallback, this);

    ui_->pushButton->setEnabled(false);

    sent_goal = false;

    connect(ui_->pushButton, SIGNAL(clicked()), this, SLOT(button()));
  }

  void HomebotPanel::stateCallback(const std_msgs::String::ConstPtr& input)
  {
    ui_->state->setText(QString::fromStdString(input->data));
  }

  void HomebotPanel::taskCallback(const std_msgs::String::ConstPtr& input)
  {
    ui_->task->setText(QString::fromStdString(input->data));
  }

  void HomebotPanel::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& input)
  {
    ui_->pushButton->setEnabled(true);
  }

  bool HomebotPanel::resetCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
  {
    if (req.data)
    {
      ui_->pushButton->setText("Send");
      ui_->pushButton->setEnabled(false);

      res.success = true;
      res.message = "reset";
    }

    return true;
  }

  void HomebotPanel::button()
  {
    std_srvs::SetBool srv;

    // abort goal
    if (sent_goal)
    {
      srv.request.data = false;

      if (goal_srv_client_.call(srv) && srv.response.success)
      {
        ROS_INFO_STREAM("Result: " << srv.response.message);
        sent_goal = false;
        ui_->pushButton->setText("Send");
      }
      else
      {
        ROS_ERROR_STREAM("Failed to abort task!!!");
      }
    }
    else // send goal
    {
      srv.request.data = true;

      if (goal_srv_client_.call(srv) && srv.response.success)
      {
        ROS_INFO_STREAM("Result: " << srv.response.message);
        sent_goal = true;
        ui_->pushButton->setText("Abort");
      }
      else
      {
        ROS_ERROR_STREAM("Failed to send goal!!!");
      }
    }
  }

  void HomebotPanel::save(rviz::Config config) const
  {
    rviz::Panel::save(config);
  }

    void HomebotPanel::load(const rviz::Config &config)
  {
    rviz::Panel::load(config);
  }

} // rviz_panel

