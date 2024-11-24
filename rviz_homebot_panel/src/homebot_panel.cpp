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

    nh_ = ros::NodeHandle();

    goal_srv_client_ = nh_.serviceClient<task_handler::GoalTask>("/homebot/goal_task");

    state_sub_ = nh_.subscribe<std_msgs::String>("/homebot/state", 1, [this](const std_msgs::String::ConstPtr& input){ this->stateCallback(input); });
    task_sub_ = nh_.subscribe<std_msgs::String>("/homebot/task", 1, [this](const std_msgs::String::ConstPtr& input){ this->taskCallback(input); });

    reset_srv_server_ = nh_.advertiseService("/homebot/reset", &HomebotPanel::resetCallback, this);

    sent_goal = false;

    zone_ = ui_->comboBox->currentText().toStdString();

    connect(ui_->pushButton, SIGNAL(clicked()), this, SLOT(button()));
    connect(ui_->comboBox, SIGNAL(currentTextChanged(QString)), this, SLOT(comboBox()));
  }

  void HomebotPanel::stateCallback(const std_msgs::String::ConstPtr& input)
  {
    ui_->state->setText(QString::fromStdString(input->data));
  }

  void HomebotPanel::taskCallback(const std_msgs::String::ConstPtr& input)
  {
    ui_->task->setText(QString::fromStdString(input->data));
  }

  bool HomebotPanel::resetCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
  {
    if (req.data)
    {
      ui_->pushButton->setText("Send");

      res.success = true;
      res.message = "reset";
    }

    return true;
  }

  void HomebotPanel::button()
  {
    task_handler::GoalTask srv;

    // abort goal
    if (sent_goal)
    {
      srv.request.active = false;

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
      srv.request.active = true;
      srv.request.zone = zone_;

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

  void HomebotPanel::comboBox()
  {
    auto cleaning_zone = ui_->comboBox->currentText().toStdString();

    if (cleaning_zone == "Coffee Table" || cleaning_zone == "Dining Table")
    {
      zone_ = cleaning_zone;
    }
    else
    {
      ROS_ERROR_STREAM("Cleaning zone not defined!");
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

