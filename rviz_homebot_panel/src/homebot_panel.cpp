#include "rviz_homebot_panel/homebot_panel.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
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
    goal_.header.frame_id = "map";
    goal_.pose.position.x = 2.645;
    goal_.pose.position.y = -1.667;
    goal_.pose.position.z = 0.0;
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, -1.57);
    quat.normalize();
    goal_.pose.orientation = tf2::toMsg(quat);

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
      srv.request.goal = goal_;

      std::cout << "sent: " << goal_.pose.position.x << " " << goal_.pose.position.y << std::endl;

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
    tf2::Quaternion quat;

    if (cleaning_zone == "Coffee Table")
    {
      zone_ = cleaning_zone;
      goal_.pose.position.x = 2.645;
      goal_.pose.position.y = -1.667;
      goal_.pose.position.z = 0.0;
      quat.setRPY(0.0, 0.0, -1.57);
      quat.normalize();
      goal_.pose.orientation = tf2::toMsg(quat);
    }
    else if (cleaning_zone == "Dining Table")
    {
      zone_ = cleaning_zone;
      goal_.pose.position.x = 5.28;
      goal_.pose.position.y = 0.98;
      goal_.pose.position.z = 0.0;
      quat.setRPY(0.0, 0.0, 1.57);
      quat.normalize();
      goal_.pose.orientation = tf2::toMsg(quat);
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

