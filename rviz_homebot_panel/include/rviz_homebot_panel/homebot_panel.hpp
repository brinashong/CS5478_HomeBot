#ifndef RVIZ_HOMEBOT_PANEL_HOMEBOT_PANEL_HPP
#define RVIZ_HOMEBOT_PANEL_HOMEBOT_PANEL_HPP

#include <ros/ros.h>
#include <rviz/panel.h>
#include <ros/service_client.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>

#include <QString>
#include <ui_homebot_panel.h>

#include "task_handler/GoalTask.h"

namespace rviz_panel
{
  class HomebotPanel : public rviz::Panel
  {
    Q_OBJECT

    public:
      explicit HomebotPanel(QWidget *parent = 0);
      ~HomebotPanel() {}

      void stateCallback(const std_msgs::String::ConstPtr& input);
      void taskCallback(const std_msgs::String::ConstPtr& input);
      void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& input);
      bool resetCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

      virtual void save(rviz::Config config) const;
      virtual void load(const rviz::Config &config);

      public Q_SLOTS:

      private Q_SLOTS:
        void button();
        void comboBox();

    protected:
      std::shared_ptr<Ui::HomebotPanel> ui_;
      // ROS declaration
      ros::NodeHandle nh_;
      ros::Subscriber state_sub_;
      ros::Subscriber task_sub_;
      ros::ServiceClient goal_srv_client_;
      ros::ServiceServer reset_srv_server_;

      bool sent_goal;
      std::string zone_;
      geometry_msgs::PoseStamped goal_;
  };
} // rviz_panel

#endif // RVIZ_HOMEBOT_PANEL_HOMEBOT_PANEL_HPP
