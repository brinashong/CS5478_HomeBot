#include "ros/spinner.h"
#include "task_handler/task_handler.hpp"
#include <memory>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "task_handler_node");
  ros::NodeHandle nh, pnh("~");

  std::shared_ptr<TaskHandler> obj = std::make_shared<TaskHandler>(nh, pnh);

  ros::MultiThreadedSpinner spinner(3);
  spinner.spin();

  return 0;
}

