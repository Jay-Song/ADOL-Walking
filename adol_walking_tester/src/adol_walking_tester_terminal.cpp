#include <ros/ros.h>
#include <ros/package.h>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include <std_msgs/String.h>

#include <adol_foot_step_generator/FootStepCommand.h>

adol_foot_step_generator::FootStepCommand foot_step_msg_;

ros::Publisher walking_cmd_pub_, set_ctrl_module_pub_;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lifting_trajectory_loader");
  ros::NodeHandle nh;

  walking_cmd_pub_     = nh.advertise<adol_foot_step_generator::FootStepCommand>("/adol/foot_step_generator/walking_command", 0);
  set_ctrl_module_pub_ = nh.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);

  std::string cmd;

  while(ros::ok())
  {
    std::cout << "[CMD]: " ;
    std::cin >> cmd;

    if (cmd == "exit")
      break;
    else if (cmd == "set")
    {
      std_msgs::String control_msg;
      control_msg.data = "preview_walking_module";

      set_ctrl_module_pub_.publish(control_msg);
    }
    else if (cmd == "none")
    {
      std_msgs::String control_msg;
      control_msg.data = "none";

      set_ctrl_module_pub_.publish(control_msg);
    }
    else if (cmd == "forward")
    {
      adol_foot_step_generator::FootStepCommand cmd_msg;
      cmd_msg.command = "forward";
      cmd_msg.step_num = 3;
      cmd_msg.step_time = 0.6;
      cmd_msg.step_length = 0.1;
      cmd_msg.side_step_length = 0;
      cmd_msg.step_angle_rad = 0.0;

      walking_cmd_pub_.publish(cmd_msg);
    }

    ros::spinOnce();
  }

  return 0;
}