/*
 * message_callback.cpp
 *
 *  Created on: Apr 23, 2018
 *      Author: robotemperor
 */

#include "adol_foot_step_generator/message_callback.h"


ros::ServiceClient   g_get_ref_step_data_client;
ros::ServiceClient   g_add_step_data_array_client;

ros::ServiceClient   g_is_running_client;

ros::ServiceClient  g_set_balance_param_client;

ros::Subscriber     g_walking_module_status_msg_sub;

ros::Subscriber     g_walking_command_sub;
ros::Subscriber     g_balance_command_sub;
ros::Subscriber     g_footsteps_2d_sub;

ros::Subscriber     g_dsp_sub;
ros::Subscriber     g_foot_z_swap_sub;
ros::Subscriber     g_body_z_swap_sub;
ros::Subscriber     g_y_zmp_convergence_sub;

adol::FootStepGenerator g_foot_stp_generator;

adol_preview_walking_module_msgs::AddStepDataArray     add_step_data_array_srv;

adol_foot_step_generator::FootStepCommand last_command;

double g_last_command_time = 0;

bool g_is_running_check_needed = false;

std::string dsp_ratio_param_name = "/adol/dsp_ratio";
std::string foot_z_swap_param_name = "/adol/foot_z_swap_m";
std::string body_z_swap_param_name = "/adol/body_z_swap_m";
std::string y_zmp_convergence_param_name = "/adol/y_zmp_convergence_m";

void initialize(void)
{
  ros::NodeHandle nh;

  g_get_ref_step_data_client      = nh.serviceClient<adol_preview_walking_module_msgs::GetReferenceStepData>("/adol/online_walking/get_reference_step_data");
  g_add_step_data_array_client    = nh.serviceClient<adol_preview_walking_module_msgs::AddStepDataArray>("/adol/online_walking/add_step_data");
  g_set_balance_param_client      = nh.serviceClient<adol_preview_walking_module_msgs::SetBalanceParam>("/adol/online_walking/set_balance_param");
  g_is_running_client             = nh.serviceClient<adol_preview_walking_module_msgs::IsRunning>("/adol/online_walking/is_running");

  g_walking_module_status_msg_sub = nh.subscribe("/robotis/status", 10, walkingModuleStatusMSGCallback);

  g_walking_command_sub           = nh.subscribe("/adol/foot_step_generator/walking_command", 0, walkingCommandCallback);
 // g_footsteps_2d_sub              = nh.subscribe("/robotis/thormang3_foot_step_generator/footsteps_2d",    0, step2DArrayCallback);
  g_dsp_sub                 = nh.subscribe("/adol/foot_step_generator/dsp", 5, dspCallback);
  g_foot_z_swap_sub         = nh.subscribe("/adol/foot_step_generator/foot_z_swap", 5, footZSwapCallback);
  g_body_z_swap_sub         = nh.subscribe("/adol/foot_step_generator/body_z_swap", 5, bodyZSwapCallback);
  g_y_zmp_convergence_sub   = nh.subscribe("/adol/foot_step_generator/y_zmp_convergence", 5, yZMPConvergenceCallback);
  

  //nh.param<double>("", )

  ros::param::get(dsp_ratio_param_name, g_foot_stp_generator.dsp_ratio_);
  ros::param::get(foot_z_swap_param_name, g_foot_stp_generator.foot_z_swap_m_);
  ros::param::get(body_z_swap_param_name, g_foot_stp_generator.body_z_swap_m_);
  ros::param::get(y_zmp_convergence_param_name, g_foot_stp_generator.y_zmp_convergence_m_);

  g_last_command_time = ros::Time::now().toSec();
}

void dspCallback(const std_msgs::Float64::ConstPtr& msg)
{
  if(msg->data <= 0 || msg->data >= 1)
  {
    ROS_ERROR("Invalid DSP Ratio");
    return;
  }

  ROS_INFO_STREAM("SET DSP RATIO : " << msg->data);
  g_foot_stp_generator.dsp_ratio_ = msg->data;

  ros::param::set(dsp_ratio_param_name, g_foot_stp_generator.dsp_ratio_);
}

void footZSwapCallback(const std_msgs::Float64::ConstPtr& msg)
{
  if(msg->data <= 0)
  {
    ROS_ERROR("Invalid Foot Z Swap");
    return;
  }

  ROS_INFO_STREAM("SET Foot Z Swap : " << msg->data);
  g_foot_stp_generator.foot_z_swap_m_ = msg->data;

  ros::param::set(foot_z_swap_param_name, g_foot_stp_generator.foot_z_swap_m_);
}

void bodyZSwapCallback(const std_msgs::Float64::ConstPtr& msg)
{
  if(msg->data < 0)
  {
    ROS_ERROR("Invalid Body Z Swap");
    return;
  }

  ROS_INFO_STREAM("SET Body Z Swap : " << msg->data);
  g_foot_stp_generator.body_z_swap_m_ = msg->data;

  ros::param::set(body_z_swap_param_name, g_foot_stp_generator.body_z_swap_m_);
}

void yZMPConvergenceCallback(const std_msgs::Float64::ConstPtr& msg)
{
  ROS_INFO_STREAM("SET Y ZMP Convergence : " << msg->data);
  g_foot_stp_generator.y_zmp_convergence_m_ = msg->data;

  ros::param::set(y_zmp_convergence_param_name, g_foot_stp_generator.y_zmp_convergence_m_);
}

void walkingModuleStatusMSGCallback(const robotis_controller_msgs::StatusMsg::ConstPtr& msg)
{
  if(msg->type == msg->STATUS_ERROR)
    ROS_ERROR_STREAM("[Robot] : " << msg->status_msg);
  else if(msg->type == msg->STATUS_INFO)
    ROS_INFO_STREAM("[Robot] : " << msg->status_msg);
  else if(msg->type == msg->STATUS_WARN)
    ROS_WARN_STREAM("[Robot] : " << msg->status_msg);
  else if(msg->type == msg->STATUS_UNKNOWN)
    ROS_ERROR_STREAM("[Robot] : " << msg->status_msg);
  else
    ROS_ERROR_STREAM("[Robot] : " << msg->status_msg);
}

void walkingCommandCallback(const adol_foot_step_generator::FootStepCommand::ConstPtr &msg)
{
  double now_time = ros::Time::now().toSec();

  if((last_command.command == msg->command)
      && (last_command.step_num == msg->step_num)
      && (last_command.step_time == msg->step_time)
      && (last_command.step_length == msg->step_length)
      && (last_command.side_step_length == msg->side_step_length)
      && (last_command.step_angle_rad == msg->step_angle_rad))
  {
    //prevent double click
    if( (fabs(now_time - g_last_command_time) < last_command.step_time) )
    {
      ROS_ERROR("Receive same command in short time");
      return;
    }
  }

  g_last_command_time = now_time;

  last_command.command          = msg->command;
  last_command.step_num         = msg->step_num;
  last_command.step_time        = msg->step_time;
  last_command.step_length      = msg->step_length;
  last_command.side_step_length = msg->side_step_length;
  last_command.step_angle_rad   = msg->step_angle_rad;


  ROS_INFO("[Demo]  : Walking Command");
  ROS_INFO_STREAM("  command          : " << msg->command );
  ROS_INFO_STREAM("  step_num         : " << msg->step_num );
  ROS_INFO_STREAM("  step_time        : " << msg->step_time );
  ROS_INFO_STREAM("  step_length      : " << msg->step_length);
  ROS_INFO_STREAM("  side_step_length : " << msg->side_step_length );
  ROS_INFO_STREAM("  step_angle_rad   : " << msg->step_angle_rad );

  if((msg->step_num == 0)
      && (msg->command != "left kick")
      && (msg->command != "right kick")
      && (msg->command != "stop"))
    return;

  //set walking parameter
  if(msg->step_length < 0)
  {
    g_foot_stp_generator.fb_step_length_m_ = 0;
    ROS_ERROR_STREAM("step_length is negative.");
    ROS_ERROR_STREAM("It will be set to zero.");
  }
  else
  {
    g_foot_stp_generator.fb_step_length_m_ = msg->step_length;
  }

  if(msg->side_step_length < 0)
  {
    g_foot_stp_generator.rl_step_length_m_ = 0;
    ROS_ERROR_STREAM("side_step_length is negative.");
    ROS_ERROR_STREAM("It will be set to zero.");
  }
  else
  {
    g_foot_stp_generator.rl_step_length_m_ = msg->side_step_length;
  }

  if(msg->step_angle_rad < 0)
  {
    g_foot_stp_generator.rotate_step_angle_rad_ = 0;
    ROS_ERROR_STREAM("step_angle_rad is negative.");
    ROS_ERROR_STREAM("It will be set to zero.");
  }
  else
  {
    g_foot_stp_generator.rotate_step_angle_rad_ = msg->step_angle_rad;
  }

  if(msg->step_time < MINIMUM_STEP_TIME_SEC)
  {
    g_foot_stp_generator.step_time_sec_ = MINIMUM_STEP_TIME_SEC;
    ROS_ERROR_STREAM("step_time is less than minimum step time. ");
    ROS_ERROR_STREAM("It will be set to minimum step time(0.4 sec).");
  }
  else
  {
    g_foot_stp_generator.step_time_sec_ = msg->step_time;
  }

  g_foot_stp_generator.num_of_step_ = 2*(msg->step_num) + 2;


  adol_preview_walking_module_msgs::GetReferenceStepData    get_ref_stp_data_srv;
  adol_preview_walking_module_msgs::StepData                ref_step_data;
  adol_preview_walking_module_msgs::AddStepDataArray        add_stp_data_srv;


  //get reference step data
  if(g_get_ref_step_data_client.call(get_ref_stp_data_srv) == false)
  {
    ROS_ERROR("Failed to get reference step data");
    return;
  }

  ref_step_data = get_ref_stp_data_srv.response.reference_step_data;

  //calc step data
  if(msg->command == "forward")
  {
    if(g_is_running_check_needed == true)
      if(isRunning() == true)
        return;

    g_foot_stp_generator.getStepData( &add_stp_data_srv.request.step_data_array, ref_step_data, FORWARD_WALKING);
    g_is_running_check_needed = false;
  }
  else if(msg->command == "backward")
  {
    if(g_is_running_check_needed == true)
      if(isRunning() == true)
        return;

    g_foot_stp_generator.getStepData( &add_stp_data_srv.request.step_data_array, ref_step_data, BACKWARD_WALKING);
    g_is_running_check_needed = false;
  }
  else if(msg->command == "turn left")
  {
    if(g_is_running_check_needed == true)
      if(isRunning() == true)
        return;

    g_foot_stp_generator.getStepData( &add_stp_data_srv.request.step_data_array, ref_step_data, LEFT_ROTATING_WALKING);
    g_is_running_check_needed = false;
  }
  else if(msg->command == "turn right")
  {
    if(g_is_running_check_needed == true)
      if(isRunning() == true)
        return;

    g_foot_stp_generator.getStepData( &add_stp_data_srv.request.step_data_array, ref_step_data, RIGHT_ROTATING_WALKING);
    g_is_running_check_needed = false;
  }
  else if(msg->command == "right")
  {
    if(g_is_running_check_needed == true)
      if(isRunning() == true)
        return;

    g_foot_stp_generator.getStepData( &add_stp_data_srv.request.step_data_array, ref_step_data, RIGHTWARD_WALKING);
    g_is_running_check_needed = false;
  }
  else if(msg->command == "left")
  {
    if(g_is_running_check_needed == true)
      if(isRunning() == true)
        return;

    g_foot_stp_generator.getStepData( &add_stp_data_srv.request.step_data_array, ref_step_data, LEFTWARD_WALKING);
    g_is_running_check_needed = false;
  }
  else if(msg->command == "right kick")
  {
    if(isRunning() == true)
      return;

    g_foot_stp_generator.calcRightKickStep( &add_stp_data_srv.request.step_data_array, ref_step_data);
    g_is_running_check_needed = true;
  }
  else if(msg->command == "left kick")
  {
    if(isRunning() == true)
      return;

    g_foot_stp_generator.calcLeftKickStep( &add_stp_data_srv.request.step_data_array, ref_step_data);
    g_is_running_check_needed = true;
  }
  else if(msg->command == "turn left right kick")
  {
    if(isRunning() == true)
      return;

	g_foot_stp_generator.calcTurnLeftAndRightKickStep( &add_stp_data_srv.request.step_data_array, ref_step_data);
	g_is_running_check_needed = true;
  }
  else if(msg->command == "turn right left kick")
  {
    if(isRunning() == true)
      return;

	g_foot_stp_generator.calcTurnRightAndLeftKickStep( &add_stp_data_srv.request.step_data_array, ref_step_data);
	g_is_running_check_needed = true;
  }
  else if(msg->command == "stop")
  {
    if(g_is_running_check_needed == true)
      if(isRunning() == true)
        return;

    g_foot_stp_generator.getStepData( &add_stp_data_srv.request.step_data_array, ref_step_data, STOP_WALKING);
    g_is_running_check_needed = false;
  }
  else
  {
    ROS_ERROR("[Demo]  : Invalid Command");
    return;
  }

  //set add step data srv for auto start
  add_stp_data_srv.request.auto_start = true;
  add_stp_data_srv.request.remove_existing_step_data = true;

  //add step data
  if(g_add_step_data_array_client.call(add_stp_data_srv) == true)
  {
    int add_stp_data_srv_result = add_stp_data_srv.response.result;
    if(add_stp_data_srv_result== adol_preview_walking_module_msgs::AddStepDataArray::Response::NO_ERROR)
    {
      ROS_INFO("[Demo]  : Succeed to add step data array");
    }
    else
    {
      ROS_ERROR("[Demo]  : Failed to add step data array");

      if(add_stp_data_srv_result & adol_preview_walking_module_msgs::AddStepDataArray::Response::NOT_ENABLED_WALKING_MODULE)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::NOT_ENABLED_WALKING_MODULE");
      if(add_stp_data_srv_result & adol_preview_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_POSITION_DATA)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::PROBLEM_IN_POSITION_DATA");
      if(add_stp_data_srv_result & adol_preview_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::PROBLEM_IN_TIME_DATA");
      if(add_stp_data_srv_result & adol_preview_walking_module_msgs::AddStepDataArray::Response::TOO_MANY_STEP_DATA)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::TOO_MANY_STEP_DATA");
      if(add_stp_data_srv_result & adol_preview_walking_module_msgs::AddStepDataArray::Response::ROBOT_IS_WALKING_NOW)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::ROBOT_IS_WALKING_NOW");

      g_foot_stp_generator.initialize();

      return;
    }
  }
  else
  {
    ROS_ERROR("[Demo]  : Failed to add step data array ");
    g_foot_stp_generator.initialize();
    return;
  }

}


void step2DArrayCallback(const adol_foot_step_generator::Step2DArray::ConstPtr& msg)
{
  adol_preview_walking_module_msgs::GetReferenceStepData get_ref_stp_data_srv;
  adol_preview_walking_module_msgs::StepData             ref_step_data;
  adol_preview_walking_module_msgs::AddStepDataArray     add_stp_data_srv;
  adol_preview_walking_module_msgs::IsRunning            is_running_srv;

  if(isRunning() == true)
    return;


  //get reference step data
  if(g_get_ref_step_data_client.call(get_ref_stp_data_srv) == false)
  {
    ROS_ERROR("[Demo]  : Failed to get reference step data");
    return;
  }

  ref_step_data = get_ref_stp_data_srv.response.reference_step_data;

  g_foot_stp_generator.getStepDataFromStepData2DArray(&add_stp_data_srv.request.step_data_array, ref_step_data, msg);
  g_is_running_check_needed = true;

  //set add step data srv fot auto start and remove existing step data
  add_stp_data_srv.request.auto_start = true;
  add_stp_data_srv.request.remove_existing_step_data = true;

  //add step data
  if(g_add_step_data_array_client.call(add_stp_data_srv) == true)
  {
    int add_stp_data_srv_result = add_stp_data_srv.response.result;
    if(add_stp_data_srv_result== adol_preview_walking_module_msgs::AddStepDataArray::Response::NO_ERROR)
      ROS_INFO("[Demo]  : Succeed to add step data array");
    else {
      ROS_ERROR("[Demo]  : Failed to add step data array");

      if(add_stp_data_srv_result & adol_preview_walking_module_msgs::AddStepDataArray::Response::NOT_ENABLED_WALKING_MODULE)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::NOT_ENABLED_WALKING_MODULE");
      if(add_stp_data_srv_result & adol_preview_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_POSITION_DATA)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::PROBLEM_IN_POSITION_DATA");
      if(add_stp_data_srv_result & adol_preview_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::PROBLEM_IN_TIME_DATA");
      if(add_stp_data_srv_result & adol_preview_walking_module_msgs::AddStepDataArray::Response::ROBOT_IS_WALKING_NOW)
        ROS_ERROR("[Demo]  : STEP_DATA_ERR::ROBOT_IS_WALKING_NOW");

      return;
    }
  }
  else
  {
    ROS_ERROR("[Demo]  : Failed to add step data array ");
    return;
  }
}

bool isRunning(void)
{
	adol_preview_walking_module_msgs::IsRunning is_running_srv;
  if(g_is_running_client.call(is_running_srv) == false)
  {
    ROS_ERROR("[Demo]  : Failed to Walking Status");
    return true;
  }
  else
  {
    if(is_running_srv.response.is_running == true)
    {
      ROS_ERROR("[Demo]  : STEP_DATA_ERR::ROBOT_IS_WALKING_NOW");
      return true;
    }
  }
  return false;
}
