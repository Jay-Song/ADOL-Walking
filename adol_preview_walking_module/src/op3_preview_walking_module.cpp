/*
 * op3_preview_walking_module.cpp
 *
 *  Created on: 2024. 04. 17.
 *      Author: Jay Sog
n */

#include "adol_preview_walking_module/op3_preview_walking_module.h"

using namespace adol;

class WalkingStatusMSG
{
public:
  static const std::string FAILED_TO_ADD_STEP_DATA_MSG;
  static const std::string BALANCE_PARAM_SETTING_STARTED_MSG;
  static const std::string BALANCE_PARAM_SETTING_FINISHED_MSG;
  static const std::string JOINT_FEEDBACK_GAIN_UPDATE_STARTED_MSG;
  static const std::string JOINT_FEEDBACK_GAIN_UPDATE_FINISHED_MSG;
  static const std::string WALKING_MODULE_IS_ENABLED_MSG;
  static const std::string WALKING_MODULE_IS_DISABLED_MSG;
  static const std::string BALANCE_HAS_BEEN_TURNED_OFF;
  static const std::string WALKING_START_MSG;
  static const std::string WALKING_FINISH_MSG;
};

const std::string WalkingStatusMSG::FAILED_TO_ADD_STEP_DATA_MSG = "Failed_to_add_Step_Data";
const std::string WalkingStatusMSG::BALANCE_PARAM_SETTING_STARTED_MSG = "Balance_Param_Setting_Started";
const std::string WalkingStatusMSG::BALANCE_PARAM_SETTING_FINISHED_MSG = "Balance_Param_Setting_Finished";
const std::string WalkingStatusMSG::JOINT_FEEDBACK_GAIN_UPDATE_STARTED_MSG = "Joint_FeedBack_Gain_Update_Started";
const std::string WalkingStatusMSG::JOINT_FEEDBACK_GAIN_UPDATE_FINISHED_MSG = "Joint_FeedBack_Gain_Update_Finished";
const std::string WalkingStatusMSG::WALKING_MODULE_IS_ENABLED_MSG = "Walking_Module_is_enabled";
const std::string WalkingStatusMSG::WALKING_MODULE_IS_DISABLED_MSG = "Walking_Module_is_disabled";
const std::string WalkingStatusMSG::BALANCE_HAS_BEEN_TURNED_OFF = "Balance_has_been_turned_off";
const std::string WalkingStatusMSG::WALKING_START_MSG = "Walking_Started";
const std::string WalkingStatusMSG::WALKING_FINISH_MSG = "Walking_Finished";

OP3PreviewWalkingModule::OP3PreviewWalkingModule()
: control_cycle_msec_(8)
{
  gazebo_          = false;
  enable_          = false;
  module_name_     = "preview_walking_module";
  control_mode_    = robotis_framework::PositionControl;

  result_["r_hip_yaw"  ] = new robotis_framework::DynamixelState();
  result_["r_hip_roll" ] = new robotis_framework::DynamixelState();
  result_["r_hip_pitch"] = new robotis_framework::DynamixelState();
  result_["r_knee"     ] = new robotis_framework::DynamixelState();
  result_["r_ank_pitch"] = new robotis_framework::DynamixelState();
  result_["r_ank_roll" ] = new robotis_framework::DynamixelState();

  result_["l_hip_yaw"  ] = new robotis_framework::DynamixelState();
  result_["l_hip_roll" ] = new robotis_framework::DynamixelState();
  result_["l_hip_pitch"] = new robotis_framework::DynamixelState();
  result_["l_knee"     ] = new robotis_framework::DynamixelState();
  result_["l_ank_pitch"] = new robotis_framework::DynamixelState();
  result_["l_ank_roll" ] = new robotis_framework::DynamixelState();

  joint_name_to_index_["r_hip_yaw"  ] = 0;
  joint_name_to_index_["r_hip_roll" ] = 1;
  joint_name_to_index_["r_hip_pitch"] = 2;
  joint_name_to_index_["r_knee"     ] = 3;
  joint_name_to_index_["r_ank_pitch"] = 4;
  joint_name_to_index_["r_ank_roll" ] = 5;

  joint_name_to_index_["l_hip_yaw"  ] = 6;
  joint_name_to_index_["l_hip_roll" ] = 7;
  joint_name_to_index_["l_hip_pitch"] = 8;
  joint_name_to_index_["l_knee"     ] = 9;
  joint_name_to_index_["l_ank_pitch"] = 10;
  joint_name_to_index_["l_ank_roll" ] = 11;

  previous_running_    = present_running    = false;


  desired_matrix_g_to_pelvis_ = Eigen::MatrixXd::Identity(4,4);
  desired_matrix_g_to_rfoot_  = Eigen::MatrixXd::Identity(4,4);
  desired_matrix_g_to_lfoot_  = Eigen::MatrixXd::Identity(4,4);


  balance_update_with_loop_ = false;
  balance_update_duration_ = 2.0;
  balance_update_sys_time_ = 2.0;
  balance_update_tra_.changeTrajectory(0, 0, 0, 0, balance_update_duration_, 1, 0, 0);

  joint_feedback_update_with_loop_ = false;
  joint_feedback_update_duration_ = 2.0;
  joint_feedback_update_sys_time_ = 2.0;
  joint_feedback_update_tra_.changeTrajectory(0, 0, 0, 0, balance_update_duration_, 1, 0, 0);

  lipm_height_m_ = 0;

  prev_walking_ = new PreviewWalking();
}

OP3PreviewWalkingModule::~OP3PreviewWalkingModule()
{
  queue_thread_.join();
}

void OP3PreviewWalkingModule::setInitialPose(robotis_framework::Pose3D r_foot, robotis_framework::Pose3D l_foot, robotis_framework::Pose3D pelvis)
{
  prev_walking_->setInitialPose(r_foot, l_foot, pelvis);
}

void OP3PreviewWalkingModule::setLIPMHeight(double height_m)
{
  lipm_height_m_ = height_m;
}

void OP3PreviewWalkingModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  queue_thread_ = boost::thread(boost::bind(&OP3PreviewWalkingModule::queueThread, this));
  control_cycle_msec_ = control_cycle_msec;
  
  prev_walking_->initialize(lipm_height_m_, 1.6, control_cycle_msec*0.001);
  prev_walking_->start();
  prev_walking_->process();

  process_mutex_.lock();
  desired_matrix_g_to_pelvis_ = prev_walking_->mat_g_to_pelvis_;
  desired_matrix_g_to_rfoot_  = prev_walking_->mat_g_to_rfoot_;
  desired_matrix_g_to_lfoot_  = prev_walking_->mat_g_to_lfoot_;
  process_mutex_.unlock();

  result_["r_hip_yaw"  ]->goal_position_ = prev_walking_->out_angle_rad_[0];
  result_["r_hip_roll" ]->goal_position_ = prev_walking_->out_angle_rad_[1];
  result_["r_hip_pitch"]->goal_position_ = prev_walking_->out_angle_rad_[2];
  result_["r_knee"     ]->goal_position_ = prev_walking_->out_angle_rad_[3];
  result_["r_ank_pitch"]->goal_position_ = prev_walking_->out_angle_rad_[4];
  result_["r_ank_roll" ]->goal_position_ = prev_walking_->out_angle_rad_[5];

  result_["l_hip_yaw"  ]->goal_position_ = prev_walking_->out_angle_rad_[6];
  result_["l_hip_roll" ]->goal_position_ = prev_walking_->out_angle_rad_[7];
  result_["l_hip_pitch"]->goal_position_ = prev_walking_->out_angle_rad_[8];
  result_["l_knee"     ]->goal_position_ = prev_walking_->out_angle_rad_[9];
  result_["l_ank_pitch"]->goal_position_ = prev_walking_->out_angle_rad_[10];
  result_["l_ank_roll" ]->goal_position_ = prev_walking_->out_angle_rad_[11];

  previous_running_ = isRunning();

  // prev_walking_->balance_ctrl_.foot_roll_gyro_ctrl_.p_gain_ = 0;
  // prev_walking_->balance_ctrl_.foot_roll_gyro_ctrl_.d_gain_ = 0;
  // prev_walking_->balance_ctrl_.foot_pitch_gyro_ctrl_.p_gain_ = 0;
  // prev_walking_->balance_ctrl_.foot_pitch_gyro_ctrl_.d_gain_ = 0;

  // prev_walking_->balance_ctrl_.foot_roll_angle_ctrl_.p_gain_ = 0;
  // prev_walking_->balance_ctrl_.foot_roll_angle_ctrl_.d_gain_ = 0;
  // prev_walking_->balance_ctrl_.foot_pitch_angle_ctrl_.p_gain_ = 0;
  // prev_walking_->balance_ctrl_.foot_pitch_angle_ctrl_.d_gain_ = 0;

  // prev_walking_->balance_ctrl_.right_foot_force_x_ctrl_.p_gain_ = 0;
  // prev_walking_->balance_ctrl_.right_foot_force_x_ctrl_.d_gain_ = 0;
  // prev_walking_->balance_ctrl_.right_foot_force_y_ctrl_.p_gain_ = 0;
  // prev_walking_->balance_ctrl_.right_foot_force_y_ctrl_.d_gain_ = 0;
  // prev_walking_->balance_ctrl_.right_foot_force_z_ctrl_.p_gain_ = 0;
  // prev_walking_->balance_ctrl_.right_foot_force_z_ctrl_.d_gain_ = 0;
  // prev_walking_->balance_ctrl_.right_foot_torque_roll_ctrl_.p_gain_ = 0;
  // prev_walking_->balance_ctrl_.right_foot_torque_roll_ctrl_.d_gain_ = 0;
  // prev_walking_->balance_ctrl_.right_foot_torque_pitch_ctrl_.p_gain_ = 0;
  // prev_walking_->balance_ctrl_.right_foot_torque_pitch_ctrl_.d_gain_ = 0;
}


void OP3PreviewWalkingModule::queueThread()
{
  ros::NodeHandle     ros_node;
  ros::CallbackQueue  callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* publish topics */
  robot_pose_pub_ = ros_node.advertise<adol_preview_walking_module_msgs::RobotPose>("/adol/op3/online_walking/robot_pose", 1);
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/adol/op3/status", 1);
  pelvis_base_msg_pub_ = ros_node.advertise<geometry_msgs::PoseStamped>("/adol/op3/pelvis_pose_base_walking", 1);
  done_msg_pub_ = ros_node.advertise<std_msgs::String>("/adol/op3/movement_done", 1);
  walking_joint_states_pub_ = ros_node.advertise<adol_preview_walking_module_msgs::WalkingJointStatesStamped>("/adol/op3/walking_joint_states", 1);

  /* ROS Service Callback Functions */
  ros::ServiceServer get_ref_step_data_server  = ros_node.advertiseService("/adol/preview_walking/get_reference_step_data",   &OP3PreviewWalkingModule::getReferenceStepDataServiceCallback,   this);
  ros::ServiceServer add_step_data_array_sever = ros_node.advertiseService("/adol/preview_walking/add_step_data",             &OP3PreviewWalkingModule::addStepDataServiceCallback,            this);
  ros::ServiceServer walking_start_server      = ros_node.advertiseService("/adol/preview_walking/walking_start",             &OP3PreviewWalkingModule::startWalkingServiceCallback,           this);
  ros::ServiceServer is_running_server         = ros_node.advertiseService("/adol/preview_walking/is_running",                &OP3PreviewWalkingModule::IsRunningServiceCallback,              this);
  ros::ServiceServer set_balance_param_server  = ros_node.advertiseService("/adol/preview_walking/set_balance_param",         &OP3PreviewWalkingModule::setBalanceParamServiceCallback,        this);
  ros::ServiceServer set_joint_feedback_gain   = ros_node.advertiseService("/adol/preview_walking/joint_feedback_gain",       &OP3PreviewWalkingModule::setJointFeedBackGainServiceCallback,   this);
  ros::ServiceServer remove_existing_step_data = ros_node.advertiseService("/adol/preview_walking/remove_existing_step_data", &OP3PreviewWalkingModule::removeExistingStepDataServiceCallback, this);

  /* sensor topic subscribe */
  ros::Subscriber imu_data_sub; 
  ros::Subscriber com_data_sub;
  //ros::Subscriber ft_data_sub  = ros_node.subscribe("/alice/force_torque_data", 3, &OP3PreviewWalkingModule::ftDataOutputCallback, this);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  if(ros::param::get("gazebo", gazebo_) == false)
    gazebo_ = false;

  if (gazebo_ == true)
  {
    imu_data_sub = ros_node.subscribe("/adol/op3/webots/imu", 3, &OP3PreviewWalkingModule::imuDataOutputCallback, this);
    com_data_sub = ros_node.subscribe("/adol/op3/webots/com", 3, &OP3PreviewWalkingModule::comDataCallback,       this);
  }

  while(ros_node.ok())
    callback_queue.callAvailable(duration);

}

void OP3PreviewWalkingModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "Walking";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);
}

void OP3PreviewWalkingModule::publishDoneMsg(std::string msg)
{
  std_msgs::String done_msg;
  done_msg.data = msg;

  done_msg_pub_.publish(done_msg);
}

int OP3PreviewWalkingModule::convertStepDataMsgToStepData(adol_preview_walking_module_msgs::StepData& src, robotis_framework::StepData& des)
{
  int copy_result = adol_preview_walking_module_msgs::AddStepDataArray::Response::NO_ERROR;
  des.time_data.walking_state           = src.time_data.walking_state;
  des.time_data.abs_step_time           = src.time_data.abs_step_time;
  des.time_data.step_duration           = src.time_data.step_duration;
  des.time_data.dsp_ratio               = src.time_data.dsp_ratio;

  des.position_data.moving_foot         = src.position_data.moving_foot;
  des.position_data.shoulder_swing_gain = 0;
  des.position_data.elbow_swing_gain    = 0;

  des.position_data.x_zmp_shift         = src.position_data.x_zmp_shift;
  des.position_data.y_zmp_shift         = src.position_data.y_zmp_shift;

  des.position_data.foot_z_swap         = src.position_data.foot_z_swap;
  des.position_data.waist_pitch_angle   = 0;
  des.position_data.waist_yaw_angle     = src.position_data.torso_yaw_angle_rad;
  des.position_data.body_z_swap         = src.position_data.body_z_swap;

  des.position_data.body_pose.z          = src.position_data.body_pose.z;
  des.position_data.body_pose.roll       = src.position_data.body_pose.roll;
  des.position_data.body_pose.pitch      = src.position_data.body_pose.pitch;
  des.position_data.body_pose.yaw        = src.position_data.body_pose.yaw;
  des.position_data.right_foot_pose.x     = src.position_data.right_foot_pose.x;
  des.position_data.right_foot_pose.y     = src.position_data.right_foot_pose.y;
  des.position_data.right_foot_pose.z     = src.position_data.right_foot_pose.z;
  des.position_data.right_foot_pose.roll  = src.position_data.right_foot_pose.roll;
  des.position_data.right_foot_pose.pitch = src.position_data.right_foot_pose.pitch;
  des.position_data.right_foot_pose.yaw   = src.position_data.right_foot_pose.yaw;
  des.position_data.left_foot_pose.x      = src.position_data.left_foot_pose.x;
  des.position_data.left_foot_pose.y      = src.position_data.left_foot_pose.y;
  des.position_data.left_foot_pose.z      = src.position_data.left_foot_pose.z;
  des.position_data.left_foot_pose.roll   = src.position_data.left_foot_pose.roll;
  des.position_data.left_foot_pose.pitch  = src.position_data.left_foot_pose.pitch;
  des.position_data.left_foot_pose.yaw    = src.position_data.left_foot_pose.yaw;

  des.time_data.start_time_delay_ratio_x         = src.time_data.start_time_delay_ratio_x;
  des.time_data.start_time_delay_ratio_y         = src.time_data.start_time_delay_ratio_y;
  des.time_data.start_time_delay_ratio_z         = src.time_data.start_time_delay_ratio_z;
  des.time_data.start_time_delay_ratio_roll      = src.time_data.start_time_delay_ratio_roll;
  des.time_data.start_time_delay_ratio_pitch     = src.time_data.start_time_delay_ratio_pitch;
  des.time_data.start_time_delay_ratio_yaw       = src.time_data.start_time_delay_ratio_yaw;

  des.time_data.finish_time_advance_ratio_x     = src.time_data.finish_time_advance_ratio_x;
  des.time_data.finish_time_advance_ratio_y     = src.time_data.finish_time_advance_ratio_y;
  des.time_data.finish_time_advance_ratio_z     = src.time_data.finish_time_advance_ratio_z;
  des.time_data.finish_time_advance_ratio_roll  = src.time_data.finish_time_advance_ratio_roll;
  des.time_data.finish_time_advance_ratio_pitch = src.time_data.finish_time_advance_ratio_pitch;
  des.time_data.finish_time_advance_ratio_yaw   = src.time_data.finish_time_advance_ratio_yaw;

  if((src.time_data.walking_state != adol_preview_walking_module_msgs::StepTimeData::IN_WALKING_STARTING)
      && (src.time_data.walking_state != adol_preview_walking_module_msgs::StepTimeData::IN_WALKING)
      && (src.time_data.walking_state != adol_preview_walking_module_msgs::StepTimeData::IN_WALKING_ENDING) )
    copy_result |= adol_preview_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;

  if((src.time_data.start_time_delay_ratio_x     < 0)
      || (src.time_data.start_time_delay_ratio_y     < 0)
      || (src.time_data.start_time_delay_ratio_z     < 0)
      || (src.time_data.start_time_delay_ratio_roll  < 0)
      || (src.time_data.start_time_delay_ratio_pitch < 0)
      || (src.time_data.start_time_delay_ratio_yaw   < 0) )
    copy_result |= adol_preview_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;

  if((src.time_data.finish_time_advance_ratio_x     < 0)
      || (src.time_data.finish_time_advance_ratio_y     < 0)
      || (src.time_data.finish_time_advance_ratio_z     < 0)
      || (src.time_data.finish_time_advance_ratio_roll  < 0)
      || (src.time_data.finish_time_advance_ratio_pitch < 0)
      || (src.time_data.finish_time_advance_ratio_yaw   < 0) )
    copy_result |= adol_preview_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;

  if(((src.time_data.start_time_delay_ratio_x + src.time_data.finish_time_advance_ratio_x) > 1.0)
      || ((src.time_data.start_time_delay_ratio_y      + src.time_data.finish_time_advance_ratio_y     ) > 1.0)
      || ((src.time_data.start_time_delay_ratio_z      + src.time_data.finish_time_advance_ratio_z     ) > 1.0)
      || ((src.time_data.start_time_delay_ratio_roll   + src.time_data.finish_time_advance_ratio_roll  ) > 1.0)
      || ((src.time_data.start_time_delay_ratio_pitch  + src.time_data.finish_time_advance_ratio_pitch ) > 1.0)
      || ((src.time_data.start_time_delay_ratio_yaw    + src.time_data.finish_time_advance_ratio_yaw   ) > 1.0) )
    copy_result |= adol_preview_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;

  if((src.position_data.moving_foot != adol_preview_walking_module_msgs::StepPositionData::STANDING)
      && (src.position_data.moving_foot != adol_preview_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING)
      && (src.position_data.moving_foot != adol_preview_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING))
    copy_result |= adol_preview_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_POSITION_DATA;

  // if(src.position_data.foot_z_swap < 0)
  //   copy_result |= adol_preview_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_POSITION_DATA;

  return copy_result;
}

int OP3PreviewWalkingModule::convertStepDataToStepDataMsg(robotis_framework::StepData& src, adol_preview_walking_module_msgs::StepData& des)
{
  des.time_data.walking_state   = src.time_data.walking_state;
  des.time_data.abs_step_time   = src.time_data.abs_step_time;
  des.time_data.step_duration   = src.time_data.step_duration;
  des.time_data.dsp_ratio       = src.time_data.dsp_ratio;

  des.time_data.start_time_delay_ratio_x     = des.time_data.finish_time_advance_ratio_x     = 0;
  des.time_data.start_time_delay_ratio_y     = des.time_data.finish_time_advance_ratio_y     = 0;
  des.time_data.start_time_delay_ratio_z     = des.time_data.finish_time_advance_ratio_z     = 0;
  des.time_data.start_time_delay_ratio_roll  = des.time_data.finish_time_advance_ratio_roll  = 0;
  des.time_data.start_time_delay_ratio_pitch = des.time_data.finish_time_advance_ratio_pitch = 0;
  des.time_data.start_time_delay_ratio_yaw   = des.time_data.finish_time_advance_ratio_yaw   = 0;

  des.position_data.moving_foot         = src.position_data.moving_foot;
  des.position_data.foot_z_swap         = src.position_data.foot_z_swap;
  des.position_data.torso_yaw_angle_rad = src.position_data.waist_yaw_angle;
  des.position_data.body_z_swap         = src.position_data.body_z_swap;

  des.position_data.x_zmp_shift         = src.position_data.x_zmp_shift;
  des.position_data.y_zmp_shift         = src.position_data.y_zmp_shift;

  des.position_data.body_pose.z           = src.position_data.body_pose.z;
  des.position_data.body_pose.roll        = src.position_data.body_pose.roll;
  des.position_data.body_pose.pitch       = src.position_data.body_pose.pitch;
  des.position_data.body_pose.yaw         = src.position_data.body_pose.yaw;
  des.position_data.right_foot_pose.x     = src.position_data.right_foot_pose.x;
  des.position_data.right_foot_pose.y     = src.position_data.right_foot_pose.y;
  des.position_data.right_foot_pose.z     = src.position_data.right_foot_pose.z;
  des.position_data.right_foot_pose.roll  = src.position_data.right_foot_pose.roll;
  des.position_data.right_foot_pose.pitch = src.position_data.right_foot_pose.pitch;
  des.position_data.right_foot_pose.yaw   = src.position_data.right_foot_pose.yaw;
  des.position_data.left_foot_pose.x      = src.position_data.left_foot_pose.x;
  des.position_data.left_foot_pose.y      = src.position_data.left_foot_pose.y;
  des.position_data.left_foot_pose.z      = src.position_data.left_foot_pose.z;
  des.position_data.left_foot_pose.roll   = src.position_data.left_foot_pose.roll;
  des.position_data.left_foot_pose.pitch  = src.position_data.left_foot_pose.pitch;
  des.position_data.left_foot_pose.yaw    = src.position_data.left_foot_pose.yaw;

  return 0;
}

bool OP3PreviewWalkingModule::getReferenceStepDataServiceCallback(adol_preview_walking_module_msgs::GetReferenceStepData::Request &req,
    adol_preview_walking_module_msgs::GetReferenceStepData::Response &res)
{
  robotis_framework::StepData refStepData;

  prev_walking_->getReferenceStepDatafotAddition(&refStepData);

  convertStepDataToStepDataMsg(refStepData, res.reference_step_data);

  return true;
}

bool OP3PreviewWalkingModule::addStepDataServiceCallback(adol_preview_walking_module_msgs::AddStepDataArray::Request &req,
    adol_preview_walking_module_msgs::AddStepDataArray::Response &res)
{
  res.result = adol_preview_walking_module_msgs::AddStepDataArray::Response::NO_ERROR;

  if(enable_ == false)
  {
    res.result |= adol_preview_walking_module_msgs::AddStepDataArray::Response::NOT_ENABLED_WALKING_MODULE;
    std::string status_msg = WalkingStatusMSG::FAILED_TO_ADD_STEP_DATA_MSG;
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return true;
  }

  if((req.step_data_array.size() > 100)
      && (req.remove_existing_step_data == true)
      && ((prev_walking_->isRunning() == true)))
  {
    res.result |= adol_preview_walking_module_msgs::AddStepDataArray::Response::TOO_MANY_STEP_DATA;
    std::string status_msg  = WalkingStatusMSG::FAILED_TO_ADD_STEP_DATA_MSG;
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return true;
  }

  robotis_framework::StepData step_data, ref_step_data;
  std::vector<robotis_framework::StepData> req_step_data_array;

  prev_walking_->getReferenceStepDatafotAddition(&ref_step_data);

  for(int i = 0; i < req.step_data_array.size(); i++)
  {
    res.result |= convertStepDataMsgToStepData(req.step_data_array[i], step_data);

    if(step_data.time_data.abs_step_time <= 0)
    {
      res.result |= adol_preview_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;
    }

    if(i != 0)
    {
      if(step_data.time_data.abs_step_time <= req_step_data_array[req_step_data_array.size() - 1].time_data.abs_step_time)
      {
        res.result |= adol_preview_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;
      }
    }
    else
    {
      if(step_data.time_data.abs_step_time <= ref_step_data.time_data.abs_step_time)
      {
        res.result |= adol_preview_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;
      }
    }

    if(res.result != adol_preview_walking_module_msgs::AddStepDataArray::Response::NO_ERROR)
    {
      std::string status_msg = WalkingStatusMSG::FAILED_TO_ADD_STEP_DATA_MSG;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
      return true;
    }

    req_step_data_array.push_back(step_data);
  }

  if(req.remove_existing_step_data == true)
  {
    int exist_num_of_step_data = prev_walking_->getNumofRemainingUnreservedStepData();
    if(exist_num_of_step_data != 0)
      for(int remove_count  = 0; remove_count < exist_num_of_step_data; remove_count++)
        prev_walking_->eraseLastStepData();
  }
  else
  {
    if(prev_walking_->isRunning() == true)
    {
      res.result |= adol_preview_walking_module_msgs::AddStepDataArray::Response::ROBOT_IS_WALKING_NOW;
      std::string status_msg  = WalkingStatusMSG::FAILED_TO_ADD_STEP_DATA_MSG;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
      return true;
    }
  }

  if(checkBalanceOnOff() == false)
  {
    std::string status_msg  = WalkingStatusMSG::BALANCE_HAS_BEEN_TURNED_OFF;
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return true;
  }

  for(unsigned int i = 0; i < req_step_data_array.size() ; i++)
    prev_walking_->addStepData(req_step_data_array[i]);

  if( req.auto_start == true)
  {
    prev_walking_->start();
  }

  return true;
}

bool OP3PreviewWalkingModule::startWalkingServiceCallback(adol_preview_walking_module_msgs::StartWalking::Request &req,
    adol_preview_walking_module_msgs::StartWalking::Response &res)
{
  res.result = adol_preview_walking_module_msgs::StartWalking::Response::NO_ERROR;

  if(enable_ == false)
  {
    res.result |= adol_preview_walking_module_msgs::StartWalking::Response::NOT_ENABLED_WALKING_MODULE;
    return true;
  }

  if(prev_walking_->isRunning() == true)
  {
    res.result |= adol_preview_walking_module_msgs::StartWalking::Response::ROBOT_IS_WALKING_NOW;
    return true;
  }

  if(checkBalanceOnOff() == false)
  {
    std::string status_msg  = WalkingStatusMSG::BALANCE_HAS_BEEN_TURNED_OFF;
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return true;
  }

  if(prev_walking_->getNumofRemainingUnreservedStepData() == 0)
  {
    res.result |= adol_preview_walking_module_msgs::StartWalking::Response::NO_STEP_DATA;
    return true;
  }

  if(res.result == adol_preview_walking_module_msgs::StartWalking::Response::NO_ERROR)
  {
    prev_walking_->start();
  }

  return true;
}

bool OP3PreviewWalkingModule::IsRunningServiceCallback(adol_preview_walking_module_msgs::IsRunning::Request &req,
    adol_preview_walking_module_msgs::IsRunning::Response &res)
{
  bool is_running = isRunning();
  res.is_running = is_running;

  return true;
}

bool OP3PreviewWalkingModule::isRunning()
{
  return prev_walking_->isRunning();
}

bool OP3PreviewWalkingModule::removeExistingStepDataServiceCallback(adol_preview_walking_module_msgs::RemoveExistingStepData::Request  &req,
    adol_preview_walking_module_msgs::RemoveExistingStepData::Response &res)
{
  res.result = adol_preview_walking_module_msgs::RemoveExistingStepData::Response::NO_ERROR;

  if(isRunning())
  {
    res.result |= adol_preview_walking_module_msgs::RemoveExistingStepData::Response::ROBOT_IS_WALKING_NOW;
  }
  else
  {
    int exist_num_of_step_data = prev_walking_->getNumofRemainingUnreservedStepData();
    if(exist_num_of_step_data != 0)
      for(int remove_count  = 0; remove_count < exist_num_of_step_data; remove_count++)
        prev_walking_->eraseLastStepData();
  }
  return true;
}

bool OP3PreviewWalkingModule::setJointFeedBackGainServiceCallback(adol_preview_walking_module_msgs::SetJointFeedBackGain::Request &req,
    adol_preview_walking_module_msgs::SetJointFeedBackGain::Response &res)
{
  res.result = adol_preview_walking_module_msgs::SetJointFeedBackGain::Response::NO_ERROR;

  if( enable_ == false)
    res.result |= adol_preview_walking_module_msgs::SetJointFeedBackGain::Response::NOT_ENABLED_WALKING_MODULE;

  if( joint_feedback_update_with_loop_ == true)
    res.result |= adol_preview_walking_module_msgs::SetJointFeedBackGain::Response::PREV_REQUEST_IS_NOT_FINISHED;

  if( res.result != adol_preview_walking_module_msgs::SetJointFeedBackGain::Response::NO_ERROR)
  {
    publishDoneMsg("walking_joint_feedback_failed");
    return true;
  }

  if( req.updating_duration <= 0.0 )
  {
    // under 8ms apply immediately
    setJointFeedBackGain(req.feedback_gain);
    std::string status_msg = WalkingStatusMSG::JOINT_FEEDBACK_GAIN_UPDATE_FINISHED_MSG;
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
    publishDoneMsg("walking_joint_feedback");
    return true;
  }
  else
  {
    joint_feedback_update_duration_ = req.updating_duration;
  }

  joint_feedback_update_sys_time_ = 0.0;
  double tf = joint_feedback_update_duration_;
  joint_feedback_update_tra_.changeTrajectory(0, 0, 0, 0, joint_feedback_update_duration_, 1.0, 0, 0);

  desired_joint_feedback_gain_ = req.feedback_gain;

  previous_joint_feedback_gain_.r_leg_hip_y_p_gain = prev_walking_->leg_angle_feed_back_[0].p_gain_;
  previous_joint_feedback_gain_.r_leg_hip_y_d_gain = prev_walking_->leg_angle_feed_back_[0].d_gain_;
  previous_joint_feedback_gain_.r_leg_hip_r_p_gain = prev_walking_->leg_angle_feed_back_[1].p_gain_;
  previous_joint_feedback_gain_.r_leg_hip_r_d_gain = prev_walking_->leg_angle_feed_back_[1].d_gain_;
  previous_joint_feedback_gain_.r_leg_hip_p_p_gain = prev_walking_->leg_angle_feed_back_[2].p_gain_;
  previous_joint_feedback_gain_.r_leg_hip_p_d_gain = prev_walking_->leg_angle_feed_back_[2].d_gain_;
  previous_joint_feedback_gain_.r_leg_kn_p_p_gain  = prev_walking_->leg_angle_feed_back_[3].p_gain_;
  previous_joint_feedback_gain_.r_leg_kn_p_d_gain  = prev_walking_->leg_angle_feed_back_[3].d_gain_;
  previous_joint_feedback_gain_.r_leg_an_p_p_gain  = prev_walking_->leg_angle_feed_back_[4].p_gain_;
  previous_joint_feedback_gain_.r_leg_an_p_d_gain  = prev_walking_->leg_angle_feed_back_[4].d_gain_;
  previous_joint_feedback_gain_.r_leg_an_r_p_gain  = prev_walking_->leg_angle_feed_back_[5].p_gain_;
  previous_joint_feedback_gain_.r_leg_an_r_d_gain  = prev_walking_->leg_angle_feed_back_[5].d_gain_;

  previous_joint_feedback_gain_.l_leg_hip_y_p_gain = prev_walking_->leg_angle_feed_back_[6].p_gain_;
  previous_joint_feedback_gain_.l_leg_hip_y_d_gain = prev_walking_->leg_angle_feed_back_[6].d_gain_;
  previous_joint_feedback_gain_.l_leg_hip_r_p_gain = prev_walking_->leg_angle_feed_back_[7].p_gain_;
  previous_joint_feedback_gain_.l_leg_hip_r_d_gain = prev_walking_->leg_angle_feed_back_[7].d_gain_;
  previous_joint_feedback_gain_.l_leg_hip_p_p_gain = prev_walking_->leg_angle_feed_back_[8].p_gain_;
  previous_joint_feedback_gain_.l_leg_hip_p_d_gain = prev_walking_->leg_angle_feed_back_[8].d_gain_;
  previous_joint_feedback_gain_.l_leg_kn_p_p_gain  = prev_walking_->leg_angle_feed_back_[9].p_gain_;
  previous_joint_feedback_gain_.l_leg_kn_p_d_gain  = prev_walking_->leg_angle_feed_back_[9].d_gain_;
  previous_joint_feedback_gain_.l_leg_an_p_p_gain  = prev_walking_->leg_angle_feed_back_[10].p_gain_;
  previous_joint_feedback_gain_.l_leg_an_p_d_gain  = prev_walking_->leg_angle_feed_back_[10].d_gain_;
  previous_joint_feedback_gain_.l_leg_an_r_p_gain  = prev_walking_->leg_angle_feed_back_[11].p_gain_;
  previous_joint_feedback_gain_.l_leg_an_r_d_gain  = prev_walking_->leg_angle_feed_back_[11].d_gain_;

  joint_feedback_update_with_loop_ = true;
  joint_feedback_update_sys_time_  = 0.0;

  return true;
}

void OP3PreviewWalkingModule::setJointFeedBackGain(adol_preview_walking_module_msgs::JointFeedBackGain& msg)
{
  prev_walking_->leg_angle_feed_back_[0].p_gain_  = msg.r_leg_hip_y_p_gain ;
  prev_walking_->leg_angle_feed_back_[0].d_gain_  = msg.r_leg_hip_y_d_gain ;
  prev_walking_->leg_angle_feed_back_[1].p_gain_  = msg.r_leg_hip_r_p_gain ;
  prev_walking_->leg_angle_feed_back_[1].d_gain_  = msg.r_leg_hip_r_d_gain ;
  prev_walking_->leg_angle_feed_back_[2].p_gain_  = msg.r_leg_hip_p_p_gain ;
  prev_walking_->leg_angle_feed_back_[2].d_gain_  = msg.r_leg_hip_p_d_gain ;
  prev_walking_->leg_angle_feed_back_[3].p_gain_  = msg.r_leg_kn_p_p_gain  ;
  prev_walking_->leg_angle_feed_back_[3].d_gain_  = msg.r_leg_kn_p_d_gain  ;
  prev_walking_->leg_angle_feed_back_[4].p_gain_  = msg.r_leg_an_p_p_gain  ;
  prev_walking_->leg_angle_feed_back_[4].d_gain_  = msg.r_leg_an_p_d_gain  ;
  prev_walking_->leg_angle_feed_back_[5].p_gain_  = msg.r_leg_an_r_p_gain  ;
  prev_walking_->leg_angle_feed_back_[5].d_gain_  = msg.r_leg_an_r_d_gain  ;

  prev_walking_->leg_angle_feed_back_[6].p_gain_  = msg.l_leg_hip_y_p_gain ;
  prev_walking_->leg_angle_feed_back_[6].d_gain_  = msg.l_leg_hip_y_d_gain ;
  prev_walking_->leg_angle_feed_back_[7].p_gain_  = msg.l_leg_hip_r_p_gain ;
  prev_walking_->leg_angle_feed_back_[7].d_gain_  = msg.l_leg_hip_r_d_gain ;
  prev_walking_->leg_angle_feed_back_[8].p_gain_  = msg.l_leg_hip_p_p_gain ;
  prev_walking_->leg_angle_feed_back_[8].d_gain_  = msg.l_leg_hip_p_d_gain ;
  prev_walking_->leg_angle_feed_back_[9].p_gain_  = msg.l_leg_kn_p_p_gain  ;
  prev_walking_->leg_angle_feed_back_[9].d_gain_  = msg.l_leg_kn_p_d_gain  ;
  prev_walking_->leg_angle_feed_back_[10].p_gain_ = msg.l_leg_an_p_p_gain  ;
  prev_walking_->leg_angle_feed_back_[10].d_gain_ = msg.l_leg_an_p_d_gain  ;
  prev_walking_->leg_angle_feed_back_[11].p_gain_ = msg.l_leg_an_r_p_gain  ;
  prev_walking_->leg_angle_feed_back_[11].d_gain_ = msg.l_leg_an_r_d_gain  ;
}

void OP3PreviewWalkingModule::updateJointFeedBackGain()
{
  double current_update_gain =  joint_feedback_update_tra_.getPosition(joint_feedback_update_sys_time_);

  current_joint_feedback_gain_.r_leg_hip_y_p_gain = current_update_gain*(desired_joint_feedback_gain_.r_leg_hip_y_p_gain - previous_joint_feedback_gain_.r_leg_hip_y_p_gain ) + previous_joint_feedback_gain_.r_leg_hip_y_p_gain  ;
  current_joint_feedback_gain_.r_leg_hip_y_d_gain = current_update_gain*(desired_joint_feedback_gain_.r_leg_hip_y_d_gain - previous_joint_feedback_gain_.r_leg_hip_y_d_gain ) + previous_joint_feedback_gain_.r_leg_hip_y_d_gain  ;
  current_joint_feedback_gain_.r_leg_hip_r_p_gain = current_update_gain*(desired_joint_feedback_gain_.r_leg_hip_r_p_gain - previous_joint_feedback_gain_.r_leg_hip_r_p_gain ) + previous_joint_feedback_gain_.r_leg_hip_r_p_gain  ;
  current_joint_feedback_gain_.r_leg_hip_r_d_gain = current_update_gain*(desired_joint_feedback_gain_.r_leg_hip_r_d_gain - previous_joint_feedback_gain_.r_leg_hip_r_d_gain ) + previous_joint_feedback_gain_.r_leg_hip_r_d_gain  ;
  current_joint_feedback_gain_.r_leg_hip_p_p_gain = current_update_gain*(desired_joint_feedback_gain_.r_leg_hip_p_p_gain - previous_joint_feedback_gain_.r_leg_hip_p_p_gain ) + previous_joint_feedback_gain_.r_leg_hip_p_p_gain  ;
  current_joint_feedback_gain_.r_leg_hip_p_d_gain = current_update_gain*(desired_joint_feedback_gain_.r_leg_hip_p_d_gain - previous_joint_feedback_gain_.r_leg_hip_p_d_gain ) + previous_joint_feedback_gain_.r_leg_hip_p_d_gain  ;
  current_joint_feedback_gain_.r_leg_kn_p_p_gain  = current_update_gain*(desired_joint_feedback_gain_.r_leg_kn_p_p_gain  - previous_joint_feedback_gain_.r_leg_kn_p_p_gain  ) + previous_joint_feedback_gain_.r_leg_kn_p_p_gain   ;
  current_joint_feedback_gain_.r_leg_kn_p_d_gain  = current_update_gain*(desired_joint_feedback_gain_.r_leg_kn_p_d_gain  - previous_joint_feedback_gain_.r_leg_kn_p_d_gain  ) + previous_joint_feedback_gain_.r_leg_kn_p_d_gain   ;
  current_joint_feedback_gain_.r_leg_an_p_p_gain  = current_update_gain*(desired_joint_feedback_gain_.r_leg_an_p_p_gain  - previous_joint_feedback_gain_.r_leg_an_p_p_gain  ) + previous_joint_feedback_gain_.r_leg_an_p_p_gain   ;
  current_joint_feedback_gain_.r_leg_an_p_d_gain  = current_update_gain*(desired_joint_feedback_gain_.r_leg_an_p_d_gain  - previous_joint_feedback_gain_.r_leg_an_p_d_gain  ) + previous_joint_feedback_gain_.r_leg_an_p_d_gain   ;
  current_joint_feedback_gain_.r_leg_an_r_p_gain  = current_update_gain*(desired_joint_feedback_gain_.r_leg_an_r_p_gain  - previous_joint_feedback_gain_.r_leg_an_r_p_gain  ) + previous_joint_feedback_gain_.r_leg_an_r_p_gain   ;
  current_joint_feedback_gain_.r_leg_an_r_d_gain  = current_update_gain*(desired_joint_feedback_gain_.r_leg_an_r_d_gain  - previous_joint_feedback_gain_.r_leg_an_r_d_gain  ) + previous_joint_feedback_gain_.r_leg_an_r_d_gain   ;

  current_joint_feedback_gain_.l_leg_hip_y_p_gain = current_update_gain*(desired_joint_feedback_gain_.l_leg_hip_y_p_gain - previous_joint_feedback_gain_.l_leg_hip_y_p_gain ) + previous_joint_feedback_gain_.l_leg_hip_y_p_gain  ;
  current_joint_feedback_gain_.l_leg_hip_y_d_gain = current_update_gain*(desired_joint_feedback_gain_.l_leg_hip_y_d_gain - previous_joint_feedback_gain_.l_leg_hip_y_d_gain ) + previous_joint_feedback_gain_.l_leg_hip_y_d_gain  ;
  current_joint_feedback_gain_.l_leg_hip_r_p_gain = current_update_gain*(desired_joint_feedback_gain_.l_leg_hip_r_p_gain - previous_joint_feedback_gain_.l_leg_hip_r_p_gain ) + previous_joint_feedback_gain_.l_leg_hip_r_p_gain  ;
  current_joint_feedback_gain_.l_leg_hip_r_d_gain = current_update_gain*(desired_joint_feedback_gain_.l_leg_hip_r_d_gain - previous_joint_feedback_gain_.l_leg_hip_r_d_gain ) + previous_joint_feedback_gain_.l_leg_hip_r_d_gain  ;
  current_joint_feedback_gain_.l_leg_hip_p_p_gain = current_update_gain*(desired_joint_feedback_gain_.l_leg_hip_p_p_gain - previous_joint_feedback_gain_.l_leg_hip_p_p_gain ) + previous_joint_feedback_gain_.l_leg_hip_p_p_gain  ;
  current_joint_feedback_gain_.l_leg_hip_p_d_gain = current_update_gain*(desired_joint_feedback_gain_.l_leg_hip_p_d_gain - previous_joint_feedback_gain_.l_leg_hip_p_d_gain ) + previous_joint_feedback_gain_.l_leg_hip_p_d_gain  ;
  current_joint_feedback_gain_.l_leg_kn_p_p_gain  = current_update_gain*(desired_joint_feedback_gain_.l_leg_kn_p_p_gain  - previous_joint_feedback_gain_.l_leg_kn_p_p_gain  ) + previous_joint_feedback_gain_.l_leg_kn_p_p_gain   ;
  current_joint_feedback_gain_.l_leg_kn_p_d_gain  = current_update_gain*(desired_joint_feedback_gain_.l_leg_kn_p_d_gain  - previous_joint_feedback_gain_.l_leg_kn_p_d_gain  ) + previous_joint_feedback_gain_.l_leg_kn_p_d_gain   ;
  current_joint_feedback_gain_.l_leg_an_p_p_gain  = current_update_gain*(desired_joint_feedback_gain_.l_leg_an_p_p_gain  - previous_joint_feedback_gain_.l_leg_an_p_p_gain  ) + previous_joint_feedback_gain_.l_leg_an_p_p_gain   ;
  current_joint_feedback_gain_.l_leg_an_p_d_gain  = current_update_gain*(desired_joint_feedback_gain_.l_leg_an_p_d_gain  - previous_joint_feedback_gain_.l_leg_an_p_d_gain  ) + previous_joint_feedback_gain_.l_leg_an_p_d_gain   ;
  current_joint_feedback_gain_.l_leg_an_r_p_gain  = current_update_gain*(desired_joint_feedback_gain_.l_leg_an_r_p_gain  - previous_joint_feedback_gain_.l_leg_an_r_p_gain  ) + previous_joint_feedback_gain_.l_leg_an_r_p_gain   ;
  current_joint_feedback_gain_.l_leg_an_r_d_gain  = current_update_gain*(desired_joint_feedback_gain_.l_leg_an_r_d_gain  - previous_joint_feedback_gain_.l_leg_an_r_d_gain  ) + previous_joint_feedback_gain_.l_leg_an_r_d_gain   ;


  setJointFeedBackGain(current_joint_feedback_gain_);
}

bool OP3PreviewWalkingModule::setBalanceParamServiceCallback(adol_preview_walking_module_msgs::SetBalanceParam::Request  &req,
    adol_preview_walking_module_msgs::SetBalanceParam::Response &res)
{
  res.result = adol_preview_walking_module_msgs::SetBalanceParam::Response::NO_ERROR;

  if( enable_ == false)
    res.result |= adol_preview_walking_module_msgs::SetBalanceParam::Response::NOT_ENABLED_WALKING_MODULE;

  if( balance_update_with_loop_ == true)
  {
    res.result |= adol_preview_walking_module_msgs::SetBalanceParam::Response::PREV_REQUEST_IS_NOT_FINISHED;
  }

  if( (req.balance_param.roll_gyro_cut_off_frequency         <= 0) ||
      (req.balance_param.pitch_gyro_cut_off_frequency        <= 0) ||
      (req.balance_param.roll_angle_cut_off_frequency        <= 0) ||
      (req.balance_param.pitch_angle_cut_off_frequency       <= 0) ||
      (req.balance_param.foot_x_force_cut_off_frequency      <= 0) ||
      (req.balance_param.foot_y_force_cut_off_frequency      <= 0) ||
      (req.balance_param.foot_z_force_cut_off_frequency      <= 0) ||
      (req.balance_param.foot_roll_torque_cut_off_frequency  <= 0) ||
      (req.balance_param.foot_pitch_torque_cut_off_frequency <= 0) )
  {
    //res.result |= thormang3_walking_module_msgs::SetBalanceParam::Response::CUT_OFF_FREQUENCY_IS_ZERO_OR_NEGATIVE;
    previous_balance_param_.roll_gyro_cut_off_frequency         = req.balance_param.roll_gyro_cut_off_frequency;
    previous_balance_param_.pitch_gyro_cut_off_frequency        = req.balance_param.pitch_gyro_cut_off_frequency;
    previous_balance_param_.roll_angle_cut_off_frequency        = req.balance_param.roll_angle_cut_off_frequency;
    previous_balance_param_.pitch_angle_cut_off_frequency       = req.balance_param.pitch_angle_cut_off_frequency;
    previous_balance_param_.foot_x_force_cut_off_frequency      = req.balance_param.foot_x_force_cut_off_frequency;
    previous_balance_param_.foot_y_force_cut_off_frequency      = req.balance_param.foot_y_force_cut_off_frequency;
    previous_balance_param_.foot_z_force_cut_off_frequency      = req.balance_param.foot_z_force_cut_off_frequency;
    previous_balance_param_.foot_roll_torque_cut_off_frequency  = req.balance_param.foot_roll_torque_cut_off_frequency;
    previous_balance_param_.foot_pitch_torque_cut_off_frequency = req.balance_param.foot_pitch_torque_cut_off_frequency;
  }

  if(res.result != adol_preview_walking_module_msgs::SetBalanceParam::Response::NO_ERROR)
  {
    publishDoneMsg("walking_balance_failed");
    return true;
  }

  if( req.updating_duration <= 0.0 )
  {
    // under 8ms apply immediately
    setBalanceParam(req.balance_param);
    std::string status_msg = WalkingStatusMSG::BALANCE_PARAM_SETTING_FINISHED_MSG;
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
    publishDoneMsg("walking_balance");
    return true;
  }
  else
  {
    balance_update_duration_ = req.updating_duration;
  }

  balance_update_sys_time_ = 0.0;
  balance_update_tra_.changeTrajectory(0, 0, 0, 0, balance_update_duration_, 1.0, 0, 0);


  desired_balance_param_ = req.balance_param;

  // previous_balance_param_.cob_x_offset_m                  = prev_walking_->balance_ctrl_.getCOBManualAdjustmentX();
  // previous_balance_param_.cob_y_offset_m                  = prev_walking_->balance_ctrl_.getCOBManualAdjustmentY();

  // ////gain
  // //gyro
  // previous_balance_param_.foot_roll_gyro_p_gain           = prev_walking_->balance_ctrl_.foot_roll_gyro_ctrl_.p_gain_;
  // previous_balance_param_.foot_roll_gyro_d_gain           = prev_walking_->balance_ctrl_.foot_roll_gyro_ctrl_.d_gain_;
  // previous_balance_param_.foot_pitch_gyro_p_gain          = prev_walking_->balance_ctrl_.foot_pitch_gyro_ctrl_.p_gain_;
  // previous_balance_param_.foot_pitch_gyro_d_gain          = prev_walking_->balance_ctrl_.foot_pitch_gyro_ctrl_.d_gain_;

  // //orientation
  // previous_balance_param_.foot_roll_angle_p_gain          = prev_walking_->balance_ctrl_.foot_roll_angle_ctrl_.p_gain_;
  // previous_balance_param_.foot_roll_angle_d_gain          = prev_walking_->balance_ctrl_.foot_roll_angle_ctrl_.d_gain_;
  // previous_balance_param_.foot_pitch_angle_p_gain         = prev_walking_->balance_ctrl_.foot_pitch_angle_ctrl_.p_gain_;
  // previous_balance_param_.foot_pitch_angle_d_gain         = prev_walking_->balance_ctrl_.foot_pitch_angle_ctrl_.d_gain_;

  // //force torque
  // previous_balance_param_.foot_x_force_p_gain               = prev_walking_->balance_ctrl_.right_foot_force_x_ctrl_.p_gain_;
  // previous_balance_param_.foot_y_force_p_gain               = prev_walking_->balance_ctrl_.right_foot_force_y_ctrl_.p_gain_;
  // previous_balance_param_.foot_z_force_p_gain               = prev_walking_->balance_ctrl_.right_foot_force_z_ctrl_.p_gain_;
  // previous_balance_param_.foot_roll_torque_p_gain           = prev_walking_->balance_ctrl_.right_foot_torque_roll_ctrl_.p_gain_;
  // previous_balance_param_.foot_pitch_torque_p_gain          = prev_walking_->balance_ctrl_.right_foot_torque_pitch_ctrl_.p_gain_;

  // previous_balance_param_.foot_x_force_d_gain               = prev_walking_->balance_ctrl_.right_foot_force_x_ctrl_.d_gain_;
  // previous_balance_param_.foot_y_force_d_gain               = prev_walking_->balance_ctrl_.right_foot_force_y_ctrl_.d_gain_;
  // previous_balance_param_.foot_z_force_d_gain               = prev_walking_->balance_ctrl_.right_foot_force_z_ctrl_.d_gain_;
  // previous_balance_param_.foot_roll_torque_d_gain           = prev_walking_->balance_ctrl_.right_foot_torque_roll_ctrl_.d_gain_;
  // previous_balance_param_.foot_pitch_torque_d_gain          = prev_walking_->balance_ctrl_.right_foot_torque_pitch_ctrl_.d_gain_;

  // ////cut off freq
  // //gyro
  // previous_balance_param_.roll_gyro_cut_off_frequency  = prev_walking_->balance_ctrl_.roll_gyro_lpf_.getCutOffFrequency();
  // previous_balance_param_.pitch_gyro_cut_off_frequency = prev_walking_->balance_ctrl_.pitch_gyro_lpf_.getCutOffFrequency();

  // //orientation
  // previous_balance_param_.roll_angle_cut_off_frequency  = prev_walking_->balance_ctrl_.roll_angle_lpf_.getCutOffFrequency();
  // previous_balance_param_.pitch_angle_cut_off_frequency = prev_walking_->balance_ctrl_.pitch_angle_lpf_.getCutOffFrequency();

  // //force torque
  // previous_balance_param_.foot_x_force_cut_off_frequency     = prev_walking_->balance_ctrl_.right_foot_force_x_lpf_.getCutOffFrequency();
  // previous_balance_param_.foot_y_force_cut_off_frequency     = prev_walking_->balance_ctrl_.right_foot_force_y_lpf_.getCutOffFrequency();
  // previous_balance_param_.foot_z_force_cut_off_frequency     = prev_walking_->balance_ctrl_.right_foot_force_z_lpf_.getCutOffFrequency();
  // previous_balance_param_.foot_roll_torque_cut_off_frequency  = prev_walking_->balance_ctrl_.right_foot_torque_roll_lpf_.getCutOffFrequency();
  // previous_balance_param_.foot_pitch_torque_cut_off_frequency = prev_walking_->balance_ctrl_.right_foot_torque_pitch_lpf_.getCutOffFrequency();

  balance_update_with_loop_ = true;

  std::string status_msg = WalkingStatusMSG::BALANCE_PARAM_SETTING_STARTED_MSG;
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);

  return true;
}

void OP3PreviewWalkingModule::setBalanceParam(adol_preview_walking_module_msgs::BalanceParam& balance_param_msg)
{
  // prev_walking_->balance_ctrl_.setCOBManualAdjustment(balance_param_msg.cob_x_offset_m, balance_param_msg.cob_y_offset_m, 0);

  // //// set gain
  // //gyro
  // prev_walking_->balance_ctrl_.foot_roll_gyro_ctrl_.p_gain_ = balance_param_msg.foot_roll_gyro_p_gain;
  // prev_walking_->balance_ctrl_.foot_roll_gyro_ctrl_.d_gain_ = balance_param_msg.foot_roll_gyro_d_gain;
  // prev_walking_->balance_ctrl_.foot_pitch_gyro_ctrl_.p_gain_ = balance_param_msg.foot_pitch_gyro_p_gain;
  // prev_walking_->balance_ctrl_.foot_pitch_gyro_ctrl_.d_gain_ = balance_param_msg.foot_pitch_gyro_d_gain;

  // //orientation
  // prev_walking_->balance_ctrl_.foot_roll_angle_ctrl_.p_gain_  = balance_param_msg.foot_roll_angle_p_gain;
  // prev_walking_->balance_ctrl_.foot_roll_angle_ctrl_.d_gain_  = balance_param_msg.foot_roll_angle_d_gain;
  // prev_walking_->balance_ctrl_.foot_pitch_angle_ctrl_.p_gain_ = balance_param_msg.foot_pitch_angle_p_gain;
  // prev_walking_->balance_ctrl_.foot_pitch_angle_ctrl_.d_gain_ = balance_param_msg.foot_pitch_angle_d_gain;

  // //force torque
  // prev_walking_->balance_ctrl_.right_foot_force_x_ctrl_.p_gain_      = balance_param_msg.foot_x_force_p_gain;
  // prev_walking_->balance_ctrl_.right_foot_force_y_ctrl_.p_gain_      = balance_param_msg.foot_y_force_p_gain;
  // prev_walking_->balance_ctrl_.right_foot_force_z_ctrl_.p_gain_      = balance_param_msg.foot_z_force_p_gain;
  // prev_walking_->balance_ctrl_.right_foot_torque_roll_ctrl_.p_gain_  = balance_param_msg.foot_roll_torque_p_gain;
  // prev_walking_->balance_ctrl_.right_foot_torque_pitch_ctrl_.p_gain_ = balance_param_msg.foot_roll_torque_p_gain;
  // prev_walking_->balance_ctrl_.right_foot_force_x_ctrl_.d_gain_      = balance_param_msg.foot_x_force_d_gain;
  // prev_walking_->balance_ctrl_.right_foot_force_y_ctrl_.d_gain_      = balance_param_msg.foot_y_force_d_gain;
  // prev_walking_->balance_ctrl_.right_foot_force_z_ctrl_.d_gain_      = balance_param_msg.foot_z_force_d_gain;
  // prev_walking_->balance_ctrl_.right_foot_torque_roll_ctrl_.d_gain_  = balance_param_msg.foot_roll_torque_d_gain;
  // prev_walking_->balance_ctrl_.right_foot_torque_pitch_ctrl_.d_gain_ = balance_param_msg.foot_roll_torque_d_gain;

  // prev_walking_->balance_ctrl_.left_foot_force_x_ctrl_.p_gain_      = balance_param_msg.foot_x_force_p_gain;
  // prev_walking_->balance_ctrl_.left_foot_force_y_ctrl_.p_gain_      = balance_param_msg.foot_y_force_p_gain;
  // prev_walking_->balance_ctrl_.left_foot_force_z_ctrl_.p_gain_      = balance_param_msg.foot_z_force_p_gain;
  // prev_walking_->balance_ctrl_.left_foot_torque_roll_ctrl_.p_gain_  = balance_param_msg.foot_roll_torque_p_gain;
  // prev_walking_->balance_ctrl_.left_foot_torque_pitch_ctrl_.p_gain_ = balance_param_msg.foot_roll_torque_p_gain;
  // prev_walking_->balance_ctrl_.left_foot_force_x_ctrl_.d_gain_      = balance_param_msg.foot_x_force_d_gain;
  // prev_walking_->balance_ctrl_.left_foot_force_y_ctrl_.d_gain_      = balance_param_msg.foot_y_force_d_gain;
  // prev_walking_->balance_ctrl_.left_foot_force_z_ctrl_.d_gain_      = balance_param_msg.foot_z_force_d_gain;
  // prev_walking_->balance_ctrl_.left_foot_torque_roll_ctrl_.d_gain_  = balance_param_msg.foot_roll_torque_d_gain;
  // prev_walking_->balance_ctrl_.left_foot_torque_pitch_ctrl_.d_gain_ = balance_param_msg.foot_roll_torque_d_gain;

  // //// set cut off freq
  // prev_walking_->balance_ctrl_.roll_gyro_lpf_.setCutOffFrequency(balance_param_msg.roll_gyro_cut_off_frequency);
  // prev_walking_->balance_ctrl_.pitch_gyro_lpf_.setCutOffFrequency(balance_param_msg.pitch_gyro_cut_off_frequency);
  // prev_walking_->balance_ctrl_.roll_angle_lpf_.setCutOffFrequency(balance_param_msg.roll_angle_cut_off_frequency);
  // prev_walking_->balance_ctrl_.pitch_angle_lpf_.setCutOffFrequency(balance_param_msg.pitch_angle_cut_off_frequency);

  // prev_walking_->balance_ctrl_.right_foot_force_x_lpf_.setCutOffFrequency(balance_param_msg.foot_x_force_cut_off_frequency);
  // prev_walking_->balance_ctrl_.right_foot_force_y_lpf_.setCutOffFrequency(balance_param_msg.foot_y_force_cut_off_frequency);
  // prev_walking_->balance_ctrl_.right_foot_force_z_lpf_.setCutOffFrequency(balance_param_msg.foot_z_force_cut_off_frequency);
  // prev_walking_->balance_ctrl_.right_foot_torque_roll_lpf_.setCutOffFrequency(balance_param_msg.foot_roll_torque_cut_off_frequency);
  // prev_walking_->balance_ctrl_.right_foot_torque_pitch_lpf_.setCutOffFrequency(balance_param_msg.foot_pitch_torque_cut_off_frequency);

  // prev_walking_->balance_ctrl_.left_foot_force_x_lpf_.setCutOffFrequency(balance_param_msg.foot_x_force_cut_off_frequency);
  // prev_walking_->balance_ctrl_.left_foot_force_y_lpf_.setCutOffFrequency(balance_param_msg.foot_y_force_cut_off_frequency);
  // prev_walking_->balance_ctrl_.left_foot_force_z_lpf_.setCutOffFrequency(balance_param_msg.foot_z_force_cut_off_frequency);
  // prev_walking_->balance_ctrl_.left_foot_torque_roll_lpf_.setCutOffFrequency(balance_param_msg.foot_roll_torque_cut_off_frequency);
  // prev_walking_->balance_ctrl_.left_foot_torque_pitch_lpf_.setCutOffFrequency(balance_param_msg.foot_pitch_torque_cut_off_frequency);
}

void OP3PreviewWalkingModule::updateBalanceParam()
{
  double current_update_gain = balance_update_tra_.getPosition(balance_update_sys_time_);

  current_balance_param_.cob_x_offset_m                  = current_update_gain*(desired_balance_param_.cob_x_offset_m                   - previous_balance_param_.cob_x_offset_m                ) + previous_balance_param_.cob_x_offset_m;
  current_balance_param_.cob_y_offset_m                  = current_update_gain*(desired_balance_param_.cob_y_offset_m                   - previous_balance_param_.cob_y_offset_m                ) + previous_balance_param_.cob_y_offset_m;

  current_balance_param_.foot_roll_gyro_p_gain                = current_update_gain*(desired_balance_param_.foot_roll_gyro_p_gain                - previous_balance_param_.foot_roll_gyro_p_gain              ) + previous_balance_param_.foot_roll_gyro_p_gain;
  current_balance_param_.foot_roll_gyro_d_gain                = current_update_gain*(desired_balance_param_.foot_roll_gyro_d_gain                - previous_balance_param_.foot_roll_gyro_d_gain              ) + previous_balance_param_.foot_roll_gyro_d_gain;
  current_balance_param_.foot_pitch_gyro_p_gain               = current_update_gain*(desired_balance_param_.foot_pitch_gyro_p_gain               - previous_balance_param_.foot_pitch_gyro_p_gain             ) + previous_balance_param_.foot_pitch_gyro_p_gain;
  current_balance_param_.foot_pitch_gyro_d_gain               = current_update_gain*(desired_balance_param_.foot_pitch_gyro_d_gain               - previous_balance_param_.foot_pitch_gyro_d_gain             ) + previous_balance_param_.foot_pitch_gyro_d_gain;
  current_balance_param_.foot_roll_angle_p_gain               = current_update_gain*(desired_balance_param_.foot_roll_angle_p_gain               - previous_balance_param_.foot_roll_angle_p_gain             ) + previous_balance_param_.foot_roll_angle_p_gain;
  current_balance_param_.foot_roll_angle_d_gain               = current_update_gain*(desired_balance_param_.foot_roll_angle_d_gain               - previous_balance_param_.foot_roll_angle_d_gain             ) + previous_balance_param_.foot_roll_angle_d_gain;
  current_balance_param_.foot_pitch_angle_p_gain              = current_update_gain*(desired_balance_param_.foot_pitch_angle_p_gain              - previous_balance_param_.foot_pitch_angle_p_gain            ) + previous_balance_param_.foot_pitch_angle_p_gain;
  current_balance_param_.foot_pitch_angle_d_gain              = current_update_gain*(desired_balance_param_.foot_pitch_angle_d_gain              - previous_balance_param_.foot_pitch_angle_d_gain            ) + previous_balance_param_.foot_pitch_angle_d_gain;
  current_balance_param_.foot_x_force_p_gain                  = current_update_gain*(desired_balance_param_.foot_x_force_p_gain                  - previous_balance_param_.foot_x_force_p_gain                ) + previous_balance_param_.foot_x_force_p_gain;
  current_balance_param_.foot_y_force_p_gain                  = current_update_gain*(desired_balance_param_.foot_y_force_p_gain                  - previous_balance_param_.foot_y_force_p_gain                ) + previous_balance_param_.foot_y_force_p_gain;
  current_balance_param_.foot_z_force_p_gain                  = current_update_gain*(desired_balance_param_.foot_z_force_p_gain                  - previous_balance_param_.foot_z_force_p_gain                ) + previous_balance_param_.foot_z_force_p_gain;
  current_balance_param_.foot_roll_torque_p_gain              = current_update_gain*(desired_balance_param_.foot_roll_torque_p_gain              - previous_balance_param_.foot_roll_torque_p_gain            ) + previous_balance_param_.foot_roll_torque_p_gain;
  current_balance_param_.foot_pitch_torque_p_gain             = current_update_gain*(desired_balance_param_.foot_pitch_torque_p_gain             - previous_balance_param_.foot_pitch_torque_p_gain           ) + previous_balance_param_.foot_pitch_torque_p_gain;
  current_balance_param_.foot_x_force_d_gain                  = current_update_gain*(desired_balance_param_.foot_x_force_d_gain                  - previous_balance_param_.foot_x_force_d_gain                ) + previous_balance_param_.foot_x_force_d_gain;
  current_balance_param_.foot_y_force_d_gain                  = current_update_gain*(desired_balance_param_.foot_y_force_d_gain                  - previous_balance_param_.foot_y_force_d_gain                ) + previous_balance_param_.foot_y_force_d_gain;
  current_balance_param_.foot_z_force_d_gain                  = current_update_gain*(desired_balance_param_.foot_z_force_d_gain                  - previous_balance_param_.foot_z_force_d_gain                ) + previous_balance_param_.foot_z_force_d_gain;
  current_balance_param_.foot_roll_torque_d_gain              = current_update_gain*(desired_balance_param_.foot_roll_torque_d_gain              - previous_balance_param_.foot_roll_torque_d_gain            ) + previous_balance_param_.foot_roll_torque_d_gain;
  current_balance_param_.foot_pitch_torque_d_gain             = current_update_gain*(desired_balance_param_.foot_pitch_torque_d_gain             - previous_balance_param_.foot_pitch_torque_d_gain           ) + previous_balance_param_.foot_pitch_torque_d_gain;

  current_balance_param_.roll_gyro_cut_off_frequency          = current_update_gain*(desired_balance_param_.roll_gyro_cut_off_frequency          - previous_balance_param_.roll_gyro_cut_off_frequency        ) + previous_balance_param_.roll_gyro_cut_off_frequency;
  current_balance_param_.pitch_gyro_cut_off_frequency         = current_update_gain*(desired_balance_param_.pitch_gyro_cut_off_frequency         - previous_balance_param_.pitch_gyro_cut_off_frequency       ) + previous_balance_param_.pitch_gyro_cut_off_frequency;
  current_balance_param_.roll_angle_cut_off_frequency         = current_update_gain*(desired_balance_param_.roll_angle_cut_off_frequency         - previous_balance_param_.roll_angle_cut_off_frequency       ) + previous_balance_param_.roll_angle_cut_off_frequency;
  current_balance_param_.pitch_angle_cut_off_frequency        = current_update_gain*(desired_balance_param_.pitch_angle_cut_off_frequency        - previous_balance_param_.pitch_angle_cut_off_frequency      ) + previous_balance_param_.pitch_angle_cut_off_frequency;
  current_balance_param_.foot_x_force_cut_off_frequency       = current_update_gain*(desired_balance_param_.foot_x_force_cut_off_frequency       - previous_balance_param_.foot_x_force_cut_off_frequency     ) + previous_balance_param_.foot_x_force_cut_off_frequency;
  current_balance_param_.foot_y_force_cut_off_frequency       = current_update_gain*(desired_balance_param_.foot_y_force_cut_off_frequency       - previous_balance_param_.foot_y_force_cut_off_frequency     ) + previous_balance_param_.foot_y_force_cut_off_frequency;
  current_balance_param_.foot_z_force_cut_off_frequency       = current_update_gain*(desired_balance_param_.foot_z_force_cut_off_frequency       - previous_balance_param_.foot_z_force_cut_off_frequency     ) + previous_balance_param_.foot_z_force_cut_off_frequency;
  current_balance_param_.foot_roll_torque_cut_off_frequency   = current_update_gain*(desired_balance_param_.foot_roll_torque_cut_off_frequency   - previous_balance_param_.foot_roll_torque_cut_off_frequency ) + previous_balance_param_.foot_roll_torque_cut_off_frequency;
  current_balance_param_.foot_pitch_torque_cut_off_frequency  = current_update_gain*(desired_balance_param_.foot_pitch_torque_cut_off_frequency  - previous_balance_param_.foot_pitch_torque_cut_off_frequency) + previous_balance_param_.foot_pitch_torque_cut_off_frequency;

  setBalanceParam(current_balance_param_);
}

bool OP3PreviewWalkingModule::checkBalanceOnOff()
{
  return true;

  if(gazebo_)
    return true;

  // if ((fabs(prev_walking_->balance_ctrl_.foot_roll_gyro_ctrl_.p_gain_           ) < 1e-7) &&
  //     (fabs(prev_walking_->balance_ctrl_.foot_roll_gyro_ctrl_.d_gain_           ) < 1e-7) &&
  //     (fabs(prev_walking_->balance_ctrl_.foot_pitch_gyro_ctrl_.p_gain_          ) < 1e-7) &&
  //     (fabs(prev_walking_->balance_ctrl_.foot_pitch_gyro_ctrl_.d_gain_          ) < 1e-7) &&
  //     (fabs(prev_walking_->balance_ctrl_.foot_roll_angle_ctrl_.p_gain_          ) < 1e-7) &&
  //     (fabs(prev_walking_->balance_ctrl_.foot_roll_angle_ctrl_.d_gain_          ) < 1e-7) &&
  //     (fabs(prev_walking_->balance_ctrl_.foot_pitch_angle_ctrl_.p_gain_         ) < 1e-7) &&
  //     (fabs(prev_walking_->balance_ctrl_.foot_pitch_angle_ctrl_.d_gain_         ) < 1e-7) &&
  //     (fabs(prev_walking_->balance_ctrl_.right_foot_force_x_ctrl_.p_gain_       ) < 1e-7) &&
  //     (fabs(prev_walking_->balance_ctrl_.right_foot_force_y_ctrl_.p_gain_       ) < 1e-7) &&
  //     (fabs(prev_walking_->balance_ctrl_.right_foot_force_z_ctrl_.p_gain_       ) < 1e-7) &&
  //     (fabs(prev_walking_->balance_ctrl_.right_foot_torque_roll_ctrl_.p_gain_   ) < 1e-7) &&
  //     (fabs(prev_walking_->balance_ctrl_.right_foot_torque_pitch_ctrl_.p_gain_  ) < 1e-7) &&
  //     (fabs(prev_walking_->balance_ctrl_.right_foot_force_x_ctrl_.d_gain_       ) < 1e-7) &&
  //     (fabs(prev_walking_->balance_ctrl_.right_foot_force_y_ctrl_.d_gain_       ) < 1e-7) &&
  //     (fabs(prev_walking_->balance_ctrl_.right_foot_force_z_ctrl_.d_gain_       ) < 1e-7) &&
  //     (fabs(prev_walking_->balance_ctrl_.right_foot_torque_roll_ctrl_.d_gain_   ) < 1e-7) &&
  //     (fabs(prev_walking_->balance_ctrl_.right_foot_torque_pitch_ctrl_.d_gain_  ) < 1e-7) &&
  //     (fabs(prev_walking_->balance_ctrl_.left_foot_force_x_ctrl_.p_gain_        ) < 1e-7) &&
  //     (fabs(prev_walking_->balance_ctrl_.left_foot_force_y_ctrl_.p_gain_        ) < 1e-7) &&
  //     (fabs(prev_walking_->balance_ctrl_.left_foot_force_z_ctrl_.p_gain_        ) < 1e-7) &&
  //     (fabs(prev_walking_->balance_ctrl_.left_foot_torque_roll_ctrl_.p_gain_    ) < 1e-7) &&
  //     (fabs(prev_walking_->balance_ctrl_.left_foot_torque_pitch_ctrl_.p_gain_   ) < 1e-7) &&
  //     (fabs(prev_walking_->balance_ctrl_.left_foot_force_x_ctrl_.d_gain_        ) < 1e-7) &&
  //     (fabs(prev_walking_->balance_ctrl_.left_foot_force_y_ctrl_.d_gain_        ) < 1e-7) &&
  //     (fabs(prev_walking_->balance_ctrl_.left_foot_force_z_ctrl_.d_gain_        ) < 1e-7) &&
  //     (fabs(prev_walking_->balance_ctrl_.left_foot_torque_roll_ctrl_.d_gain_    ) < 1e-7) &&
  //     (fabs(prev_walking_->balance_ctrl_.left_foot_torque_pitch_ctrl_.d_gain_   ) < 1e-7))
  // {
  //   return false;
  // }
  // else
  //   return true;
}

// void OP3PreviewWalkingModule::ftDataOutputCallback(const diana_msgs::ForceTorque::ConstPtr &msg)
// {
//   ALICEOnlineWalking *online_walking = ALICEOnlineWalking::getInstance();

//   online_walking->setCurrentFTSensorOutput(msg->force_x_raw_r, msg->force_y_raw_r, msg->force_z_raw_r,
//       msg->torque_x_raw_r, msg->torque_y_raw_r, msg->torque_z_raw_r,
//       msg->force_x_raw_l, msg->force_y_raw_l, msg->force_z_raw_l,
//       msg->torque_x_raw_l, msg->torque_y_raw_l, msg->torque_z_raw_l);
// }


void OP3PreviewWalkingModule::onModuleEnable()
{
  std::string status_msg = WalkingStatusMSG::WALKING_MODULE_IS_ENABLED_MSG;
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
}

void OP3PreviewWalkingModule::onModuleDisable()
{
  previous_running_ = present_running = false;

  std::string status_msg = WalkingStatusMSG::WALKING_MODULE_IS_DISABLED_MSG;
  balance_update_with_loop_ = false;

  // prev_walking_->leg_angle_feed_back_[0].p_gain_ = 0;
  // prev_walking_->leg_angle_feed_back_[0].d_gain_ = 0;
  // prev_walking_->leg_angle_feed_back_[1].p_gain_ = 0;
  // prev_walking_->leg_angle_feed_back_[1].d_gain_ = 0;
  // prev_walking_->leg_angle_feed_back_[2].p_gain_ = 0;
  // prev_walking_->leg_angle_feed_back_[2].d_gain_ = 0;
  // prev_walking_->leg_angle_feed_back_[3].p_gain_ = 0;
  // prev_walking_->leg_angle_feed_back_[3].d_gain_ = 0;
  // prev_walking_->leg_angle_feed_back_[4].p_gain_ = 0;
  // prev_walking_->leg_angle_feed_back_[4].d_gain_ = 0;
  // prev_walking_->leg_angle_feed_back_[5].p_gain_ = 0;
  // prev_walking_->leg_angle_feed_back_[5].d_gain_ = 0;

  // prev_walking_->leg_angle_feed_back_[6].p_gain_ = 0;
  // prev_walking_->leg_angle_feed_back_[6].d_gain_ = 0;
  // prev_walking_->leg_angle_feed_back_[7].p_gain_ = 0;
  // prev_walking_->leg_angle_feed_back_[7].d_gain_ = 0;
  // prev_walking_->leg_angle_feed_back_[8].p_gain_ = 0;
  // prev_walking_->leg_angle_feed_back_[8].d_gain_ = 0;
  // prev_walking_->leg_angle_feed_back_[9].p_gain_ = 0;
  // prev_walking_->leg_angle_feed_back_[9].d_gain_ = 0;
  // prev_walking_->leg_angle_feed_back_[10].p_gain_ = 0;
  // prev_walking_->leg_angle_feed_back_[10].d_gain_ = 0;
  // prev_walking_->leg_angle_feed_back_[11].p_gain_ = 0;
  // prev_walking_->leg_angle_feed_back_[11].d_gain_ = 0;

  // prev_walking_->balance_ctrl_.foot_roll_gyro_ctrl_.p_gain_           = 0;
  // prev_walking_->balance_ctrl_.foot_roll_gyro_ctrl_.d_gain_           = 0;
  // prev_walking_->balance_ctrl_.foot_pitch_gyro_ctrl_.p_gain_          = 0;
  // prev_walking_->balance_ctrl_.foot_pitch_gyro_ctrl_.d_gain_          = 0;
  // prev_walking_->balance_ctrl_.foot_roll_angle_ctrl_.p_gain_          = 0;
  // prev_walking_->balance_ctrl_.foot_roll_angle_ctrl_.d_gain_          = 0;
  // prev_walking_->balance_ctrl_.foot_pitch_angle_ctrl_.p_gain_         = 0;
  // prev_walking_->balance_ctrl_.foot_pitch_angle_ctrl_.d_gain_         = 0;
  // prev_walking_->balance_ctrl_.right_foot_force_x_ctrl_.p_gain_       = 0;
  // prev_walking_->balance_ctrl_.right_foot_force_y_ctrl_.p_gain_       = 0;
  // prev_walking_->balance_ctrl_.right_foot_force_z_ctrl_.p_gain_       = 0;
  // prev_walking_->balance_ctrl_.right_foot_torque_roll_ctrl_.p_gain_   = 0;
  // prev_walking_->balance_ctrl_.right_foot_torque_pitch_ctrl_.p_gain_  = 0;
  // prev_walking_->balance_ctrl_.right_foot_force_x_ctrl_.d_gain_       = 0;
  // prev_walking_->balance_ctrl_.right_foot_force_y_ctrl_.d_gain_       = 0;
  // prev_walking_->balance_ctrl_.right_foot_force_z_ctrl_.d_gain_       = 0;
  // prev_walking_->balance_ctrl_.right_foot_torque_roll_ctrl_.d_gain_   = 0;
  // prev_walking_->balance_ctrl_.right_foot_torque_pitch_ctrl_.d_gain_  = 0;
  // prev_walking_->balance_ctrl_.left_foot_force_x_ctrl_.p_gain_        = 0;
  // prev_walking_->balance_ctrl_.left_foot_force_y_ctrl_.p_gain_        = 0;
  // prev_walking_->balance_ctrl_.left_foot_force_z_ctrl_.p_gain_        = 0;
  // prev_walking_->balance_ctrl_.left_foot_torque_roll_ctrl_.p_gain_    = 0;
  // prev_walking_->balance_ctrl_.left_foot_torque_pitch_ctrl_.p_gain_   = 0;
  // prev_walking_->balance_ctrl_.left_foot_force_x_ctrl_.d_gain_        = 0;
  // prev_walking_->balance_ctrl_.left_foot_force_y_ctrl_.d_gain_        = 0;
  // prev_walking_->balance_ctrl_.left_foot_force_z_ctrl_.d_gain_        = 0;
  // prev_walking_->balance_ctrl_.left_foot_torque_roll_ctrl_.d_gain_    = 0;
  // prev_walking_->balance_ctrl_.left_foot_torque_pitch_ctrl_.d_gain_   = 0;

  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
}

void OP3PreviewWalkingModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors)
{
  if(enable_ == false)
    return;

//  r_foot_fx_N_  = sensors["r_foot_fx_scaled_N"];
//  r_foot_fy_N_  = sensors["r_foot_fy_scaled_N"];
//  r_foot_fz_N_  = sensors["r_foot_fz_scaled_N"];
//  r_foot_Tx_Nm_ = sensors["r_foot_tx_scaled_Nm"];
//  r_foot_Ty_Nm_ = sensors["r_foot_ty_scaled_Nm"];
//  r_foot_Tz_Nm_ = sensors["r_foot_tz_scaled_Nm"];
//
//  l_foot_fx_N_  = sensors["l_foot_fx_scaled_N"];
//  l_foot_fy_N_  = sensors["l_foot_fy_scaled_N"];
//  l_foot_fz_N_  = sensors["l_foot_fz_scaled_N"];
//  l_foot_Tx_Nm_ = sensors["l_foot_tx_scaled_Nm"];
//  l_foot_Ty_Nm_ = sensors["l_foot_ty_scaled_Nm"];
//  l_foot_Tz_Nm_ = sensors["l_foot_tz_scaled_Nm"];
//
//
//  r_foot_fx_N_ = robotis_framework::sign(r_foot_fx_N_) * fmin( fabs(r_foot_fx_N_), 2000.0);
//  r_foot_fy_N_ = robotis_framework::sign(r_foot_fy_N_) * fmin( fabs(r_foot_fy_N_), 2000.0);
//  r_foot_fz_N_ = robotis_framework::sign(r_foot_fz_N_) * fmin( fabs(r_foot_fz_N_), 2000.0);
//  r_foot_Tx_Nm_ = robotis_framework::sign(r_foot_Tx_Nm_) *fmin(fabs(r_foot_Tx_Nm_), 300.0);
//  r_foot_Ty_Nm_ = robotis_framework::sign(r_foot_Ty_Nm_) *fmin(fabs(r_foot_Ty_Nm_), 300.0);
//  r_foot_Tz_Nm_ = robotis_framework::sign(r_foot_Tz_Nm_) *fmin(fabs(r_foot_Tz_Nm_), 300.0);
//
//  l_foot_fx_N_ = robotis_framework::sign(l_foot_fx_N_) * fmin( fabs(l_foot_fx_N_), 2000.0);
//  l_foot_fy_N_ = robotis_framework::sign(l_foot_fy_N_) * fmin( fabs(l_foot_fy_N_), 2000.0);
//  l_foot_fz_N_ = robotis_framework::sign(l_foot_fz_N_) * fmin( fabs(l_foot_fz_N_), 2000.0);
//  l_foot_Tx_Nm_ = robotis_framework::sign(l_foot_Tx_Nm_) *fmin(fabs(l_foot_Tx_Nm_), 300.0);
//  l_foot_Ty_Nm_ = robotis_framework::sign(l_foot_Ty_Nm_) *fmin(fabs(l_foot_Ty_Nm_), 300.0);
//  l_foot_Tz_Nm_ = robotis_framework::sign(l_foot_Tz_Nm_) *fmin(fabs(l_foot_Tz_Nm_), 300.0);


  if(balance_update_with_loop_ == true)
  {
    balance_update_sys_time_ += control_cycle_msec_ * 0.001;
    if(balance_update_sys_time_ >= balance_update_duration_ )
    {
      balance_update_sys_time_ = balance_update_duration_;
      balance_update_with_loop_ = false;
      setBalanceParam(desired_balance_param_);
      std::string status_msg = WalkingStatusMSG::BALANCE_PARAM_SETTING_FINISHED_MSG;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
      publishDoneMsg("walking_balance");
    }
    else
    {
      updateBalanceParam();
    }
  }

  if(joint_feedback_update_with_loop_ == true)
  {
    joint_feedback_update_sys_time_ += control_cycle_msec_ * 0.001;
    if(joint_feedback_update_sys_time_ >= joint_feedback_update_duration_ )
    {
      joint_feedback_update_sys_time_ = joint_feedback_update_duration_;
      joint_feedback_update_with_loop_ = false;
      setJointFeedBackGain(desired_joint_feedback_gain_);
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, WalkingStatusMSG::JOINT_FEEDBACK_GAIN_UPDATE_FINISHED_MSG);
      publishDoneMsg("walking_joint_feedback");
    }
    else
    {
      updateJointFeedBackGain();
    }
  }

//  online_walking->current_right_fx_N_  = r_foot_fx_N_;
//  online_walking->current_right_fy_N_  = r_foot_fy_N_;
//  online_walking->current_right_fz_N_  = r_foot_fz_N_;
//  online_walking->current_right_tx_Nm_ = r_foot_Tx_Nm_;
//  online_walking->current_right_ty_Nm_ = r_foot_Ty_Nm_;
//  online_walking->current_right_tz_Nm_ = r_foot_Tz_Nm_;
//
//  online_walking->current_left_fx_N_  = l_foot_fx_N_;
//  online_walking->current_left_fy_N_  = l_foot_fy_N_;
//  online_walking->current_left_fz_N_  = l_foot_fz_N_;
//  online_walking->current_left_tx_Nm_ = l_foot_Tx_Nm_;
//  online_walking->current_left_ty_Nm_ = l_foot_Ty_Nm_;
//  online_walking->current_left_tz_Nm_ = l_foot_Tz_Nm_;
  
  // getting present values
  for(std::map<std::string, robotis_framework::DynamixelState*>::iterator result_it = result_.begin();
      result_it != result_.end();
      result_it++)
  {
    std::map<std::string, robotis_framework::Dynamixel*>::iterator dxls_it = dxls.find(result_it->first);
    if(dxls_it != dxls.end())
    {
      prev_walking_->curr_angle_rad_[joint_name_to_index_[result_it->first]] = dxls_it->second->dxl_state_->present_position_;

      if (gazebo_)
        prev_walking_->curr_torque_Nm_[joint_name_to_index_[result_it->first]] = dxls_it->second->dxl_state_->present_torque_; // for simulation
    }
  }

  if (gazebo_ == false)
  {
    // prev_walking_->curr_torque_Nm_[0] = (int16_t) dxls["r_sho_pitch"]->dxl_state_->bulk_read_table_["present_current"];
    // prev_walking_->curr_torque_Nm_[0] = (int16_t) dxls["r_sho_roll"]->dxl_state_->bulk_read_table_["present_current"] ;
    // prev_walking_->curr_torque_Nm_[0] = (int16_t) dxls["r_el"]->dxl_state_->bulk_read_table_["present_current"]       ;
    // prev_walking_->curr_torque_Nm_[0] = (int16_t) dxls["l_sho_pitch"]->dxl_state_->bulk_read_table_["present_current"];
    // prev_walking_->curr_torque_Nm_[0] = (int16_t) dxls["l_sho_roll"]->dxl_state_->bulk_read_table_["present_current"] ;
    // prev_walking_->curr_torque_Nm_[0] = (int16_t) dxls["l_el"]->dxl_state_->bulk_read_table_["present_current"]       ;
    prev_walking_->curr_torque_Nm_[0] = (int16_t) dxls["r_hip_yaw"]->dxl_state_->bulk_read_table_["present_current"]  ;
    prev_walking_->curr_torque_Nm_[1] = (int16_t) dxls["r_hip_roll"]->dxl_state_->bulk_read_table_["present_current"] ;
    prev_walking_->curr_torque_Nm_[2] = (int16_t) dxls["r_hip_pitch"]->dxl_state_->bulk_read_table_["present_current"];
    prev_walking_->curr_torque_Nm_[3] = (int16_t) dxls["r_knee"]->dxl_state_->bulk_read_table_["present_current"]     ;
    prev_walking_->curr_torque_Nm_[4] = (int16_t) dxls["r_ank_pitch"]->dxl_state_->bulk_read_table_["present_current"];
    prev_walking_->curr_torque_Nm_[5] = (int16_t) dxls["r_ank_roll"]->dxl_state_->bulk_read_table_["present_current"] ;
    prev_walking_->curr_torque_Nm_[6] = (int16_t) dxls["l_hip_yaw"]->dxl_state_->bulk_read_table_["present_current"]  ;
    prev_walking_->curr_torque_Nm_[7] = (int16_t) dxls["l_hip_roll"]->dxl_state_->bulk_read_table_["present_current"] ;
    prev_walking_->curr_torque_Nm_[8] = (int16_t) dxls["l_hip_pitch"]->dxl_state_->bulk_read_table_["present_current"];
    prev_walking_->curr_torque_Nm_[9] = (int16_t) dxls["l_knee"]->dxl_state_->bulk_read_table_["present_current"]     ;
    prev_walking_->curr_torque_Nm_[10] = (int16_t) dxls["l_ank_pitch"]->dxl_state_->bulk_read_table_["present_current"];
    prev_walking_->curr_torque_Nm_[11] = (int16_t) dxls["l_ank_roll"]->dxl_state_->bulk_read_table_["present_current"] ;
    // prev_walking_->curr_torque_Nm_[0] = (int16_t) dxls["head_pan"]->dxl_state_->bulk_read_table_["present_current"]   ;
    // prev_walking_->curr_torque_Nm_[0] = (int16_t) dxls["head_tilt"]->dxl_state_->bulk_read_table_["present_current"]  ;

    prev_walking_->setCurrentIMUSensorOutput(sensors["gyro_x"], sensors["gyro_y"], sensors["roll"], sensors["pitch"], sensors["yaw"]);
  }
  
  
  // preview control walking
  process_mutex_.lock();
  prev_walking_->process();

  desired_matrix_g_to_pelvis_ = prev_walking_->mat_g_to_pelvis_;
  desired_matrix_g_to_rfoot_  = prev_walking_->mat_g_to_rfoot_;
  desired_matrix_g_to_lfoot_  = prev_walking_->mat_g_to_lfoot_;
  process_mutex_.unlock();

  //publishRobotPose();
  result_["r_hip_yaw"  ]->goal_position_ = prev_walking_->out_angle_rad_[0];
  result_["r_hip_roll" ]->goal_position_ = prev_walking_->out_angle_rad_[1];
  result_["r_hip_pitch"]->goal_position_ = prev_walking_->out_angle_rad_[2];
  result_["r_knee"     ]->goal_position_ = prev_walking_->out_angle_rad_[3];
  result_["r_ank_pitch"]->goal_position_ = prev_walking_->out_angle_rad_[4];
  result_["r_ank_roll" ]->goal_position_ = prev_walking_->out_angle_rad_[5];

  result_["l_hip_yaw"  ]->goal_position_ = prev_walking_->out_angle_rad_[6];
  result_["l_hip_roll" ]->goal_position_ = prev_walking_->out_angle_rad_[7];
  result_["l_hip_pitch"]->goal_position_ = prev_walking_->out_angle_rad_[8];
  result_["l_knee"     ]->goal_position_ = prev_walking_->out_angle_rad_[9];
  result_["l_ank_pitch"]->goal_position_ = prev_walking_->out_angle_rad_[10];
  result_["l_ank_roll" ]->goal_position_ = prev_walking_->out_angle_rad_[11];

//  walking_joint_states_msg_.header.stamp = ros::Time::now();
//  walking_joint_states_msg_.r_goal_hip_y = online_walking->r_leg_out_angle_rad_[0];
//  walking_joint_states_msg_.r_goal_hip_r = online_walking->r_leg_out_angle_rad_[1];
//  walking_joint_states_msg_.r_goal_hip_p = online_walking->r_leg_out_angle_rad_[2];
//  walking_joint_states_msg_.r_goal_kn_p  = online_walking->r_leg_out_angle_rad_[3];
//  walking_joint_states_msg_.r_goal_an_p  = online_walking->r_leg_out_angle_rad_[4];
//  walking_joint_states_msg_.r_goal_an_r  = online_walking->r_leg_out_angle_rad_[5];
//  walking_joint_states_msg_.l_goal_hip_y = online_walking->l_leg_out_angle_rad_[0];
//  walking_joint_states_msg_.l_goal_hip_r = online_walking->l_leg_out_angle_rad_[1];
//  walking_joint_states_msg_.l_goal_hip_p = online_walking->l_leg_out_angle_rad_[2];
//  walking_joint_states_msg_.l_goal_kn_p  = online_walking->l_leg_out_angle_rad_[3];
//  walking_joint_states_msg_.l_goal_an_p  = online_walking->l_leg_out_angle_rad_[4];
//  walking_joint_states_msg_.l_goal_an_r  = online_walking->l_leg_out_angle_rad_[5];
//
//  walking_joint_states_msg_.r_present_hip_y = online_walking->curr_angle_rad_[0];
//  walking_joint_states_msg_.r_present_hip_r = online_walking->curr_angle_rad_[1];
//  walking_joint_states_msg_.r_present_hip_p = online_walking->curr_angle_rad_[2];
//  walking_joint_states_msg_.r_present_kn_p  = online_walking->curr_angle_rad_[3];
//  walking_joint_states_msg_.r_present_an_p  = online_walking->curr_angle_rad_[4];
//  walking_joint_states_msg_.r_present_an_r  = online_walking->curr_angle_rad_[5];
//  walking_joint_states_msg_.l_present_hip_y = online_walking->curr_angle_rad_[6];
//  walking_joint_states_msg_.l_present_hip_r = online_walking->curr_angle_rad_[7];
//  walking_joint_states_msg_.l_present_hip_p = online_walking->curr_angle_rad_[8];
//  walking_joint_states_msg_.l_present_kn_p  = online_walking->curr_angle_rad_[9];
//  walking_joint_states_msg_.l_present_an_p  = online_walking->curr_angle_rad_[10];
//  walking_joint_states_msg_.l_present_an_r  = online_walking->curr_angle_rad_[11];
//  walking_joint_states_pub_.publish(walking_joint_states_msg_);

  std::cout << prev_walking_->walking_pattern_.current_balancing_index_ << " " <<  prev_walking_->walking_pattern_.x_lipm_(0,0) << " " << prev_walking_->walking_pattern_.y_lipm_(0,0) << " " 
  << prev_walking_->walking_pattern_.ep_calculator_.reference_zmp_x_.coeff(0,0) << " " << prev_walking_->walking_pattern_.ep_calculator_.reference_zmp_y_.coeff(0,0) << " " 
  << prev_walking_->walking_pattern_.ep_calculator_.present_body_pose_.x          << " " << prev_walking_->walking_pattern_.ep_calculator_.present_body_pose_.y           << " " << prev_walking_->walking_pattern_.ep_calculator_.present_body_pose_.z << " " 
  << prev_walking_->walking_pattern_.ep_calculator_.present_body_pose_.roll       << " " << prev_walking_->walking_pattern_.ep_calculator_.present_body_pose_.pitch       << " " << prev_walking_->walking_pattern_.ep_calculator_.present_body_pose_.yaw << " " 
  << prev_walking_->walking_pattern_.ep_calculator_.present_right_foot_pose_.x    << " " << prev_walking_->walking_pattern_.ep_calculator_.present_right_foot_pose_.y     << " " << prev_walking_->walking_pattern_.ep_calculator_.present_right_foot_pose_.z << " " 
  << prev_walking_->walking_pattern_.ep_calculator_.present_right_foot_pose_.roll << " " << prev_walking_->walking_pattern_.ep_calculator_.present_right_foot_pose_.pitch << " " << prev_walking_->walking_pattern_.ep_calculator_.present_right_foot_pose_.yaw << " " 
  << prev_walking_->walking_pattern_.ep_calculator_.present_left_foot_pose_.x     << " " << prev_walking_->walking_pattern_.ep_calculator_.present_left_foot_pose_.y      << " " << prev_walking_->walking_pattern_.ep_calculator_.present_left_foot_pose_.z  << " " 
  << prev_walking_->walking_pattern_.ep_calculator_.present_left_foot_pose_.roll  << " " << prev_walking_->walking_pattern_.ep_calculator_.present_left_foot_pose_.pitch  << " " << prev_walking_->walking_pattern_.ep_calculator_.present_left_foot_pose_.yaw  << " " 
  << prev_walking_->r_leg_out_angle_rad_[0] << " " << prev_walking_->r_leg_out_angle_rad_[1] << " " << prev_walking_->r_leg_out_angle_rad_[2] << " " 
  << prev_walking_->r_leg_out_angle_rad_[3] << " " << prev_walking_->r_leg_out_angle_rad_[4] << " " << prev_walking_->r_leg_out_angle_rad_[5] << " " 
  << prev_walking_->l_leg_out_angle_rad_[0] << " " << prev_walking_->l_leg_out_angle_rad_[1] << " " << prev_walking_->l_leg_out_angle_rad_[2] << " " 
  << prev_walking_->l_leg_out_angle_rad_[3] << " " << prev_walking_->l_leg_out_angle_rad_[4] << " " << prev_walking_->l_leg_out_angle_rad_[5] << " " 
  << prev_walking_->curr_angle_rad_[0] << " " << prev_walking_->curr_angle_rad_[1] << " " << prev_walking_->curr_angle_rad_[2] << " " 
  << prev_walking_->curr_angle_rad_[3] << " " << prev_walking_->curr_angle_rad_[4] << " " << prev_walking_->curr_angle_rad_[5] << " " 
  << prev_walking_->curr_angle_rad_[6] << " " << prev_walking_->curr_angle_rad_[7] << " " << prev_walking_->curr_angle_rad_[8] << " " 
  << prev_walking_->curr_angle_rad_[9] << " " << prev_walking_->curr_angle_rad_[10] << " " << prev_walking_->curr_angle_rad_[11] << " " 
  << prev_walking_->curr_torque_Nm_[0] << " " << prev_walking_->curr_torque_Nm_[1] << " " << prev_walking_->curr_torque_Nm_[2] << " " 
  << prev_walking_->curr_torque_Nm_[3] << " " << prev_walking_->curr_torque_Nm_[4] << " " << prev_walking_->curr_torque_Nm_[5] << " " 
  << prev_walking_->curr_torque_Nm_[6] << " " << prev_walking_->curr_torque_Nm_[7] << " " << prev_walking_->curr_torque_Nm_[8] << " " 
  << prev_walking_->curr_torque_Nm_[9] << " " << prev_walking_->curr_torque_Nm_[10] << " " << prev_walking_->curr_torque_Nm_[11] << " " 
  << prev_walking_->current_imu_roll_rad_ << " " << prev_walking_->current_imu_pitch_rad_;

  if (gazebo_)
    std::cout << com_pos_.x << " " << com_pos_.y << " " << com_pos_.z << " "  << std::endl;
  else
    std::cout << " " << std::endl;

  present_running = isRunning();
  if(previous_running_ != present_running)
  {
    if(present_running == true)
    {
      std::string status_msg = WalkingStatusMSG::WALKING_START_MSG;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
      publishDoneMsg("walking_start");
      ROS_INFO("walking_start");
    }
    else
    {
      std::string status_msg = WalkingStatusMSG::WALKING_FINISH_MSG;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
      publishDoneMsg("walking_completed");
      ROS_INFO("walking_completed");
    }
  }
  previous_running_ = present_running;
}

void OP3PreviewWalkingModule::stop()
{
  return;
}

void OP3PreviewWalkingModule::imuDataOutputCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  prev_walking_->setCurrentIMUSensorOutput(msg->angular_velocity.x, msg->angular_velocity.y, msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  imu_data_ = *msg;
}

void OP3PreviewWalkingModule::comDataCallback(const geometry_msgs::Vector3::ConstPtr &msg)
{
  com_pos_ = *msg;
}