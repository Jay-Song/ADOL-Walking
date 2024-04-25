/*
 * preview_walking_module.h
 *
 *  Created on: 2024. 04. 17.
 *      Author: Jay-Song
 */

#ifndef ADOL_PREVIEW_WALKING_MODULE_OP3_PREVIEW_WALKING_MODULE_H_
#define ADOL_PREVIEW_WALKING_MODULE_OP3_PREVIEW_WALKING_MODULE_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/thread.hpp>

#include "adol_preview_walking_module/adol_preview_walking.h"
#include "robotis_framework_common/motion_module.h"

#include "robotis_controller_msgs/StatusMsg.h"

#include "adol_preview_walking_module_msgs/RobotPose.h"
#include "adol_preview_walking_module_msgs/GetReferenceStepData.h"
#include "adol_preview_walking_module_msgs/AddStepDataArray.h"
#include "adol_preview_walking_module_msgs/StartWalking.h"
#include "adol_preview_walking_module_msgs/IsRunning.h"
#include "adol_preview_walking_module_msgs/RemoveExistingStepData.h"

#include "adol_preview_walking_module_msgs/SetBalanceParam.h"
#include "adol_preview_walking_module_msgs/SetJointFeedBackGain.h"

#include "adol_preview_walking_module_msgs/WalkingJointStatesStamped.h"

//#include "diana_msgs/ForceTorque.h"

namespace adol
{
class OP3PreviewWalkingModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<OP3PreviewWalkingModule>
{
public:
  OP3PreviewWalkingModule();
  virtual ~OP3PreviewWalkingModule();

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void onModuleEnable();
  void onModuleDisable();

  void stop();
  bool isRunning();

  void setLIPMHeight(double height_m);
  void setInitialPose(robotis_framework::Pose3D r_foot, robotis_framework::Pose3D l_foot, robotis_framework::Pose3D pelvis);

private:
  void publishRobotPose(void);
  void publishStatusMsg(unsigned int type, std::string msg);
  void publishDoneMsg(std::string msg);
  void publishWalkingTuningData();

  /* ROS Topic Callback Functions */
  //void imuDataOutputCallback(const sensor_msgs::Imu::ConstPtr &msg);
  //void ftDataOutputCallback(const diana_msgs::ForceTorque::ConstPtr &msg);

  /* ROS Service Callback Functions */
  bool setBalanceParamServiceCallback(adol_preview_walking_module_msgs::SetBalanceParam::Request  &req,
      adol_preview_walking_module_msgs::SetBalanceParam::Response &res);

  bool setJointFeedBackGainServiceCallback(adol_preview_walking_module_msgs::SetJointFeedBackGain::Request  &req,
      adol_preview_walking_module_msgs::SetJointFeedBackGain::Response &res);

  bool getReferenceStepDataServiceCallback(adol_preview_walking_module_msgs::GetReferenceStepData::Request  &req,
      adol_preview_walking_module_msgs::GetReferenceStepData::Response &res);
  bool addStepDataServiceCallback(adol_preview_walking_module_msgs::AddStepDataArray::Request  &req,
      adol_preview_walking_module_msgs::AddStepDataArray::Response &res);
  bool startWalkingServiceCallback(adol_preview_walking_module_msgs::StartWalking::Request  &req,
      adol_preview_walking_module_msgs::StartWalking::Response &res);
  bool IsRunningServiceCallback(adol_preview_walking_module_msgs::IsRunning::Request  &req,
      adol_preview_walking_module_msgs::IsRunning::Response &res);
  bool removeExistingStepDataServiceCallback(adol_preview_walking_module_msgs::RemoveExistingStepData::Request  &req,
      adol_preview_walking_module_msgs::RemoveExistingStepData::Response &res);

  int convertStepDataMsgToStepData(adol_preview_walking_module_msgs::StepData& src, robotis_framework::StepData& des);
  int convertStepDataToStepDataMsg(robotis_framework::StepData& src, adol_preview_walking_module_msgs::StepData& des);

  void setBalanceParam(adol_preview_walking_module_msgs::BalanceParam& balance_param_msg);

  void updateBalanceParam();

  bool checkBalanceOnOff();

  void queueThread();

  void setJointFeedBackGain(adol_preview_walking_module_msgs::JointFeedBackGain& msg);
  void updateJointFeedBackGain();

  std::map<std::string, int> joint_name_to_index_;

  bool            gazebo_;
  int             control_cycle_msec_;
  boost::thread   queue_thread_;
  boost::mutex    process_mutex_;

  Eigen::MatrixXd desired_matrix_g_to_pelvis_;
  Eigen::MatrixXd desired_matrix_g_to_rfoot_;
  Eigen::MatrixXd desired_matrix_g_to_lfoot_;

  bool previous_running_, present_running;

  ros::Publisher robot_pose_pub_;
  ros::Publisher status_msg_pub_;
  ros::Publisher pelvis_base_msg_pub_;
  ros::Publisher done_msg_pub_;

  ros::Publisher walking_joint_states_pub_;
  ros::Publisher imu_orientation_states_pub_;
  ros::Publisher ft_states_pub_;
  adol_preview_walking_module_msgs::WalkingJointStatesStamped walking_joint_states_msg_;

  adol_preview_walking_module_msgs::RobotPose  robot_pose_msg_;
  bool balance_update_with_loop_;
  double balance_update_duration_;
  double balance_update_sys_time_;
  robotis_framework::FifthOrderPolynomialTrajectory balance_update_tra_;


  bool joint_feedback_update_with_loop_;
  double joint_feedback_update_duration_;
  double joint_feedback_update_sys_time_;
  robotis_framework::FifthOrderPolynomialTrajectory joint_feedback_update_tra_;

  adol_preview_walking_module_msgs::JointFeedBackGain previous_joint_feedback_gain_;
  adol_preview_walking_module_msgs::JointFeedBackGain current_joint_feedback_gain_;
  adol_preview_walking_module_msgs::JointFeedBackGain desired_joint_feedback_gain_;

  adol_preview_walking_module_msgs::BalanceParam previous_balance_param_;
  adol_preview_walking_module_msgs::BalanceParam current_balance_param_;
  adol_preview_walking_module_msgs::BalanceParam desired_balance_param_;

  double lipm_height_m_;
  PreviewWalking* prev_walking_;
};

}

#endif /* ADOL_PREVIEW_WALKING_MODULE_OP3_PREVIEW_WALKING_MODULE_H_ */