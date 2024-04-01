/*
 * message_callback.h
 *
 *  Created on: Mar 28, 2024
 *      Author: Jay Song
 */

#ifndef ADOL_FOOT_STEP_GENERATOR_MESSAGE_CALLBACK_H_
#define ADOL_FOOT_STEP_GENERATOR_MESSAGE_CALLBACK_H_


#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include "adol_foot_step_generator/FootStepCommand.h"
#include "adol_foot_step_generator/Step2DArray.h"

#include "robotis_controller_msgs/StatusMsg.h"

#include "adol_preview_walking_module_msgs/RobotPose.h"
#include "adol_preview_walking_module_msgs/GetReferenceStepData.h"
#include "adol_preview_walking_module_msgs/AddStepDataArray.h"
#include "adol_preview_walking_module_msgs/StartWalking.h"
#include "adol_preview_walking_module_msgs/SetBalanceParam.h"
#include "adol_preview_walking_module_msgs/IsRunning.h"
#include "adol_preview_walking_module_msgs/RemoveExistingStepData.h"


#include "adol_foot_step_generator.h"


void initialize(void);

void walkingModuleStatusMSGCallback(const robotis_controller_msgs::StatusMsg::ConstPtr& msg);

void walkingCommandCallback(const adol_foot_step_generator::FootStepCommand::ConstPtr& msg);
void step2DArrayCallback(const adol_foot_step_generator::Step2DArray::ConstPtr& msg);

void dspCallback(const std_msgs::Float64::ConstPtr& msg);
void footZSwapCallback(const std_msgs::Float64::ConstPtr& msg);
void bodyZSwapCallback(const std_msgs::Float64::ConstPtr& msg);
void yZMPConvergenceCallback(const std_msgs::Float64::ConstPtr& msg);

bool isRunning(void);



#endif /* ADOL_FOOT_STEP_GENERATOR_MESSAGE_CALLBACK_H_ */
