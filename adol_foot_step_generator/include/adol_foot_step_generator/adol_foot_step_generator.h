/*
 * adol_foot_step_generator_node.h
 *
 *  Created on: Mar 28, 2024
 *      Author: Jay Song
 */

#ifndef ADOL_FOOT_STEP_GENERATOR_ADOL_FOOT_STEP_GENERATOR_NODE_H_
#define ADOL_FOOT_STEP_GENERATOR_ADOL_FOOT_STEP_GENERATOR_NODE_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include "adol_preview_walking_module_msgs/AddStepDataArray.h"
#include "adol_foot_step_generator/Step2DArray.h"

#define STOP_WALKING           (0)
#define FORWARD_WALKING        (1)
#define BACKWARD_WALKING       (2)
#define RIGHTWARD_WALKING      (3)
#define LEFTWARD_WALKING       (4)
#define LEFT_ROTATING_WALKING  (5)
#define RIGHT_ROTATING_WALKING (6)

#define MINIMUM_STEP_TIME_SEC  (0.4)

namespace adol
{

class FootStepGenerator
{
public:
  FootStepGenerator();
  ~FootStepGenerator();

  void initialize();

  void getStepData(adol_preview_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
      const adol_preview_walking_module_msgs::StepData& ref_step_data,
      int desired_step_type);

  void getStepDataFromStepData2DArray(adol_preview_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
      const adol_preview_walking_module_msgs::StepData& ref_step_data,
      const adol_foot_step_generator::Step2DArray::ConstPtr& request_step_2d);

  void calcRightKickStep(adol_preview_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
      const adol_preview_walking_module_msgs::StepData& ref_step_data);
  void calcLeftKickStep(adol_preview_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
      const adol_preview_walking_module_msgs::StepData& ref_step_data);

  void calcTurnLeftAndRightKickStep(adol_preview_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
      const adol_preview_walking_module_msgs::StepData& ref_step_data);
  void calcTurnRightAndLeftKickStep(adol_preview_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
      const adol_preview_walking_module_msgs::StepData& ref_step_data);

  int    num_of_step_;
  double fb_step_length_m_;
  double rl_step_length_m_;
  double rotate_step_angle_rad_;
  double step_time_sec_;

  double dsp_ratio_;
  double foot_z_swap_m_;
  double body_z_swap_m_;

  double y_zmp_convergence_m_;

  double start_end_time_sec_;
  double default_y_feet_offset_m_;

  double kick_height_m_;
  double kick_far_m_;
  double kick_pitch_rad_;
  double kick_time_sec_;


private:
  bool calcStep(const adol_preview_walking_module_msgs::StepData& ref_step_data, int previous_step_type,  int desired_step_type);

  void calcFBStep(const adol_preview_walking_module_msgs::StepData& ref_step_data, int direction);
  void calcRLStep(const adol_preview_walking_module_msgs::StepData& ref_step_data, int direction);
  void calcRoStep(const adol_preview_walking_module_msgs::StepData& ref_step_data, int direction);
  void calcStopStep(const adol_preview_walking_module_msgs::StepData& ref_step_data, int direction);

  Eigen::MatrixXd getTransformationXYZRPY(double position_x, double position_y, double position_z, double roll, double pitch, double yaw);
  void getPosefromTransformMatrix(const Eigen::MatrixXd &matTransform, double *position_x, double *position_y, double *position_z, double *roll, double *pitch, double *yaw);
  adol_preview_walking_module_msgs::PoseXYZRPY getPosefromTransformMatrix(const Eigen::MatrixXd &matTransform);
  Eigen::MatrixXd getInverseTransformation(Eigen::MatrixXd transform);

  adol_preview_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type step_data_array_;

  int previous_step_type_;

};

}

#endif /* ADOL_FOOT_STEP_GENERATOR_ADOL_FOOT_STEP_GENERATOR_NODE_H_ */
