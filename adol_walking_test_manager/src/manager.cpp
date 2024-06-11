
#include <ros/ros.h>
#include <std_msgs/String.h>

/* ROBOTIS Controller Header */
#include "robotis_controller/robotis_controller.h"

/* Sensor Module Header */
#include "adol_open_cr_module/adol_open_cr_module.h"

/* Motion Module Header */
#include "op3_base_module/base_module.h"
#include "adol_preview_walking_module/op3_preview_walking_module.h"

using namespace robotis_framework;
using namespace dynamixel;
using namespace robotis_op;

bool simulation_flag = false;

int g_baudrate;
std::string g_offset_file;
std::string g_robot_file;
std::string g_init_file;
std::string g_device_name;

ros::Publisher g_init_pose_pub;

const int BAUD_RATE = 2000000;
const double PROTOCOL_VERSION = 2.0;
const int SUB_CONTROLLER_ID = 200;
const int DXL_BROADCAST_ID = 254;
const int DEFAULT_DXL_ID = 1;
const std::string SUB_CONTROLLER_DEVICE = "/dev/ttyUSB0";
const int POWER_CTRL_TABLE = 24;
const int RGB_LED_CTRL_TABLE = 26;
const int TORQUE_ON_CTRL_TABLE = 64;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_manager");
  ros::NodeHandle nh;

  ROS_INFO("test_manager->init");
  RobotisController *controller = RobotisController::getInstance();

  nh.param<std::string>("offset_file_path", g_offset_file, "");
  nh.param<std::string>("robot_file_path", g_robot_file, "");
  nh.param<std::string>("init_file_path", g_init_file, "");
  nh.param<std::string>("device_name", g_device_name, SUB_CONTROLLER_DEVICE);
  nh.param<bool>("gazebo", simulation_flag, false);
  controller->gazebo_mode_ = simulation_flag;

  g_init_pose_pub = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);

  /* real robot */ // for turning on the motors and their torque
  if (simulation_flag == false)
  {
    // open port
    PortHandler *port_handler = (PortHandler *) PortHandler::getPortHandler(g_device_name.c_str());
    bool set_port_result = port_handler->setBaudRate(BAUD_RATE);
    if (set_port_result == false)
      ROS_ERROR("Error Set port");

    PacketHandler *packet_handler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // power on dxls
    int torque_on_count = 0;

    while (torque_on_count < 5)
    {
      int _return = packet_handler->write1ByteTxRx(port_handler, SUB_CONTROLLER_ID, POWER_CTRL_TABLE, 1);

      if(_return != 0)
        ROS_ERROR("Torque on DXLs! [%s]", packet_handler->getRxPacketError(_return));
      else
        ROS_INFO("Torque on DXLs!");

      if (_return == 0)
        break;
      else
        torque_on_count++;
    }

    usleep(100 * 1000);

    // set RGB-LED to GREEN
    int led_full_unit = 0x1F;
    int led_range = 5;
    int led_value = led_full_unit << led_range;
    int _return = packet_handler->write2ByteTxRx(port_handler, SUB_CONTROLLER_ID, RGB_LED_CTRL_TABLE, led_value);

    if(_return != 0)
      ROS_ERROR("Fail to control LED [%s]", packet_handler->getRxPacketError(_return));

    port_handler->closePort();
  }
  /* gazebo simulation */
  else
  {
    ROS_WARN("SET TO SIMULATION MODE!");
    std::string robot_name;
    nh.param<std::string>("gazebo_robot_name", robot_name, "");
    if (robot_name != "")
      controller->gazebo_robot_name_ = robot_name;
  }

  if (g_robot_file == "")
  {
    ROS_ERROR("NO robot file path in the ROS parameters.");
    return -1;
  }

    // initialize robot
  if (controller->initialize(g_robot_file, g_init_file) == false)
  {
    ROS_ERROR("ROBOTIS Controller Initialize Fail!");
    return -1;
  }

  // load offset
  if (g_offset_file != "")
    controller->loadOffset(g_offset_file);

  usleep(300 * 1000);


  // set initial pose
  robotis_framework::Pose3D r_foot, l_foot, pelvis;
  r_foot.x = 0.0;    r_foot.y = -0.035;  r_foot.z = 0.0;
  r_foot.roll = 0.0; r_foot.pitch = 0.0; r_foot.yaw = 0.0;

  l_foot.x = 0.0;    l_foot.y = 0.035;   l_foot.z = 0.0;
  l_foot.roll = 0.0; l_foot.pitch = 0.0; l_foot.yaw = 0.0;

  pelvis.x = 0.0;    pelvis.y = 0.0;     pelvis.z = 0.215;
  pelvis.roll = 0.0; pelvis.pitch = 0.0; pelvis.yaw = 0;

  adol::OP3PreviewWalkingModule::getInstance()->setInitialPose(r_foot, l_foot, pelvis);
  adol::OP3PreviewWalkingModule::getInstance()->setLIPMHeight(0.23);


  if (simulation_flag == false)
  {
    /* Add Sensor Module */
    controller->addSensorModule((SensorModule *) adol::OpenCRModule::getInstance());
  }

  controller->addMotionModule((MotionModule*) BaseModule::getInstance());
  controller->addMotionModule((MotionModule*) adol::OP3PreviewWalkingModule::getInstance());

  // start timer
  controller->startTimer();

  usleep(100 * 1000);

  // go to init pose
  std_msgs::String init_msg;
  init_msg.data = "ini_pose";

  g_init_pose_pub.publish(init_msg);
  ROS_INFO("Go to init pose");

  while (ros::ok())
  {
    usleep(1 * 1000);

    ros::spin();
  }

  return 0;
}
