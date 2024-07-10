/*******************************************************************************
* Copyright 2019 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

//#include "open_manipulator_x_master_slave/open_manipulator_x_master_slave.hpp"
#include "../include/open_manipulator_x_master_slave/open_manipulator_x_master_slave.hpp"

using namespace std::chrono_literals;
int loop_counter=0;

OpenManipulatorXMasterSlave::OpenManipulatorXMasterSlave(std::string usb_port, std::string baud_rate)
: Node("open_manipulator_x_master_slave"){
  /************************************************************
  ** Initialise ROS parameters
  ************************************************************/
  init_parameters();
  printf("init parameters setup is done.\n");
  /************************************************************
  ** Initialise variables
  ************************************************************/
  goal_joint_position_.resize(NUM_OF_JOINT);
  goal_tool_position_ = 0.0;

  // dxl_id_={1,2,3,4,5,6,7};
  open_manipulator_x_.init_open_manipulator_x(false, usb_port, baud_rate, service_call_period_, dxl_id_);
  open_manipulator_x_.disableAllActuator();

  mode_state_ = MASTER_SLAVE_MODE;
  buffer_index_ = 0;
  
  this->disable_waiting_for_enter();

  /************************************************************
  ** Initialise ROS clients
  ************************************************************/
  goal_joint_space_path_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>(
    "open_manipulator_x/goal_joint_space_path");
  goal_tool_control_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>(
    "open_manipulator_x/goal_tool_control");

  /************************************************************
  ** Initialise ROS timers
  ************************************************************/
  update_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&OpenManipulatorXMasterSlave::update_callback, this));  

  RCLCPP_INFO(this->get_logger(), "OpenManipulator-X master slave node has been initialised.");
}

OpenManipulatorXMasterSlave::~OpenManipulatorXMasterSlave()
{
  this->restore_terminal_settings();
  RCLCPP_INFO(this->get_logger(), "OpenManipulator-X master slave node has been terminated.");
}

/********************************************************************************
** Init Functions
********************************************************************************/
void OpenManipulatorXMasterSlave::init_parameters()
{
  // Declare parameters that may be set on this node
  this->declare_parameter("service_call_period", rclcpp::PARAMETER_DOUBLE);

  this->declare_parameter("joint01_id", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("joint02_id", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("joint03_id", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("joint04_id", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("joint05_id", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("joint06_id", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("gripper01_id", rclcpp::PARAMETER_INTEGER);

  // Get parameter from yaml
  int dxl_id_1, dxl_id_2, dxl_id_3, dxl_id_4, dxl_id_5, dxl_id_6, dxl_id_7;
  this->get_parameter_or<double>("service_call_period", 0.001f);

  this->get_parameter_or<int>("joint01_id", dxl_id_1, 1);
  this->get_parameter_or<int>("joint02_id", dxl_id_2, 2);
  this->get_parameter_or<int>("joint03_id", dxl_id_3, 3);
  this->get_parameter_or<int>("joint04_id", dxl_id_4, 4);
  this->get_parameter_or<int>("joint05_id", dxl_id_5, 5);
  this->get_parameter_or<int>("joint06_id", dxl_id_6, 6);
  this->get_parameter_or<int>("gripper01_id", dxl_id_7, 7);
  dxl_id_.push_back(dxl_id_1);
  dxl_id_.push_back(dxl_id_2);
  dxl_id_.push_back(dxl_id_3);
  dxl_id_.push_back(dxl_id_4);
  dxl_id_.push_back(dxl_id_5);
  dxl_id_.push_back(dxl_id_6);
  dxl_id_.push_back(dxl_id_7);
}

/*****************************************************************************
** Callback functions for ROS clients
*****************************************************************************/
void OpenManipulatorXMasterSlave::sync_open_manipulator_x(bool recorded_state){
  RCLCPP_INFO(this->get_logger(), "Synchronizing OpenManipulator-X");
  double sync_path_time = 1.0;

  open_manipulator_x_.receiveAllJointActuatorValue();
  open_manipulator_x_.receiveAllToolActuatorValue();

  if (recorded_state) // move to first pose of recorded buffer
  {
    set_joint_space_path(sync_path_time, record_buffer_.at(buffer_index_).joint_angle);
    set_tool_control(record_buffer_.at(buffer_index_).tool_position);
  }
  else // move to present master pose
  {
    set_joint_space_path(sync_path_time);
    set_tool_control();
  }
  return;
}

void OpenManipulatorXMasterSlave::set_goal()
{
  if(mode_state_ == MASTER_SLAVE_MODE){
    set_joint_space_path(service_call_period_);
    set_tool_control();
  }
  else if(mode_state_ == START_RECORDING_TRAJECTORY_MODE){
    set_joint_space_path(service_call_period_);
    set_tool_control();

    WaypointBuffer temp;
    temp.joint_angle = goal_joint_position_;
    temp.tool_position = goal_tool_position_;
    record_buffer_.push_back(temp);
  }
  else if(mode_state_ == STOP_RECORDING_TRAJECTORY_MODE){}
  else if(mode_state_ == PLAY_RECORDED_TRAJECTORY_MODE){
    if(record_buffer_.size() > (unsigned)buffer_index_){
      set_joint_space_path(service_call_period_, record_buffer_.at(buffer_index_).joint_angle);
      set_tool_control(record_buffer_.at(buffer_index_).tool_position);
      buffer_index_ ++;
    }
  }
}

bool OpenManipulatorXMasterSlave::set_joint_space_path(double path_time, std::vector<double> joint_angle){
  auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();

  std::vector<std::string> joint_name = open_manipulator_x_.getManipulator()->getAllActiveJointComponentName();
  request->joint_position.joint_name = joint_name;

  // // Debug ioint name
  // std::cout << "Active Joint Names:" << std::endl;
  // for (const auto& name : joint_name) {
  //     std::cout << name << std::endl;
  // }

  std::vector<double> joint_value;
  if (joint_angle.size()) joint_value = joint_angle;
  else{
    joint_value.push_back(open_manipulator_x_.getJointValue(joint_name.at(0)).position);
    joint_value.push_back(open_manipulator_x_.getJointValue(joint_name.at(1)).position);
    joint_value.push_back(open_manipulator_x_.getJointValue(joint_name.at(2)).position);
    joint_value.push_back(open_manipulator_x_.getJointValue(joint_name.at(3)).position);
    joint_value.push_back(open_manipulator_x_.getJointValue(joint_name.at(4)).position);
    joint_value.push_back(open_manipulator_x_.getJointValue(joint_name.at(5)).position);
  }
  //open_manipulator_x_.printManipulatorSetting();
  open_manipulator_x_.receiveAllJointActuatorValue();
  // // Debug joint value
  // for (size_t i = 0; i < joint_value.size(); ++i){
  //   printf("joint_value[%zu] = %.3f\n", i, joint_value[i]);
  //   //printf("joint_name[%zu] = %s\n", i, open_manipulator_x_);
  //   }
  
  for (int i = 0; i < NUM_OF_JOINT; i ++){
    if(open_manipulator_x_.getManipulator()->checkJointLimit(joint_name.at(i), joint_value.at(i)))
      request->joint_position.position.push_back(joint_value.at(i));
    else
      request->joint_position.position.push_back(goal_joint_position_.at(i));
  }

  goal_joint_position_ = request->joint_position.position;
  request->path_time = path_time;

  //   // Debug: Print the joint positions being set
  // std::cout << "setting joint positions: ";
  // for (const auto& pos : goal_joint_position_) {
  //     std::cout << pos << " ";
  // }
  // std::cout << std::endl;

  using ServiceResponseFuture = rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future){
    auto result = future.get();
    return result->is_planned;
  };

  auto future_result = goal_joint_space_path_client_->async_send_request(request, response_received_callback);
  return false;
}

bool OpenManipulatorXMasterSlave::set_tool_control(double set_goal_tool_position){
  double tool_value;
  
  // printf("set_goal_tool_position: %lf\n",set_goal_tool_position);
  if(set_goal_tool_position < -0.1)
    tool_value = open_manipulator_x_.getAllToolValue().at(0).position;
  else
    tool_value = set_goal_tool_position;
  
  // printf("tool_value: %lf\n",tool_value);

  auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
  request->joint_position.joint_name.push_back("gripper01");
  
  if (open_manipulator_x_.getManipulator()->checkJointLimit("gripper01", tool_value))
    request->joint_position.position.push_back(tool_value);
  else
    request->joint_position.position.push_back(goal_tool_position_);

  goal_tool_position_ = request->joint_position.position.at(0);
  // Debug: Print the tool position being set
  // std::cout << "setting tool position: " << goal_tool_position_ << std::endl;
  
  using ServiceResponseFuture = rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) 
  {
    auto result = future.get();
    return result->is_planned;
  };
  auto future_result = goal_tool_control_client_->async_send_request(request, response_received_callback);
  return false;
}

void OpenManipulatorXMasterSlave::update_callback()
{
  // take current joint, tool value
  open_manipulator_x_.receiveAllJointActuatorValue();
  open_manipulator_x_.receiveAllToolActuatorValue();

  this->print_text();

  // //print dxl_id_
  // std::cout << "Contents of dxl_id_:" << std::endl;
  // for (size_t i = 0; i < dxl_id_.size(); ++i){
  //   std::cout << "Element " << i << ": " << static_cast<int>(dxl_id_[i]) << std::endl;
  // }
  printf("\n=====loop counter: %d=====\n",loop_counter); loop_counter++;
  
  char ch = std::getchar();
  if (ch=='1' || ch=='2' || ch=='3' || ch=='4') this->set_mode_state(ch);
  this->set_goal();
}

void OpenManipulatorXMasterSlave::set_mode_state(char ch)
{
  if (ch == '1')
  {
    sync_open_manipulator_x(false);
    mode_state_ = MASTER_SLAVE_MODE;
  }
  else if (ch == '2')
  {
    sync_open_manipulator_x(false);
    record_buffer_.clear();
    mode_state_ = START_RECORDING_TRAJECTORY_MODE;
  }
  else if (ch == '3')
  {
    mode_state_ = STOP_RECORDING_TRAJECTORY_MODE;
  }
  else if (ch == '4')
  {
    buffer_index_ = 0;
    if (record_buffer_.size()) sync_open_manipulator_x(true);
    mode_state_ = PLAY_RECORDED_TRAJECTORY_MODE;
  }
}

/********************************************************************************
** Other Functions
********************************************************************************/
void OpenManipulatorXMasterSlave::print_text()
{
  system("clear");
  printf("\n");
  printf("-----------------------------\n");
  printf("Control Your OpenManipulator!\n");
  printf("-----------------------------\n");
  printf("Present Control Mode\n");

  if (mode_state_ == MASTER_SLAVE_MODE)
  {
    printf("Master - Slave Mode\n");
  }
  else if (mode_state_ == START_RECORDING_TRAJECTORY_MODE)
  {
    printf("Start Recording Trajectory\n");
    printf("Buffer Size : %d\n", (int)(record_buffer_.size()));
  }
  else if (mode_state_ == STOP_RECORDING_TRAJECTORY_MODE)
  {
    printf("Stop Recording Trajectory\n");
    printf("Buffer Size : %d\n", (int)(record_buffer_.size()));
  }
  else if (mode_state_ == PLAY_RECORDED_TRAJECTORY_MODE)
  {
    printf("Play Recorded Trajectory Mode\n");
    printf("Buffer Size : %d\n", (int)(record_buffer_.size()));
    printf("Buffer index : %d\n", buffer_index_);
  }

  printf("-----------------------------\n");
  printf("1 : Master - Slave Mode\n");
  printf("2 : Start Recording Trajectory\n");
  printf("3 : Stop Recording Trajectory\n");
  printf("4 : Play Recorded Trajectory\n");
  printf("-----------------------------\n");
  printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf J5: %.3lf J6: %.3lf\n",
    goal_joint_position_.at(0),
    goal_joint_position_.at(1),
    goal_joint_position_.at(2),
    goal_joint_position_.at(3),
    goal_joint_position_.at(4),
    goal_joint_position_.at(5));
  printf("Present Tool Position: %.3lf\n", goal_tool_position_);
  printf("-----------------------------\n");
}

void OpenManipulatorXMasterSlave::restore_terminal_settings()
{
  tcsetattr(0, TCSANOW, &oldt_);  /* Apply saved settings */
}

void OpenManipulatorXMasterSlave::disable_waiting_for_enter()
{
  struct termios newt;

  tcgetattr(0, &oldt_);             /* Save terminal settings */
  newt = oldt_;                     /* Init new settings */
  newt.c_lflag &= ~(ICANON | ECHO); /* Change settings */
  newt.c_cc[VMIN] = 0;
  newt.c_cc[VTIME] = 0;
  tcsetattr(0, TCSANOW, &newt);     /* Apply settings */
}

/*****************************************************************************
** Main
*****************************************************************************/
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::string usb_port = "/dev/ttyUSB1";
  std::string baud_rate = "57600";

  if (argc == 3)
  {
    usb_port = argv[1];
    baud_rate = argv[2];
    printf("port_name and baud_rate are set to %s, %s \n", usb_port.c_str(), baud_rate.c_str());
  }
  else
    printf("port_name and baud_rate are set to %s, %s \n", usb_port.c_str(), baud_rate.c_str());

  rclcpp::spin(std::make_shared<OpenManipulatorXMasterSlave>(usb_port, baud_rate));
  rclcpp::shutdown();

  return 0;
}
