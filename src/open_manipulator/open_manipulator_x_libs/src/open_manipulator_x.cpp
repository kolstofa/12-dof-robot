﻿// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na

#include "../include/open_manipulator_x_libs/open_manipulator_x.hpp"
#include "rclcpp/rclcpp.hpp"

OpenManipulatorX::OpenManipulatorX() {}

OpenManipulatorX::~OpenManipulatorX()
{
  delete kinematics_;
  delete actuator_;
  delete tool_;
  for(uint8_t index = 0; index < CUSTOM_TRAJECTORY_SIZE; index++)
    delete custom_trajectory_[index];
}
// left
void OpenManipulatorX::init_open_manipulator_x(bool sim, STRING usb_port, STRING baud_rate, float control_loop_time, std::vector<uint8_t> dxl_id)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "init_open_manipulator_x start "); 
  /*****************************************************************************
    ** Initialize Manipulator Parameter
    *****************************************************************************/
  // May 9 by Hojin
  // Left Arm
     addWorld("world01",   // world name
          "joint01"); // child name

  addJoint("joint01",  // my name
           "world01",   // parent name
           "joint02",  // child name
            math::vector3(0.012, 0.0, 0.017),                // relative position
            math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
            Z_AXIS,    // axis of rotation
            dxl_id[0],        // actuator id
            M_PI * 4,      // max joint limit (3.14 rad)
            -M_PI * 4,     // min joint limit (-3.14 rad)
            1.0,       // coefficient
            9.8406837e-02,                                                        // mass
            math::inertiaMatrix(3.4543422e-05, -1.6031095e-08, -3.8375155e-07,
                                3.2689329e-05, 2.8511935e-08,
                                1.8850320e-05),                                   // inertial tensor
            math::vector3(-3.0184870e-04, 5.4043684e-04, 0.018 + 2.9433464e-02)   // COM
            );

  addJoint("joint02",  // my name
            "joint01",  // parent name
            "joint03",  // child name
            math::vector3(0.0, 0.0, 0.0595),                // relative position
            math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
            Y_AXIS,    // axis of rotation
            dxl_id[1],        // actuator id
            M_PI * 4,    // max joint limit (1.67 rad)
            -M_PI * 4,     // min joint limit (-2.05 rad)
            1.0,       // coefficient
            1.3850917e-01,                                                        // mass
            math::inertiaMatrix(3.3055381e-04, 9.7940978e-08, -3.8505711e-05,
                                3.4290447e-04, -1.5717516e-06,
                                6.0346498e-05),                                   // inertial tensor
            math::vector3(1.0308393e-02, 3.7743363e-04, 1.0170197e-01)            // COM
            );

  addJoint("joint03",  // my name
            "joint02",  // parent name
            "joint04",  // child name
            math::vector3(0.044, 0.0, 0.0),               // relative position
            math::convertRPYToRotationMatrix(1.57, 0.0, 0.0), // relative orientation
            X_AXIS,    // axis of rotation
            dxl_id[2],        // actuator id
            M_PI * 4,      // max joint limit (1.53 rad)
            -M_PI * 4,   // min joint limit (-1.67 rad)
            1.0,       // coefficient
            1.3274562e-01,                                                        // mass
            math::inertiaMatrix(3.0654178e-05, -1.2764155e-06, -2.6874417e-07,
                                2.4230292e-04, 1.1559550e-08,
                                2.5155057e-04),                                   // inertial tensor
            math::vector3(9.0909590e-02, 3.8929816e-04, 2.2413279e-04)            // COM
            );
  addJoint("joint04",  // my name
            "joint03",  // parent name
            "joint05",  // child name
            math::vector3(0.129, 0.0, 0.022),               // relative position
            math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
            Y_AXIS,    // axis of rotation
            dxl_id[3],        // actuator id
            M_PI * 4,      // max joint limit (1.53 rad)
            -M_PI * 4,   // min joint limit (-1.67 rad)
            1.0,       // coefficient
            1.3274562e-01,                                                        // mass
            math::inertiaMatrix(3.0654178e-05, -1.2764155e-06, -2.6874417e-07,
                                2.4230292e-04, 1.1559550e-08,
                                2.5155057e-04),                                   // inertial tensor
            math::vector3(9.0909590e-02, 3.8929816e-04, 2.2413279e-04)            // COM
            );
  addJoint("joint05",  // my name
            "joint04",  // parent name
            "joint06",  // child name
            math::vector3(0.122, 0.0, 0.0),               // relative position
            math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
            Y_AXIS,    // axis of rotation
            dxl_id[4],        // actuator id
            M_PI * 4,      // max joint limit (1.53 rad)
            -M_PI * 4,   // min joint limit (-1.67 rad)
            1.0,       // coefficient
            1.3274562e-01,                                                        // mass
            math::inertiaMatrix(3.0654178e-05, -1.2764155e-06, -2.6874417e-07,
                                2.4230292e-04, 1.1559550e-08,
                                2.5155057e-04),                                   // inertial tensor
            math::vector3(9.0909590e-02, 3.8929816e-04, 2.2413279e-04)            // COM
            );
  addJoint("joint06",  // my name
            "joint05",  // parent name
            "gripper01", // child name
            math::vector3(0.043, 0.0, 0.0),                 // relative position
            math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
            X_AXIS,    // axis of rotation
            dxl_id[5],        // actuator id
            M_PI * 4,       // max joint limit (2.0 rad)
            -M_PI * 4,      // min joint limit (-1.8 rad)
            1.0,       // coefficient
            1.4327573e-01,                                                        // mass
            math::inertiaMatrix(8.0870749e-05, 0.0, -1.0157896e-06,
                                7.5980465e-05, 0.0,
                                9.3127351e-05),                                   // inertial tensor
            math::vector3(4.4206755e-02, 3.6839985e-07, 8.9142216e-03)            // COM
            );
  addTool("gripper01",  // my name
          "joint06",   // parent name
          math::vector3(0.126, 0.0, 0.0),                 // relative position
          math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
          dxl_id[6],         // actuator id
          0.010,      // max gripper limit (0.01 m)
          -0.010,     // min gripper limit (-0.01 m)
          -0.015,     // Change unit from `meter` to `radian`
          3.2218127e-02 * 2,                                                    // mass
          math::inertiaMatrix(9.5568826e-06, 2.8424644e-06, -3.2829197e-10,
                              2.2552871e-05, -3.1463634e-10,
                              1.7605306e-05),                                   // inertial tensor
          math::vector3(0.028 + 8.3720668e-03, 0.0246, -4.2836895e-07)          // COM
          );        
  /*****************************************************************************
  ** Initialize Kinematics 
  *****************************************************************************/
  kinematics_ = new kinematics::SolverCustomizedforOMChain();
//  kinematics_ = new kinematics::SolverUsingCRAndSRPositionOnlyJacobian();
  addKinematics(kinematics_);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "kinematics_ complete"); 
  if(!sim)
  {
    /*****************************************************************************
    ** Initialize Joint Actuator
    *****************************************************************************/
    // actuator_ = new dynamixel::JointDynamixel();
    actuator_ = new dynamixel::JointDynamixelProfileControl(control_loop_time);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "initialize joint actuator : 1"); 
    // Set communication arguments
    STRING dxl_comm_arg[2] = {usb_port, baud_rate};
    void *p_dxl_comm_arg = &dxl_comm_arg;

    // Set joint actuator id
    std::vector<uint8_t> jointDxlId;
    jointDxlId.push_back(dxl_id[0]);
    jointDxlId.push_back(dxl_id[1]);
    jointDxlId.push_back(dxl_id[2]);
    jointDxlId.push_back(dxl_id[3]);
    jointDxlId.push_back(dxl_id[4]);
    jointDxlId.push_back(dxl_id[5]);
    addJointActuator(JOINT_DYNAMIXEL, actuator_, jointDxlId, p_dxl_comm_arg);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "initialize joint actuator : 2"); 
    // Set joint actuator control mode
    STRING joint_dxl_mode_arg = "position_mode";
    void *p_joint_dxl_mode_arg = &joint_dxl_mode_arg;
    setJointActuatorMode(JOINT_DYNAMIXEL, jointDxlId, p_joint_dxl_mode_arg);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "initialize joint actuator : 3"); 
    /*****************************************************************************
    ** Initialize Tool Actuator
    *****************************************************************************/
    tool_ = new dynamixel::GripperDynamixel();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "initialize tool actuator : 1"); 
    uint8_t gripperDxlId = dxl_id[6];
    addToolActuator(TOOL_DYNAMIXEL, tool_, gripperDxlId, p_dxl_comm_arg);

    // Set gripper actuator control mode
    STRING gripper_dxl_mode_arg = "current_based_position_mode";
    void *p_gripper_dxl_mode_arg = &gripper_dxl_mode_arg;
    setToolActuatorMode(TOOL_DYNAMIXEL, p_gripper_dxl_mode_arg);

    // Set gripper actuator parameter
    STRING gripper_dxl_opt_arg[2];
    void *p_gripper_dxl_opt_arg = &gripper_dxl_opt_arg;
    gripper_dxl_opt_arg[0] = "Profile_Acceleration";
    gripper_dxl_opt_arg[1] = "20";
    setToolActuatorMode(TOOL_DYNAMIXEL, p_gripper_dxl_opt_arg);

    gripper_dxl_opt_arg[0] = "Profile_Velocity";
    gripper_dxl_opt_arg[1] = "200";
    setToolActuatorMode(TOOL_DYNAMIXEL, p_gripper_dxl_opt_arg);

    // Enable All Actuators 
    enableAllActuator();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "initialize tool actuator : 2"); 
    // Receive current angles from all actuators 
    receiveAllJointActuatorValue();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "receiveAllJointActuatorValue : complete"); 
    receiveAllToolActuatorValue();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "receiveAllToolActuatorValue : complete"); 
    
  }

  /*****************************************************************************
  ** Initialize Custom Trajectory
  *****************************************************************************/
  custom_trajectory_[0] = new custom_trajectory::Line();
  custom_trajectory_[1] = new custom_trajectory::Circle();
  custom_trajectory_[2] = new custom_trajectory::Rhombus();
  custom_trajectory_[3] = new custom_trajectory::Heart();

  addCustomTrajectory(CUSTOM_TRAJECTORY_LINE, custom_trajectory_[0]);
  addCustomTrajectory(CUSTOM_TRAJECTORY_CIRCLE, custom_trajectory_[1]);
  addCustomTrajectory(CUSTOM_TRAJECTORY_RHOMBUS, custom_trajectory_[2]);
  addCustomTrajectory(CUSTOM_TRAJECTORY_HEART, custom_trajectory_[3]);
}

void OpenManipulatorX::process_open_manipulator_x(double present_time)
{
  // Planning (ik)
  JointWaypoint goal_joint_value = getJointGoalValueFromTrajectory(present_time);
  JointWaypoint goal_tool_value  = getToolGoalValue();

  // Control (motor)
  receiveAllJointActuatorValue();
  receiveAllToolActuatorValue();
  if(goal_joint_value.size() != 0) sendAllJointActuatorValue(goal_joint_value);
  if(goal_tool_value.size() != 0) sendAllToolActuatorValue(goal_tool_value);

  // Perception (fk)
  solveForwardKinematics();
}
