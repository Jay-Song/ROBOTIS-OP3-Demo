/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
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

/* Author: Kayman Jung */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "robotis_controller_msgs/srv/set_module.hpp"
#include "robotis_controller_msgs/msg/sync_write_item.hpp"

void buttonHandlerCallback(const std_msgs::msg::String::SharedPtr msg);
void jointstatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
void readyToDemo();
void setModule(const std::string& module_name);
void goInitPose();
void setLED(int led);
bool checkManagerRunning(rclcpp::Node::SharedPtr node, std::string& manager_name);
void torqueOnAll();
void torqueOff(const std::string& body_side);

enum ControlModule
{
  None = 0,
  DirectControlModule = 1,
  Framework = 2,
};

const int SPIN_RATE = 30;
const bool DEBUG_PRINT = false;

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr init_pose_pub;
rclcpp::Publisher<robotis_controller_msgs::msg::SyncWriteItem>::SharedPtr sync_write_pub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dxl_torque_pub;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr write_joint_pub;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr write_joint_pub2;
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr button_sub;
rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr read_joint_sub;

rclcpp::Client<robotis_controller_msgs::srv::SetModule>::SharedPtr set_joint_module_client;

int control_module = None;
bool demo_ready = false;

//node main
int main(int argc, char **argv)
{
  //init ros
  rclcpp::init(argc, argv);

  auto nh = rclcpp::Node::make_shared("read_write_demo");

  init_pose_pub = nh->create_publisher<std_msgs::msg::String>("/robotis/base/ini_pose", 5);
  sync_write_pub = nh->create_publisher<robotis_controller_msgs::msg::SyncWriteItem>("/robotis/sync_write_item", 10);
  dxl_torque_pub = nh->create_publisher<std_msgs::msg::String>("/robotis/dxl_torque", 10);
  write_joint_pub = nh->create_publisher<sensor_msgs::msg::JointState>("/robotis/set_joint_states", 10);
  write_joint_pub2 = nh->create_publisher<sensor_msgs::msg::JointState>("/robotis/direct_control/set_joint_states", 10);

  read_joint_sub = nh->create_subscription<sensor_msgs::msg::JointState>("/robotis/present_joint_states", 10, jointstatesCallback);
  button_sub = nh->create_subscription<std_msgs::msg::String>("/robotis/open_cr/button", 10, buttonHandlerCallback);

  // service
  set_joint_module_client = nh->create_client<robotis_controller_msgs::srv::SetModule>("/robotis/set_present_ctrl_modules");

  rclcpp::Rate loop_rate(SPIN_RATE);

  // wait for starting of op3_manager
  std::string manager_name = "/op3_manager";
  while (rclcpp::ok())
  {
    rclcpp::sleep_for(std::chrono::seconds(1));

    if (checkManagerRunning(nh, manager_name) == true)
    {
      break;
      RCLCPP_INFO(nh->get_logger(), "Succeed to connect");
    }
    RCLCPP_WARN(nh->get_logger(), "Waiting for op3 manager");
  }

  readyToDemo();

  //node loop
  while (rclcpp::ok())
  {
    // process

    //execute pending callbacks
    rclcpp::spin_some(nh);

    //relax to fit output rate
    loop_rate.sleep();
  }

  //exit program
  rclcpp::shutdown();
  return 0;
}

void buttonHandlerCallback(const std_msgs::msg::String::SharedPtr msg)
{
  // starting demo using robotis_controller
  if (msg->data == "mode")
  {
    control_module = Framework;
    RCLCPP_INFO(rclcpp::get_logger("read_write_demo"), "Button : mode | Framework");
    readyToDemo();
  }
  // starting demo using direct_control_module
  else if (msg->data == "start")
  {
    control_module = DirectControlModule;
    RCLCPP_INFO(rclcpp::get_logger("read_write_demo"), "Button : start | Direct control module");
    readyToDemo();
  }
  // torque on all joints of ROBOTIS-OP3
  else if (msg->data == "user")
  {
    torqueOnAll();
    control_module = None;
  }
}

void jointstatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if(control_module == None)
    return;

  sensor_msgs::msg::JointState write_msg;
  write_msg.header = msg->header;

  for(int ix = 0; ix < msg->name.size(); ix++)
  {
    std::string joint_name = msg->name[ix];
    double joint_position = msg->position[ix];

    // mirror and copy joint angles from right to left
    if(joint_name == "r_sho_pitch")
    {
      write_msg.name.push_back("r_sho_pitch");
      write_msg.position.push_back(joint_position);
      write_msg.name.push_back("l_sho_pitch");
      write_msg.position.push_back(-joint_position);
    }
    if(joint_name == "r_sho_roll")
    {
      write_msg.name.push_back("r_sho_roll");
      write_msg.position.push_back(joint_position);
      write_msg.name.push_back("l_sho_roll");
      write_msg.position.push_back(-joint_position);
    }
    if(joint_name == "r_el")
    {
      write_msg.name.push_back("r_el");
      write_msg.position.push_back(joint_position);
      write_msg.name.push_back("l_el");
      write_msg.position.push_back(-joint_position);
    }
  }

  // publish a message to set the joint angles
  if(control_module == Framework)
    write_joint_pub->publish(write_msg);
  else if(control_module == DirectControlModule)
    write_joint_pub2->publish(write_msg);
}

void readyToDemo()
{
  RCLCPP_INFO(rclcpp::get_logger("read_write_demo"), "Start Read-Write Demo");
  // turn off LED
  setLED(0x04);

  torqueOnAll();
  RCLCPP_INFO(rclcpp::get_logger("read_write_demo"), "Torque on All joints");

  // send message for going init posture
  goInitPose();
  RCLCPP_INFO(rclcpp::get_logger("read_write_demo"), "Go Init pose");

  // wait while ROBOTIS-OP3 goes to the init posture.
  rclcpp::sleep_for(std::chrono::seconds(4));

  // turn on R/G/B LED [0x01 | 0x02 | 0x04]
  setLED(control_module);

  // change the module for demo
  if(control_module == Framework)
  {
    setModule("none");
    RCLCPP_INFO(rclcpp::get_logger("read_write_demo"), "Change module to none");
  }
  else if(control_module == DirectControlModule)
  {
    setModule("direct_control_module");
    RCLCPP_INFO(rclcpp::get_logger("read_write_demo"), "Change module to direct_control_module");
  }
  else
    return;

  // torque off : right arm
  torqueOff("right");
  RCLCPP_INFO(rclcpp::get_logger("read_write_demo"), "Torque off");
}

void goInitPose()
{
  std_msgs::msg::String init_msg;
  init_msg.data = "ini_pose";

  init_pose_pub->publish(init_msg);
}

void setLED(int led)
{
  robotis_controller_msgs::msg::SyncWriteItem syncwrite_msg;
  syncwrite_msg.item_name = "LED";
  syncwrite_msg.joint_name.push_back("open-cr");
  syncwrite_msg.value.push_back(led);

  sync_write_pub->publish(syncwrite_msg);
}

bool checkManagerRunning(rclcpp::Node::SharedPtr node, std::string& manager_name)
{
  auto node_list = node->get_node_names();

  for (const auto& node_name : node_list)
  {
    if (node_name == manager_name)
      return true;
  }

  RCLCPP_ERROR(rclcpp::get_logger("read_write_demo"), "Can't find op3_manager");
  return false;
}

void setModule(const std::string& module_name)
{
  auto request = std::make_shared<robotis_controller_msgs::srv::SetModule::Request>();
  request->module_name = module_name;

  auto result = set_joint_module_client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(rclcpp::Node::make_shared("set_module_node"), result) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_demo"), "Failed to set module");
    return;
  }

  return ;
}

void torqueOnAll()
{
  std_msgs::msg::String check_msg;
  check_msg.data = "check";

  dxl_torque_pub->publish(check_msg);
}

void torqueOff(const std::string& body_side)
{
  robotis_controller_msgs::msg::SyncWriteItem syncwrite_msg;
  int torque_value = 0;
  syncwrite_msg.item_name = "torque_enable";

  if(body_side == "right")
  {
    syncwrite_msg.joint_name.push_back("r_sho_pitch");
    syncwrite_msg.value.push_back(torque_value);
    syncwrite_msg.joint_name.push_back("r_sho_roll");
    syncwrite_msg.value.push_back(torque_value);
    syncwrite_msg.joint_name.push_back("r_el");
    syncwrite_msg.value.push_back(torque_value);
  }
  else if(body_side == "left")
  {
    syncwrite_msg.joint_name.push_back("l_sho_pitch");
    syncwrite_msg.value.push_back(torque_value);
    syncwrite_msg.joint_name.push_back("l_sho_roll");
    syncwrite_msg.value.push_back(torque_value);
    syncwrite_msg.joint_name.push_back("l_el");
    syncwrite_msg.value.push_back(torque_value);
  }
  else
    return;

  sync_write_pub->publish(syncwrite_msg);
}
