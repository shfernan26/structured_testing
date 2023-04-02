#include "master_task.h"

MasterTask::MasterTask(rclcpp::Node::SharedPtr nodeHandle) : node(nodeHandle) {
  std::function<void(const common::msg::DriveCtrlInputMsg::SharedPtr)> temp_drive_ctrl_callback = std::bind(&MasterTask::drive_ctrl_msg_callback, this, std::placeholders::_1);
  drive_ctrl_sub = node->create_subscription<common::msg::DriveCtrlInputMsg>("drive_ctrl_input", MASTER_MESSAGE_BUFFER_SIZE, temp_drive_ctrl_callback);
  
  std::function<void(const common::msg::AccOutputMsg::SharedPtr)> temp_acc_sub_callback = std::bind(&MasterTask::acc_output_msg_callback, this, std::placeholders::_1);
  acc_sub = node->create_subscription<common::msg::AccOutputMsg>("acc_output_msg", MASTER_MESSAGE_BUFFER_SIZE, temp_acc_sub_callback);
  
  std::function<void(const common::msg::V2xDataMsg::SharedPtr)> temp_v2x_callback = std::bind(&MasterTask::v2x_callback, this, std::placeholders::_1);
  v2x_sub = node->create_subscription<common::msg::V2xDataMsg>("v2x_deceleration", MASTER_MESSAGE_BUFFER_SIZE, temp_v2x_callback);

  master_task_pub = node->create_publisher<common::msg::CanCommsDataMsg>("can_comms_data", MASTER_MESSAGE_BUFFER_SIZE);

  // init timers
  ten_sec_start_acc = node->get_clock()->now();
  acc_period_start = node->get_clock()->now();
  last_time_tests_passed = node->get_clock()->now() - buffer_time;
}

MasterTask::~MasterTask() {}

void MasterTask::publish_can_comms_msg() { 
  master_task_pub->publish(can_comms_msg); 
  print_debug();
}

void MasterTask::drive_ctrl_msg_callback(const common::msg::DriveCtrlInputMsg::SharedPtr drive_ctrl_msg) 
{
    ACC_ALLOWED = drive_ctrl_msg->acc_allowed;
    V2X_OVERHEAD_SWITCH = drive_ctrl_msg->v2x_overhead_switch;

    alive_rolling_counter_mabx = drive_ctrl_msg->alive_rolling_counter_mabx;
    alive_rolling_counter_jetson = drive_ctrl_msg->alive_rolling_counter_jetson;

    acc_speed_set_point = drive_ctrl_msg->acc_speed_set_point;
    acc_gap_level = drive_ctrl_msg->acc_gap_level;
    VEHICLE_SPEED = drive_ctrl_msg->veh_spd;
    STEERING_ANGLE = drive_ctrl_msg->str_ang;
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Drive control callback called.");

}

void MasterTask::acc_output_msg_callback(const common::msg::AccOutputMsg::SharedPtr acc_msg)
{
    ACC_FAULT = acc_msg->acc_fault;
    ACC_ACCEL = acc_msg->acc_accel;
    can_comms_msg.long_accel = ACC_ACCEL;
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "acc output control callback called.");
    can_comms_msg.acc_fault = acc_msg->acc_fault;
}

void MasterTask::v2x_callback(const common::msg::V2xDataMsg::SharedPtr v2x_data){
  V2X_ENABLED = v2x_data->v2x_enabled;
  V2X_ACCEL = v2x_data->v2x_accel;
}

common::msg::CanCommsDataMsg MasterTask::get_can_comms_msg() { 
  return can_comms_msg; 
}

void MasterTask::print_debug(){
  if (DEBUG && !tests_passed){
    for (std::string error: failed_tests){
      std::cout << "Failed master task diagnostic: " << error << std::endl;
    }
  }
}

void MasterTask::V2X_Engagement(){
  if (can_comms_msg.acc_valid && V2X_OVERHEAD_SWITCH){
    can_comms_msg.v2x_valid = 1;
  }
  else
    can_comms_msg.v2x_valid = 0;
}

void MasterTask::append_debug_msg(std::string message){
  if (DEBUG)
    failed_tests.push_back(message);
}

// call this AFTER all other functions 
void MasterTask::ACC_1_1() {
    rclcpp::Time now = node->get_clock()->now();
    if (tests_passed) {
      last_time_tests_passed = now;
      can_comms_msg.acc_valid = 1;
    }
    else if (now - last_time_tests_passed < buffer_time){
      can_comms_msg.acc_valid = 1;
    }
    else {
      can_comms_msg.acc_valid = 0;
    }
}

// must be called before all other tests
void MasterTask::ACC_16() {
    failed_tests.clear();
    if (ACC_ALLOWED) {
      rclcpp::Time curr = node->get_clock()->now();
      if (initial_OFF_request_8) {
        begin9 = node->get_clock()->now();
        initial_OFF_request_8 = false;
        tests_passed = 0;
        append_debug_msg("ACC-16 - Overhead switch ACC_ALLOWED must be on for at least 500 ms before acc is valid");
      } 
      else if ((curr - begin9) >= rclcpp::Duration(0.5)) {
        tests_passed = 1;
      }
      else {
        tests_passed = 0;
        append_debug_msg("ACC-16 - Overhead switch ACC_ALLOWED must be on for at least 500 ms before acc is valid");
      }
    }
    else {
      initial_OFF_request_8 = true;
      tests_passed = 0;
    }
}

void MasterTask::ACC_18() {
  rclcpp::Time curr = node->get_clock()->now();
  if (curr - ten_sec_start_acc >= ACC_18_INTERVAL) {
    if ((can_comms_msg.acc_valid != ACC_ALLOWED)) {
      opposite_counter += 1;
    }
    ten_sec_start_acc = curr;
  }
  // check if period is over
  if ((curr - acc_period_start > PERIOD_MAX || opposite_counter >= OPPOSITE_MAX)) {  // reset period
    if (opposite_counter >= OPPOSITE_MAX) { // counter too high
      prev_period_result_acc = 0;
      tests_passed = 0;
      append_debug_msg("ACC-18 - CAV acc_valid opposite to overhead switch ACC_ALLOWED for too long");
    }
    else {
      prev_period_result_acc = 1;
    }
    acc_period_start = curr;
    opposite_counter = 0;
  }
  else if (!prev_period_result_acc){ // keep acc off if previous period failed
    tests_passed = 0;
    append_debug_msg("ACC-18 - CAV acc_valid opposite to overhead switch ACC_ALLOWED for too long");
  }
}

// redundant after ACC_16
// void MasterTask::ACC_20() {
//     if (!ACC_ALLOWED) {
//         can_comms_msg.acc_valid = 0;
//     }
// }


void MasterTask::INT_1() {
    can_comms_msg.alive_rolling_counter += 1;
    can_comms_msg.alive_rolling_counter %= 16;
}

void MasterTask::INT_2() {
  rclcpp::Time curr = node->get_clock()->now();
  // check if MABx rolling counter is valid / increasing by 1
  if (!first_RC && (alive_rolling_counter_mabx == (prev_rolling_counter_MABx + 1) % 16)) {
    if (initial_HSC_alive) {  // check if rolling counter requirements are satisfied for the first time after start
                              // up/last fault;
      initial_HSC_alive = false;
      rc_time_init = node->get_clock()->now();
    } 
    if ((curr - rc_time_init) > rclcpp::Duration(0.100)) {  // if rolling counter has been valid for more than 100 ms
      prev_time_correct_mabx = curr;
    }
    else{
      tests_passed = 0;
      append_debug_msg("INT-2 - MABx rolling counter must be valid for at least 100 ms");
    }
  } 
  else if ((curr - prev_time_correct_mabx) > rclcpp::Duration(0.050)) { // 50 ms buffer for rc to be incorrect
    initial_HSC_alive = true;
    tests_passed = 0;
    append_debug_msg("INT-2 - MABx rolling counter bad");
  }
  first_RC = false;
  prev_rolling_counter_MABx = alive_rolling_counter_mabx;
  prev_time_rc = curr;
}

void MasterTask::INT_7() {
  rclcpp::Time curr = node->get_clock()->now();
  // check if Jetson rolling counter is valid / increasing by 1
  if (!first_RC_Jetson && (alive_rolling_counter_jetson == (prev_rolling_counter_Jetson + 1) % 16)) {
    if (initial_Jetson_alive) {  // check if rolling counter requirements are satisfied for the first time after start
                                 // up/last fault;
      initial_Jetson_alive = false;
      rc_time_init_jetson = node->get_clock()->now();
    } 
    if ((curr - rc_time_init_jetson) > rclcpp::Duration(0.100)) {  // if rolling counter has been valid for more than 100 ms
      prev_time_correct_jetson = curr;
    }
    else{
      tests_passed = 0;
      append_debug_msg("INT-7 - Jetson rolling counter must be valid for at least 100 ms");
    }
  }   
  else if ((curr - prev_time_correct_jetson) > rclcpp::Duration(0.050)) { // 50 ms buffer for rc to be incorrect
    initial_Jetson_alive = true;
    tests_passed = 0;
    append_debug_msg("INT-7 - Jetson rolling counter bad");
  }
  first_RC_Jetson = false;
  prev_rolling_counter_Jetson = alive_rolling_counter_jetson;
  prev_time_rc_jetson = curr;
}