#ifndef __MASTER_TASK_H__
#define __MASTER_TASK_H__

#include "rclcpp/rclcpp.hpp"

#include <common/msg/can_comms_data_msg.hpp>
#include <common/msg/drive_ctrl_input_msg.hpp>
#include <common/msg/acc_output_msg.hpp>
#include <common/msg/v2x_data_msg.hpp>
#include <thread>
#include <time.h>

static const uint8_t MASTER_MESSAGE_BUFFER_SIZE = 10;

class MasterTask {
 public:
  MasterTask(rclcpp::Node::SharedPtr nodeHandle);
  virtual ~MasterTask();

  bool DEBUG = 0;
  bool tests_passed = 0;
  void print_debug();

  void publish_can_comms_msg();

  void drive_ctrl_msg_callback(const common::msg::DriveCtrlInputMsg::SharedPtr drive_ctrl_msg);
  void acc_output_msg_callback(const common::msg::AccOutputMsg::SharedPtr acc_msg);
  void v2x_callback(const common::msg::V2xDataMsg::SharedPtr v2x_data);
  
  /**
   * If ACC is valid and the overhead V2X switch is engaged,
   * indicate that V2X can be engaged
   */
  void V2X_Engagement();

  /**
  * If ACC is IN USE and any OFF condition is requested, 
  * system shall remain at the target speed for a set time buffer before reverting to driver inputs
  * Basically to not turn off ACC if some test fails for an insignificant amount of time
  */
  void ACC_1_1(); 

  /**
  * A requested turn larger than 15 degrees shall instantly turn off ACC
  */
  void ACC_15();

  /**
  * To ensure enable data is correct, 
  * 500 ms of a continuously active Enable signal must pass between the enabling of the ACC and the actual ACC engagement
  */
  void ACC_16();

  /**
  * signal error message sent to the Jetson + ACC disabled if, within period_max,
  * signal (ACC_activatioin) is in the opposite state for more than 120 ms cumulatively
  */
  void ACC_18();

  /**
  * ACC Allowed signal shall prevent the use of the ACC algorithm if it is not active
  */
  void ACC_20();
  
  /**
  * Automated braking commands cannot exceed 0.5 g deceleration
  */
  void CAV_2_2();

  /**
  * CAVS rolling counter shall be sent every CAN frame
  */
  void INT_1();
  
  /**
  * If the HSC alive rolling counter is not functioning as intended for 50 ms, 
  * all signals being sent to the HSC shall be set to zero/default values until 
  * the rolling counter is functioning properly for 100 ms
  */
  void INT_2();

  /**
  * If the Jetson alive rolling counter is not functioning as intended for 50 ms, 
  * all signals being sent to the CAVS controller shall be set to zero/default values until 
  * the rolling counter is functioning properly for 100 ms
  */
  void INT_7();

  common::msg::CanCommsDataMsg get_can_comms_msg();

 protected:
 private:
  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<common::msg::DriveCtrlInputMsg>::SharedPtr drive_ctrl_sub;
  rclcpp::Subscription<common::msg::AccOutputMsg>::SharedPtr acc_sub;
  rclcpp::Subscription<common::msg::V2xDataMsg>::SharedPtr v2x_sub;
  rclcpp::Publisher<common::msg::CanCommsDataMsg>::SharedPtr master_task_pub;
  common::msg::CanCommsDataMsg can_comms_msg;

  bool ACC_ALLOWED = 0;
  bool V2X_OVERHEAD_SWITCH;

  std::vector<std::string> failed_tests;
  void append_debug_msg(std::string message);

  uint64_t alive_rolling_counter_jetson;
  uint64_t prev_rolling_counter_Jetson;
  bool first_RC_Jetson = true;
  rclcpp::Time prev_time_rc_jetson;
  rclcpp::Time prev_time_correct_jetson = rclcpp::Time(0);
  bool initial_Jetson_alive = true;

  uint64_t alive_rolling_counter_mabx; 
  uint64_t prev_rolling_counter_MABx; 
  bool first_RC = true;
  rclcpp::Time prev_time_rc;
  rclcpp::Time prev_time_correct_mabx = rclcpp::Time(0);
  bool initial_HSC_alive = true;

  double acc_speed_set_point;
  int acc_gap_level;
  double VEHICLE_SPEED;
  double STEERING_ANGLE;
  //acc topic 
  bool ACC_FAULT;
  double ACC_ACCEL;
  //v2x topic
  bool V2X_ENABLED;
  double V2X_ACCEL;


  rclcpp::Duration buffer_time = rclcpp::Duration(0.2);  // ACC_1_1

  bool initial_OFF_request_2 = true;
  bool initial_OFF_request_3 = true;
  bool initial_OFF_request_4 = true;
  bool initial_OFF_request_5 = true;
  bool initial_OFF_request_6 = true;
  bool initial_OFF_request_7 = true;
  bool initial_OFF_request_8 = true;

  rclcpp::Time rc_time_init;
  rclcpp::Time rc_time_init_jetson;
  rclcpp::Time acc_period_start;
  rclcpp::Time ten_sec_start_acc;
  rclcpp::Time last_time_tests_passed;
  rclcpp::Time begin3;
  rclcpp::Time begin4;
  rclcpp::Time begin5;
  rclcpp::Time begin6;
  rclcpp::Time begin7;
  rclcpp::Time begin8;
  rclcpp::Time begin9;

  unsigned int opposite_counter = 0;
  bool prev_period_result_acc = 1;

  double prev_vel; //ego vehicle previous velocity, used to calculate acceleration of ego vehicle
  double curr_vel; //ego vehicle current velocity
  double ego_accel;
  rclcpp::Time prev_time;

  // constants
  rclcpp::Duration ACC_18_INTERVAL = rclcpp::Duration(0.01);
  rclcpp::Duration PERIOD_MAX = rclcpp::Duration(0.3); // ACC_18 
  unsigned long OPPOSITE_MAX = 12; // ACC_18 
};

#endif  // __MASTER_TASK_H__