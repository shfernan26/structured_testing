#include <gtest/gtest.h>
#include "master_task.h"

double GRACE_PERIOD = 0.05;

TEST(ACC_1_1, validInput) {

  std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("ACC11_validInput");
  MasterTask master_task_test(nh);
  // For some reason this method produces gtest errors so the subsequent implementation is used
  // common::msg::DriveCtrlInputMsg::SharedPtr drive_ctrl_msg; 
  // drive_ctrl_msg.acc_allowed = 1;
 
  common::msg::DriveCtrlInputMsg temp_drive_ctrl_msg;
  std::shared_ptr<common::msg::DriveCtrlInputMsg> drive_ctrl_msg;
  temp_drive_ctrl_msg.acc_allowed = 1;
  drive_ctrl_msg = std::make_shared<common::msg::DriveCtrlInputMsg>(temp_drive_ctrl_msg);

  common::msg::AccOutputMsg temp_acc_msg;
  std::shared_ptr<common::msg::AccOutputMsg> acc_msg;
  temp_acc_msg.acc_fault= false;
  acc_msg = std::make_shared<common::msg::AccOutputMsg>(temp_acc_msg);

  rclcpp::Time start = nh->get_clock()->now();
  while (nh->get_clock()->now() - start <= rclcpp::Duration::from_seconds(3.0)){
    master_task_test.drive_ctrl_msg_callback(drive_ctrl_msg);
    master_task_test.acc_output_msg_callback(acc_msg);
    master_task_test.ACC_16();
    master_task_test.ACC_1_1();
    common::msg::CanCommsDataMsg can_msg = master_task_test.get_can_comms_msg();
    rclcpp::Duration timePassed = nh->get_clock()->now() - start;

    if (timePassed > rclcpp::Duration::from_seconds(0.5 + GRACE_PERIOD)){
      ASSERT_EQ(1, can_msg.acc_valid);
    }
    rclcpp::Rate(200).sleep();
  }

}

/**
 * ALL TESTS BELOW MUST BE CONVERTED FROM ROS TO ROS2
 */

// TEST(ACC_1_1, invalidInput) {
//   rclcpp::NodeHandle nh;
//   MasterTask master_task_test(&nh);

//   common::DriveCtrlInputMsg drive_ctrl_msg;
//   drive_ctrl_msg.acc_allowed = 1;
//   common::AccOutputMsg acc_msg;
//   acc_msg.acc_fault = 0;
//   common::SensorDiagnosticFlagCH2::Request req_CH2;
//   common::SensorDiagnosticFlagCH2::Response res_CH2;
//   req_CH2.front_radar = 1;
//   common::SensorDiagnosticFlagCH4::Request req_CH4;
//   common::SensorDiagnosticFlagCH4::Response res_CH4;
//   req_CH4.mobileye = 0;

//   rclcpp::Rate rate(200);
//   rclcpp::SteadyTime start = rclcpp::SteadyTime::now();
//   while (rclcpp::SteadyTime::now() - start <= rclcpp::WallDuration(3)){
//     master_task_test.drive_ctrl_msg_callback(drive_ctrl_msg);
//     master_task_test.acc_output_msg_callback(acc_msg);
//     master_task_test.sensor_diagnostic_callback_CH2(req_CH2, res_CH2);
//     master_task_test.sensor_diagnostic_callback_CH4(req_CH4, res_CH4);
//     master_task_test.ACC_16();
//     master_task_test.CAV_1_5();
//     master_task_test.ACC_1_1();
//     common::CanCommsDataMsg can_msg = master_task_test.get_can_comms_msg();
//     ASSERT_EQ(0, can_msg.acc_valid);
//     rate.sleep();
//   }
// }

// TEST(ACC_1_1, bufferCheck) {
//   rclcpp::NodeHandle nh;
//   MasterTask master_task_test(&nh);

//   common::DriveCtrlInputMsg drive_ctrl_msg;
//   drive_ctrl_msg.acc_allowed = 1;
//   common::AccOutputMsg acc_msg;
//   acc_msg.acc_fault = 0;
//   common::SensorDiagnosticFlagCH2::Request req_CH2;
//   common::SensorDiagnosticFlagCH2::Response res_CH2;
//   req_CH2.front_radar = 1;
//   common::SensorDiagnosticFlagCH4::Request req_CH4;
//   common::SensorDiagnosticFlagCH4::Response res_CH4;
//   req_CH4.mobileye = 1;

//   rclcpp::Rate rate(200);
//   rclcpp::SteadyTime start = rclcpp::SteadyTime::now();
//   while (rclcpp::SteadyTime::now() - start <= rclcpp::WallDuration(1)){
//     master_task_test.drive_ctrl_msg_callback(drive_ctrl_msg);
//     master_task_test.acc_output_msg_callback(acc_msg);
//     master_task_test.sensor_diagnostic_callback_CH2(req_CH2, res_CH2);
//     master_task_test.sensor_diagnostic_callback_CH4(req_CH4, res_CH4);
//     master_task_test.ACC_16();
//     master_task_test.CAV_1_5();
//     master_task_test.ACC_1_1();
//     common::CanCommsDataMsg can_msg = master_task_test.get_can_comms_msg();
//     rclcpp::WallDuration timePassed = rclcpp::SteadyTime::now() - start;
//     if (timePassed <= rclcpp::WallDuration(0.5 - GRACE_PERIOD)){
//       ASSERT_EQ(0, can_msg.acc_valid);
//     }
//     else if (timePassed > rclcpp::WallDuration(0.5 + GRACE_PERIOD)){
//       ASSERT_EQ(1, can_msg.acc_valid);
//     }
//     rate.sleep();
//   }
//   req_CH4.mobileye = 0;
//   start = rclcpp::SteadyTime::now();
//   while (rclcpp::SteadyTime::now() - start <= rclcpp::WallDuration(1)){
//     master_task_test.drive_ctrl_msg_callback(drive_ctrl_msg);
//     master_task_test.acc_output_msg_callback(acc_msg);
//     master_task_test.sensor_diagnostic_callback_CH2(req_CH2, res_CH2);
//     master_task_test.sensor_diagnostic_callback_CH4(req_CH4, res_CH4);
//     master_task_test.ACC_16();
//     master_task_test.CAV_1_5();
//     master_task_test.ACC_1_1();
//     common::CanCommsDataMsg can_msg = master_task_test.get_can_comms_msg();
//     rclcpp::WallDuration timePassed = rclcpp::SteadyTime::now() - start;
//     if (timePassed <= rclcpp::WallDuration(0.2 - GRACE_PERIOD)){
//       ASSERT_EQ(1, can_msg.acc_valid);
//     }
//     else if (timePassed > rclcpp::WallDuration(0.2 + GRACE_PERIOD)){
//       ASSERT_EQ(0, can_msg.acc_valid);
//     }
//     rate.sleep();
//   }
//   req_CH4.mobileye = 1;
//   start = rclcpp::SteadyTime::now();
//   while (rclcpp::SteadyTime::now() - start <= rclcpp::WallDuration(1)){
//     master_task_test.drive_ctrl_msg_callback(drive_ctrl_msg);
//     master_task_test.acc_output_msg_callback(acc_msg);
//     master_task_test.sensor_diagnostic_callback_CH2(req_CH2, res_CH2);
//     master_task_test.sensor_diagnostic_callback_CH4(req_CH4, res_CH4);
//     master_task_test.ACC_16();
//     master_task_test.CAV_1_5();
//     master_task_test.ACC_1_1();
//     common::CanCommsDataMsg can_msg = master_task_test.get_can_comms_msg();
//     rclcpp::WallDuration timePassed = rclcpp::SteadyTime::now() - start;
//     ASSERT_EQ(1, can_msg.acc_valid);
//     rate.sleep();
//   }
// }

// TEST(ACC_16, validInput) {
//   rclcpp::NodeHandle nh;
//   MasterTask master_task_test(&nh);

//   common::DriveCtrlInputMsg drive_ctrl_msg;
//   drive_ctrl_msg.acc_allowed = 1;

//   rclcpp::Rate rate(200);
//   rclcpp::SteadyTime start = rclcpp::SteadyTime::now();
//   while (rclcpp::SteadyTime::now() - start <= rclcpp::WallDuration(3)){
//     master_task_test.drive_ctrl_msg_callback(drive_ctrl_msg);
//     master_task_test.ACC_16();
//     rclcpp::WallDuration timePassed = rclcpp::SteadyTime::now() - start;
//     if (timePassed <= rclcpp::WallDuration(0.5 - GRACE_PERIOD)){
//       ASSERT_EQ(0, master_task_test.tests_passed);
//     }
//     else if (timePassed > rclcpp::WallDuration(0.5 + GRACE_PERIOD)){
//       ASSERT_EQ(1, master_task_test.tests_passed);
//     }
//     master_task_test.print_debug();
//     rate.sleep();
//   }
// }

// TEST(ACC_16, invalidInput) {
//   rclcpp::NodeHandle nh;
//   MasterTask master_task_test(&nh);
//   common::DriveCtrlInputMsg drive_ctrl_msg;
//   drive_ctrl_msg.acc_allowed = 0;

//   rclcpp::Rate rate(200);
//   rclcpp::SteadyTime start = rclcpp::SteadyTime::now();
//   while (rclcpp::SteadyTime::now() - start <= rclcpp::WallDuration(3)){
//     master_task_test.drive_ctrl_msg_callback(drive_ctrl_msg);
//     master_task_test.ACC_16();
//     ASSERT_EQ(0, master_task_test.tests_passed);
//     master_task_test.print_debug();
//     rate.sleep();
//   }
// }

// TEST(ACC_18, validInput) {
//   rclcpp::NodeHandle nh;
//   MasterTask master_task_test(&nh);
//   common::DriveCtrlInputMsg drive_ctrl_msg;
//   drive_ctrl_msg.acc_allowed = 1;

//   rclcpp::Rate rate(200);

//   rclcpp::SteadyTime start = rclcpp::SteadyTime::now();
//   while (rclcpp::SteadyTime::now() - start <= rclcpp::WallDuration(0.5 + GRACE_PERIOD)){
//     master_task_test.drive_ctrl_msg_callback(drive_ctrl_msg);
//     master_task_test.tests_passed = 1;
//     master_task_test.ACC_1_1();
//     master_task_test.ACC_18();
//     master_task_test.print_debug();
//     rate.sleep();
//   }
//   start = rclcpp::SteadyTime::now();
//   while (rclcpp::SteadyTime::now() - start > rclcpp::WallDuration(0.5 + GRACE_PERIOD) && 
//   rclcpp::SteadyTime::now() - start <= rclcpp::WallDuration(3)){
//     master_task_test.drive_ctrl_msg_callback(drive_ctrl_msg);
//     master_task_test.tests_passed = 1;
//     master_task_test.ACC_1_1();
//     master_task_test.ACC_18();
//     ASSERT_EQ(1, master_task_test.tests_passed);
//     master_task_test.print_debug();
//     rate.sleep();
//   }
// }

// TEST(ACC_18, fullCheck) {
//   rclcpp::NodeHandle nh;
//   MasterTask master_task_test(&nh);
//   common::DriveCtrlInputMsg drive_ctrl_msg;
//   drive_ctrl_msg.acc_allowed = 1;
//   rclcpp::Rate rate(200);

//   // start with valid input
//   rclcpp::SteadyTime start = rclcpp::SteadyTime::now();
//   while (rclcpp::SteadyTime::now() - start <= rclcpp::WallDuration(0.5 + GRACE_PERIOD)){
//     master_task_test.drive_ctrl_msg_callback(drive_ctrl_msg);
//     master_task_test.tests_passed = 1;
//     master_task_test.ACC_1_1(); // to set acc_valid to true
//     master_task_test.ACC_18();
//     master_task_test.print_debug();
//     rate.sleep();
//   }
//   start = rclcpp::SteadyTime::now();
//   while (rclcpp::SteadyTime::now() - start > rclcpp::WallDuration(0.5 + GRACE_PERIOD) && 
//   rclcpp::SteadyTime::now() - start <= rclcpp::WallDuration(2)){
//     master_task_test.drive_ctrl_msg_callback(drive_ctrl_msg);
//     master_task_test.tests_passed = 1;
//     master_task_test.ACC_1_1();
//     master_task_test.ACC_18();
//     ASSERT_EQ(1, master_task_test.tests_passed);
//     master_task_test.print_debug();
//     rate.sleep();
//   }
//   // now make input invalid for ~ 300ms
//   start = rclcpp::SteadyTime::now();
//   while (rclcpp::SteadyTime::now() - start <= rclcpp::WallDuration(1)){
//     master_task_test.tests_passed = 1;
//     drive_ctrl_msg.acc_allowed = 0;
//     master_task_test.drive_ctrl_msg_callback(drive_ctrl_msg);
//     master_task_test.ACC_1_1();
//     master_task_test.ACC_18();
//     master_task_test.print_debug();
//     rate.sleep();
//   }
//   start = rclcpp::SteadyTime::now();
//   while (rclcpp::SteadyTime::now() - start <= rclcpp::WallDuration(1)){
//     master_task_test.tests_passed = 1;
//     drive_ctrl_msg.acc_allowed = 0;
//     master_task_test.drive_ctrl_msg_callback(drive_ctrl_msg);
//     master_task_test.ACC_1_1();
//     master_task_test.ACC_18();
//     ASSERT_EQ(0, master_task_test.tests_passed);
//     master_task_test.print_debug();
//     rate.sleep();
//   }
//   // make input valid again
//   start = rclcpp::SteadyTime::now();
//   drive_ctrl_msg.acc_allowed = 1;
//   while (rclcpp::SteadyTime::now() - start <= rclcpp::WallDuration(0.6)){
//     master_task_test.drive_ctrl_msg_callback(drive_ctrl_msg);
//     master_task_test.tests_passed = 1;
//     master_task_test.ACC_1_1();
//     master_task_test.ACC_18();
//     master_task_test.print_debug();
//     rate.sleep();
//   }
//   start = rclcpp::SteadyTime::now();
//   while (rclcpp::SteadyTime::now() - start <= rclcpp::WallDuration(1)){
//     master_task_test.drive_ctrl_msg_callback(drive_ctrl_msg);
//     master_task_test.tests_passed = 1;
//     master_task_test.ACC_1_1();
//     master_task_test.ACC_18();
//     ASSERT_EQ(1, master_task_test.tests_passed);
//     master_task_test.print_debug();
//     rate.sleep();
//   }
// }

// TEST(INT_2, validInput) {
//   rclcpp::NodeHandle nh;
//   MasterTask master_task_test(&nh);
//   common::DriveCtrlInputMsg drive_ctrl_msg;
//   drive_ctrl_msg.acc_allowed = 1;
//   drive_ctrl_msg.alive_rolling_counter_mabx = 0;

//   rclcpp::Rate rate(200);
//   rclcpp::SteadyTime start = rclcpp::SteadyTime::now();
//   while (rclcpp::SteadyTime::now() - start <= rclcpp::WallDuration(3)){
//     drive_ctrl_msg.alive_rolling_counter_mabx += 1;
//     drive_ctrl_msg.alive_rolling_counter_mabx %= 16;
//     master_task_test.drive_ctrl_msg_callback(drive_ctrl_msg);
//     master_task_test.ACC_16();
//     master_task_test.INT_2();
//     rclcpp::WallDuration timePassed = rclcpp::SteadyTime::now() - start;
//     if (timePassed <= rclcpp::WallDuration(0.5 - GRACE_PERIOD)){
//       ASSERT_EQ(0, master_task_test.tests_passed);
//     }
//     else if (timePassed > rclcpp::WallDuration(0.5 + GRACE_PERIOD)){
//       ASSERT_EQ(1, master_task_test.tests_passed);
//     }
//     master_task_test.print_debug();
//     rate.sleep();
//   }
// }

// TEST(INT_2, invalidInput) {
//   rclcpp::NodeHandle nh;
//   MasterTask master_task_test(&nh);
//   common::DriveCtrlInputMsg drive_ctrl_msg;
//   drive_ctrl_msg.acc_allowed = 1;
//   drive_ctrl_msg.alive_rolling_counter_mabx = 0;

//   rclcpp::Rate rate(200);
//   rclcpp::SteadyTime start = rclcpp::SteadyTime::now();
//   while (rclcpp::SteadyTime::now() - start <= rclcpp::WallDuration(3)){
//     master_task_test.drive_ctrl_msg_callback(drive_ctrl_msg);
//     master_task_test.ACC_16();
//     master_task_test.INT_2();
//     ASSERT_EQ(0, master_task_test.tests_passed);
//     master_task_test.print_debug();
//     rate.sleep();
//   }
// }

// TEST(INT_2, fullCheck) {
//   rclcpp::NodeHandle nh;
//   MasterTask master_task_test(&nh);
//   common::DriveCtrlInputMsg drive_ctrl_msg;
//   drive_ctrl_msg.acc_allowed = 1;
//   drive_ctrl_msg.alive_rolling_counter_mabx = 0;

//   rclcpp::Rate rate(200);
//   rclcpp::SteadyTime start = rclcpp::SteadyTime::now();
//   while (rclcpp::SteadyTime::now() - start <= rclcpp::WallDuration(2)){
//     master_task_test.drive_ctrl_msg_callback(drive_ctrl_msg);
//     master_task_test.ACC_16();
//     master_task_test.INT_2();
//     ASSERT_EQ(0, master_task_test.tests_passed);
//     master_task_test.print_debug();
//     rate.sleep();
//   }
//   start = rclcpp::SteadyTime::now();
//   while (rclcpp::SteadyTime::now() - start <= rclcpp::WallDuration(2)){
//     drive_ctrl_msg.alive_rolling_counter_mabx += 1;
//     drive_ctrl_msg.alive_rolling_counter_mabx %= 16;
//     master_task_test.drive_ctrl_msg_callback(drive_ctrl_msg);
//     master_task_test.ACC_16();
//     master_task_test.INT_2();
//     rclcpp::WallDuration timePassed = rclcpp::SteadyTime::now() - start;
//     if (timePassed <= rclcpp::WallDuration(0.1 - GRACE_PERIOD)){
//       ASSERT_EQ(0, master_task_test.tests_passed);
//     }
//     else if (timePassed > rclcpp::WallDuration(0.1 + GRACE_PERIOD)){
//       ASSERT_EQ(1, master_task_test.tests_passed);
//     }
//     master_task_test.print_debug();
//     rate.sleep();
//   }
//   start = rclcpp::SteadyTime::now();
//   while (rclcpp::SteadyTime::now() - start <= rclcpp::WallDuration(1)){
//     master_task_test.drive_ctrl_msg_callback(drive_ctrl_msg);
//     master_task_test.ACC_16();
//     master_task_test.INT_2();
//     rclcpp::WallDuration timePassed = rclcpp::SteadyTime::now() - start;
//     if (timePassed <= rclcpp::WallDuration(0.05 - GRACE_PERIOD)){
//       ASSERT_EQ(1, master_task_test.tests_passed);
//     }
//     else if (timePassed > rclcpp::WallDuration(0.05 + GRACE_PERIOD)){
//       ASSERT_EQ(0, master_task_test.tests_passed);
//     }
//     master_task_test.print_debug();
//     rate.sleep();
//   }
// }

// TEST(INT_7, validInput) {
//   rclcpp::NodeHandle nh;
//   MasterTask master_task_test(&nh);
//   common::DriveCtrlInputMsg drive_ctrl_msg;
//   drive_ctrl_msg.acc_allowed = 1;
//   drive_ctrl_msg.alive_rolling_counter_jetson = 0;

//   rclcpp::Rate rate(200);
//   rclcpp::SteadyTime start = rclcpp::SteadyTime::now();
//   while (rclcpp::SteadyTime::now() - start <= rclcpp::WallDuration(3)){
//     drive_ctrl_msg.alive_rolling_counter_jetson += 1;
//     drive_ctrl_msg.alive_rolling_counter_jetson %= 16;
//     master_task_test.drive_ctrl_msg_callback(drive_ctrl_msg);
//     master_task_test.ACC_16();
//     master_task_test.INT_7();
//     common::CanCommsDataMsg can_msg = master_task_test.get_can_comms_msg();
//     rclcpp::WallDuration timePassed = rclcpp::SteadyTime::now() - start;
//     if (timePassed <= rclcpp::WallDuration(0.5 - GRACE_PERIOD)){
//       ASSERT_EQ(0, master_task_test.tests_passed);
//     }
//     else if (timePassed > rclcpp::WallDuration(0.5 + GRACE_PERIOD)){
//       ASSERT_EQ(1, master_task_test.tests_passed);
//     }
//     master_task_test.print_debug();
//     rate.sleep();
//   }
// }

// TEST(INT_7, invalidInput) {
//   rclcpp::NodeHandle nh;
//   MasterTask master_task_test(&nh);
//   common::DriveCtrlInputMsg drive_ctrl_msg;
//   drive_ctrl_msg.acc_allowed = 1;
//   drive_ctrl_msg.alive_rolling_counter_jetson = 0;

//   rclcpp::Rate rate(200);
//   rclcpp::SteadyTime start = rclcpp::SteadyTime::now();
//   while (rclcpp::SteadyTime::now() - start <= rclcpp::WallDuration(3)){
//     master_task_test.drive_ctrl_msg_callback(drive_ctrl_msg);
//     master_task_test.ACC_16();
//     master_task_test.INT_7();
//     ASSERT_EQ(0, master_task_test.tests_passed);
//     master_task_test.print_debug();
//     rate.sleep();
//   }
// }

// TEST(INT_7, fullCheck) {
//   rclcpp::NodeHandle nh;
//   MasterTask master_task_test(&nh);
//   common::DriveCtrlInputMsg drive_ctrl_msg;
//   drive_ctrl_msg.acc_allowed = 1;
//   drive_ctrl_msg.alive_rolling_counter_jetson = 0;

//   rclcpp::Rate rate(200);
//   rclcpp::SteadyTime start = rclcpp::SteadyTime::now();
//   while (rclcpp::SteadyTime::now() - start <= rclcpp::WallDuration(2)){
//     master_task_test.drive_ctrl_msg_callback(drive_ctrl_msg);
//     master_task_test.ACC_16();
//     master_task_test.INT_7();
//     ASSERT_EQ(0, master_task_test.tests_passed);
//     master_task_test.print_debug();
//     rate.sleep();
//   }
//   start = rclcpp::SteadyTime::now();
//   while (rclcpp::SteadyTime::now() - start <= rclcpp::WallDuration(2)){
//     drive_ctrl_msg.alive_rolling_counter_jetson += 1;
//     drive_ctrl_msg.alive_rolling_counter_jetson %= 16;
//     master_task_test.drive_ctrl_msg_callback(drive_ctrl_msg);
//     master_task_test.ACC_16();
//     master_task_test.INT_7();
//     rclcpp::WallDuration timePassed = rclcpp::SteadyTime::now() - start;
//     if (timePassed <= rclcpp::WallDuration(0.1 - GRACE_PERIOD)){
//       ASSERT_EQ(0, master_task_test.tests_passed);
//     }
//     else if (timePassed > rclcpp::WallDuration(0.1 + GRACE_PERIOD)){
//       ASSERT_EQ(1, master_task_test.tests_passed);
//     }
//     master_task_test.print_debug();
//     rate.sleep();
//   }
//   start = rclcpp::SteadyTime::now();
//   while (rclcpp::SteadyTime::now() - start <= rclcpp::WallDuration(1)){
//     master_task_test.drive_ctrl_msg_callback(drive_ctrl_msg);
//     master_task_test.ACC_16();
//     master_task_test.INT_2();
//     rclcpp::WallDuration timePassed = rclcpp::SteadyTime::now() - start;
//     if (timePassed <= rclcpp::WallDuration(0.05 - GRACE_PERIOD)){
//       ASSERT_EQ(1, master_task_test.tests_passed);
//     }
//     else if (timePassed > rclcpp::WallDuration(0.05 + GRACE_PERIOD)){
//       ASSERT_EQ(0, master_task_test.tests_passed);
//     }
//     master_task_test.print_debug();
    // rate.sleep();
//   }
// }


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // std::shared_ptr<rclcpp::Node> test_master_task_node = rclcpp::Node::make_shared("test_master_task");
  
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
  rclcpp::shutdown();
}