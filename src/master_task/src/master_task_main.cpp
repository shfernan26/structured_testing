#include "master_task.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("master_task");
  MasterTask master_task(node);  
  
  while (rclcpp::ok()) {
    /*ACC_16 and AEB_24 are the functions that initially enable acc and aeb
      and must be called before other functions which can only disable acc/aeb */
    master_task.ACC_16();
    /*-------------------------------------------------*/

    // master_task.ACC_15();
    // master_task.ACC_17();
    // master_task.ACC_20();
    master_task.INT_1();

    /***************************************************/
    // call this after all other functions
    master_task.ACC_1_1();
    master_task.V2X_Engagement();

    master_task.publish_can_comms_msg();

    rclcpp::spin_some(node);
    rclcpp::Rate(200).sleep(); // the sleep must be less than 50 ms
  }

  rclcpp::shutdown();
  return 0;
}