#include "env_state.h"
#include <cinttypes>

int main(int argc, char** argv){
  rclcpp::init(argc, argv); 
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("env_state");
  EnvironmentState env_state = EnvironmentState(node);
  rclcpp::Time mem1 = node->now();
  ObjectState default_obj;

  while (rclcpp::ok()) {
    rclcpp::Time time_now = node->now();

    // Stores the time difference since last time a message was received. If Kalman Filter
    // has not sent a message in the past UPDATE_TOL, then reset everything to publish default values
    double update_time_difference_centre = time_now.seconds() - env_state.last_msg_ros_timestamp_centre.seconds();
    double update_time_difference_left = time_now.seconds() - env_state.last_msg_ros_timestamp_left.seconds();
    double update_time_difference_right = time_now.seconds() - env_state.last_msg_ros_timestamp_right.seconds();
    if (update_time_difference_centre > UPDATE_TOL) {
      env_state.targetObjectsInLanes[0] = default_obj;
    }
    if (update_time_difference_left > UPDATE_TOL) {
      env_state.targetObjectsInLanes[1] = default_obj;
    }
    if (update_time_difference_right > UPDATE_TOL) {
      env_state.targetObjectsInLanes[2] = default_obj;
    }

    // While the env_state node is running, we should be checking validity of messages
    // to remove outdated objects as well as updating the 3 tracked obj and target obj to publish
    env_state.check_tracked_time(); 
    env_state.find_target_object(); 

    if (time_now.seconds() - mem1.seconds() > 0.01) {
      env_state.publish_target_obj();
      env_state.publish_tracked_obj();
      env_state.publish_all_tracked_obj();
      env_state.publish_binary_class();
      env_state.global_clk += 0.01;
      // env_state.counter = env_state.counter + 0.01;
      // env_state.counter = env_state.counter + (time_now.seconds() - mem1.seconds());
      mem1 = time_now;
    }

    rclcpp::spin_some(node);
    rclcpp::Rate(2000).sleep();
  }
	// ros::spin();

  return 0;
}
