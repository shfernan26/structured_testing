#include <gtest/gtest.h>
#include "env_state.h"

TEST(FindTargetObject, FindClosestObject) {
    std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("FindTargetObject_FindClosestObject");
    EnvironmentState env_state_test(nh);

    ObjectState default_obj;

    ObjectState close_center_object(1, 6.0 , 1, 0.5, 0, 0, 1, 0, 0, 0); // id, dx, obj_lane, ...
    ObjectState far_center_object(2, 10.0 , 1, 0.5, 0, 0, 1, 0, 0, 0); // id, dx, obj_lane, ...
    
    // Add close and further objects to center lane track of trackedObjects
    env_state_test.add_object(close_center_object);
    env_state_test.add_object(far_center_object);
    
    
    rclcpp::Time start = nh->get_clock()->now();
    while (rclcpp::ok() && (nh->get_clock()->now() - start <= rclcpp::Duration::from_seconds(5.0)) ) {
      rclcpp::Time time_now = nh->now();

      // Stores the time difference since last time a message was received. If Kalman Filter
      // has not sent a message in the past UPDATE_TOL, then reset everything to publish default values
      double update_time_difference_centre = time_now.seconds() - env_state_test.last_msg_ros_timestamp_centre.seconds();
      double update_time_difference_left = time_now.seconds() - env_state_test.last_msg_ros_timestamp_left.seconds();
      double update_time_difference_right = time_now.seconds() - env_state_test.last_msg_ros_timestamp_right.seconds();
      if (update_time_difference_centre > UPDATE_TOL) {
        env_state_test.targetObjectsInLanes[0] = default_obj;
      }
      if (update_time_difference_left > UPDATE_TOL) {
        env_state_test.targetObjectsInLanes[1] = default_obj;
      }
      if (update_time_difference_right > UPDATE_TOL) {
        env_state_test.targetObjectsInLanes[2] = default_obj;
      }

      env_state_test.find_target_object(); 

      rclcpp::spin_some(nh);
      rclcpp::Rate(2000).sleep();
  }



    ASSERT_EQ(env_state_test.targetObjectsInLanes[0].get_obj_id(), 1);// check that this function finds closest object in center lane
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> test_env_state = rclcpp::Node::make_shared("test_env_state");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    rclcpp::shutdown();
}