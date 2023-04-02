#ifndef __ENV_STATE_H__
#define __ENV_STATE_H__

#include <rclcpp/rclcpp.hpp>
#include "object_state.h"
#include "common/msg/filtered_object_msg.hpp" // sub
#include "common/msg/tracked_output_msg.hpp"  // pub
#include "common/msg/target_output_msg.hpp"  // pub
#include "common/msg/binary_class_msg.hpp"
#include "common/srv/env_state_srv.hpp"  //service


#include <vector>

static const uint8_t MESSAGE_BUFFER_SIZE = 10;
// #define COUNTER_LIM 2
#define UPDATE_TOL 1
#define ERASE_TOL 1
#define MAX_OBJ 32

class EnvironmentState {
  public:
    EnvironmentState(std::shared_ptr<rclcpp::Node> node);
    virtual ~EnvironmentState();
    void publish_target_obj(); /*!< Publishes single target object in ego lane. */
    void publish_tracked_obj(); /*!< Publishes target object in adjacent and ego lanes. */
    void publish_all_tracked_obj(); /*!< Publishes all currently tracked objects. */
    void publish_binary_class();
    void filtered_object_callback(const common::msg::FilteredObjectMsg::SharedPtr filtered_msg); /*!< Called when KF publishes to filtered_object topic. */
    common::msg::TrackedOutputMsg get_tracked_output_msg();
    common::msg::TargetOutputMsg get_target_output_msg();

    void add_object(const ObjectState& tracked_msg); /*!< Add object to tracked objects vector. */
    void update_object(const ObjectState& tracked_msg, size_t index); /*!< Update tracked object msg at specfied index. */
    void check_tracked_time();  /*!< Check when object was last seen. If over threshold, erase. */
    void update_env_state(const ObjectState& tracked_msg);  /*!< Update env state vector depending on object match or not. */
    void find_target_object();  /*!< Assigns target object in each lane based on dx or ID match. */

    void env_state_srv_callback(const std::shared_ptr<common::srv::EnvStateSrv::Request>, std::shared_ptr<common::srv::EnvStateSrv::Response> res);
    bool changed_lane(int target_lane);

    std::vector<ObjectState> trackedObjects;
    ObjectState targetObjectsInLanes[3];
    double global_clk;
    double prev_time[3] = {-1,-1,-1};
    double prev_time_target = -1;
    // double counter = 0;
    rclcpp::Time last_msg_ros_timestamp_left;
    rclcpp::Time last_msg_ros_timestamp_centre;
    rclcpp::Time last_msg_ros_timestamp_right;
    double last_msg_timestamp;

   private:
    std::shared_ptr<rclcpp::Node> node;

    std::function<void(const common::msg::FilteredObjectMsg::SharedPtr)> temp_filtered_object_callback;
    rclcpp::Subscription<common::msg::FilteredObjectMsg>::SharedPtr filtered_object_sub;
    rclcpp::Publisher<common::msg::TrackedOutputMsg>::SharedPtr tracked_obj_pub;
    rclcpp::Publisher<common::msg::TargetOutputMsg>::SharedPtr all_tracked_obj_pub;
    rclcpp::Publisher<common::msg::TargetOutputMsg>::SharedPtr target_obj_pub;
    rclcpp::Publisher<common::msg::BinaryClassMsg>::SharedPtr binary_class_pub;
    common::msg::TrackedOutputMsg tracked_output_msg;
    common::msg::TargetOutputMsg target_output_msg;
    common::msg::TargetOutputMsg all_tracked_output_msg;
    // std::function<void(const common::srv::EnvStateSrv::SharedPtr)> temp_env_state_srv_callback;
    rclcpp::Service<common::srv::EnvStateSrv>::SharedPtr service;
    
};

#endif  // __ENV_STATE_H__