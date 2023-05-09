#include "env_state.h"
#include <cinttypes>

EnvironmentState::EnvironmentState(std::shared_ptr<rclcpp::Node> node) : node(node) {
  // temp_filtered_object_callback = std::bind(&EnvironmentState::filtered_object_callback, this, std::placeholders::_1);
  filtered_object_sub = node->create_subscription<common::msg::FilteredObjectMsg>(
    "filtered_obj", MESSAGE_BUFFER_SIZE, 
    std::bind(&EnvironmentState::filtered_object_callback, this, std::placeholders::_1)
  );

  tracked_obj_pub = node->create_publisher<common::msg::TrackedOutputMsg>("tracked_obj", MESSAGE_BUFFER_SIZE);
  all_tracked_obj_pub = node->create_publisher<common::msg::TargetOutputMsg>("all_tracked_obj", MESSAGE_BUFFER_SIZE);
  target_obj_pub = node->create_publisher<common::msg::TargetOutputMsg>("target_output", MESSAGE_BUFFER_SIZE);
  binary_class_pub = node->create_publisher<common::msg::BinaryClassMsg>("binary_class", MESSAGE_BUFFER_SIZE);
  
  // temp_env_state_srv_callback = std::bind(&EnvironmentState::env_state_srv_callback, this, std::placeholders::_1);
  service = node->create_service<common::srv::EnvStateSrv>(
    "env_service_topic", 
    std::bind(
      &EnvironmentState::env_state_srv_callback, 
      this, 
      std::placeholders::_1,              // Corresponds to the 'request' input
      std::placeholders::_2              // Corresponds to the 'response' output
    )
  );

  global_clk = 0;
  trackedObjects.reserve(MAX_OBJ); 	// reserve memory for vector (MAX_OBJ)
}

EnvironmentState::~EnvironmentState() {}

void EnvironmentState::publish_target_obj() { 

  target_output_msg.obj_id = targetObjectsInLanes[0].get_obj_id();
  target_output_msg.obj_dx = targetObjectsInLanes[0].get_obj_dx();
  target_output_msg.obj_lane = targetObjectsInLanes[0].get_obj_lane(); 
  target_output_msg.obj_vx = targetObjectsInLanes[0].get_obj_vx();
  target_output_msg.obj_dy = targetObjectsInLanes[0].get_obj_dy();
  target_output_msg.obj_ax = targetObjectsInLanes[0].get_obj_ax();
  target_output_msg.obj_path = targetObjectsInLanes[0].get_obj_path();
  target_output_msg.obj_vy = targetObjectsInLanes[0].get_obj_vy();
  target_output_msg.obj_timestamp = targetObjectsInLanes[0].get_obj_timestamp();
  target_output_msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(target_output_msg.obj_timestamp * 1e9));

    
  target_obj_pub->publish(target_output_msg);
}

void EnvironmentState::publish_tracked_obj() { // from left to right
  for (size_t lane = 0; lane < 3; lane++) {
      tracked_output_msg.obj_id[lane] = targetObjectsInLanes[lane].get_obj_id();
      tracked_output_msg.obj_dx[lane] = targetObjectsInLanes[lane].get_obj_dx();
      tracked_output_msg.obj_lane[lane] = targetObjectsInLanes[lane].get_obj_lane();  // 0 to 2
      tracked_output_msg.obj_vx[lane] = targetObjectsInLanes[lane].get_obj_vx();
      tracked_output_msg.obj_dy[lane] = targetObjectsInLanes[lane].get_obj_dy();
      tracked_output_msg.obj_ax[lane] = targetObjectsInLanes[lane].get_obj_ax();
      tracked_output_msg.obj_path[lane] = targetObjectsInLanes[lane].get_obj_path();
      tracked_output_msg.obj_vy[lane] = targetObjectsInLanes[lane].get_obj_vy();
      tracked_output_msg.obj_timestamp[lane] = targetObjectsInLanes[lane].get_obj_timestamp();
      tracked_output_msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(tracked_output_msg.obj_timestamp[lane] * 1e9));
  }
  tracked_obj_pub->publish(tracked_output_msg);
}

void EnvironmentState::publish_all_tracked_obj() { 
  // If trackedObjects is empty or counter < limit, publish default
  // if (trackedObjects.size() >= 1 && counter <= COUNTER_LIM) {
  if (trackedObjects.size() >= 1) {
    // Publish all objects in trackedObjects
    for (size_t i = 0; i < trackedObjects.size(); i++) {
      all_tracked_output_msg.obj_id = trackedObjects[i].get_obj_id();
      all_tracked_output_msg.obj_dx = trackedObjects[i].get_obj_dx();
      all_tracked_output_msg.obj_lane = trackedObjects[i].get_obj_lane();  // 0 to 2
      all_tracked_output_msg.obj_vx = trackedObjects[i].get_obj_vx();
      all_tracked_output_msg.obj_dy = trackedObjects[i].get_obj_dy();
      all_tracked_output_msg.obj_ax = trackedObjects[i].get_obj_ax();
      all_tracked_output_msg.obj_path = trackedObjects[i].get_obj_path();
      all_tracked_output_msg.obj_vy = trackedObjects[i].get_obj_vy();
      all_tracked_output_msg.obj_timestamp = trackedObjects[i].get_obj_timestamp();
      all_tracked_output_msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(all_tracked_output_msg.obj_timestamp * 1e9));
      
      all_tracked_obj_pub->publish(all_tracked_output_msg);
    }
  }
}

void EnvironmentState::filtered_object_callback(const common::msg::FilteredObjectMsg::SharedPtr filtered_msg) {
  // counter = 0;
 
  ObjectState tracked_msg;
  tracked_msg.copy_info(filtered_msg); // copy constructor
  std::cout << " Called -- " << tracked_msg.get_obj_lane() << std::endl;
  
  update_env_state(tracked_msg); // update id of objects in state vector
  // last_msg_ros_timestamp = node->now(); // This is the ROS timestamp when last msg received from Kalman Filter 
  last_msg_timestamp = tracked_msg.get_obj_timestamp(); // This is the actual timestamp of the last msg received from KF
}

void EnvironmentState::publish_binary_class() {
  common::msg::BinaryClassMsg out;
  for (auto i : trackedObjects) {
    out.dx.push_back(i.get_obj_dx());
    out.dy.push_back(i.get_obj_dy());
    out.vx.push_back(i.get_obj_vx());
    out.vy.push_back(i.get_obj_vy());
    out.timestamp.push_back(i.get_obj_timestamp());
  }
  out.global_clk = global_clk;
  binary_class_pub->publish(out);
}

common::msg::TargetOutputMsg EnvironmentState::get_target_output_msg() { return target_output_msg; }

common::msg::TrackedOutputMsg EnvironmentState::get_tracked_output_msg() { return tracked_output_msg; }

void EnvironmentState::add_object(const ObjectState& tracked_msg) {
  trackedObjects.push_back(tracked_msg); 
}

void EnvironmentState::update_object(const ObjectState& tracked_msg, size_t index) {
  trackedObjects[index] = tracked_msg; 
}

void EnvironmentState::check_tracked_time() {
  for (size_t index = 0; index < trackedObjects.size(); index++) {
    // printf("Tracked obj id: %lu, time difference: %f\r\n", trackedObjects[index].get_obj_id(), last_msg_timestamp - trackedObjects[index].get_obj_timestamp());
    if ((last_msg_timestamp - trackedObjects[index].get_obj_timestamp()) > ERASE_TOL) {  // more recent timestamps are larger
      trackedObjects.erase(trackedObjects.begin() + static_cast<std::vector<ObjectState>::difference_type>(index));
      index--;
      printf("Erased object from trackedObjects\r\n");
    }
  }
}

void EnvironmentState::update_env_state(const ObjectState& tracked_msg) {
  bool found = 0;
  size_t index_found = 0;
  printf("New msg timesatmp: %f\r\n", tracked_msg.get_obj_timestamp());
  

  for (size_t index = 0; index < trackedObjects.size(); index++) {
    // if object is found in the state vector (has been tracked), update it's ID
    // printf("Tracked id: %f\r\n", tracked_msg.get_obj_id());
    // printf("Untracked id: %f\r\n", trackedObjects[index].get_obj_id());
    if (tracked_msg.get_obj_id() == trackedObjects[index].get_obj_id()) {
      std::cout << "matched id ---- " <<  tracked_msg.get_obj_id() << std:: endl;
      found = true;
      index_found = index;
    }
  }

  // if object has been tracked, update the object in the state vector
  if (found == true) {
    update_object(tracked_msg, index_found);
  }
  // if object has not been tracked, add the object to state vector
  else {
    add_object(tracked_msg);
  }

}

// void EnvironmentState::find_target_object(const ObjectState& tracked_msg){
void EnvironmentState::find_target_object(){
  ObjectState empty_obj(0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

  for (size_t i = 0; i < trackedObjects.size(); i++) {
    int tracked_lane = trackedObjects[i].get_obj_lane();
    // 1 = center lane, 2 = left lane, 3 = right lane
    // update target object if a new object is closer than current target in the same lane, or if the current target
    // moves or if lane is empty (i.e. dx = 0)

    if (tracked_lane == 1) {
      last_msg_ros_timestamp_centre =  node->now();

      if ((trackedObjects[i].get_obj_dx() <= targetObjectsInLanes[0].get_obj_dx()) ||
          (trackedObjects[i].get_obj_id() == targetObjectsInLanes[0].get_obj_id()) ||
          (targetObjectsInLanes[0].get_obj_dx() == 0)) {
        targetObjectsInLanes[0] = trackedObjects[i];

      } 
    } else if (tracked_lane == 2) {
      last_msg_ros_timestamp_left =  node->now();

      if ((trackedObjects[i].get_obj_dx() <= targetObjectsInLanes[1].get_obj_dx()) ||
          (trackedObjects[i].get_obj_id() == targetObjectsInLanes[1].get_obj_id()) ||
          (targetObjectsInLanes[1].get_obj_dx() == 0)) {
        targetObjectsInLanes[1] = trackedObjects[i];

        } 
    } else if (tracked_lane == 3) {
      last_msg_ros_timestamp_right =  node->now();

      if ((trackedObjects[i].get_obj_dx() <= targetObjectsInLanes[2].get_obj_dx()) ||
          (trackedObjects[i].get_obj_id() == targetObjectsInLanes[2].get_obj_id()) ||
          (targetObjectsInLanes[2].get_obj_dx() == 0)) {
        targetObjectsInLanes[2] = trackedObjects[i];

        } 
    }
  }

}

void EnvironmentState::env_state_srv_callback(const std::shared_ptr<common::srv::EnvStateSrv::Request>, 
                                              std::shared_ptr<common::srv::EnvStateSrv::Response> res){
  // breaking the objects stored in the vector into members and storing in multiple vectors for srv communication
  // index refers to specific objectNum in the original vector
  for (size_t i = 0; i < trackedObjects.size(); i++) {
    res->id.push_back(trackedObjects[i].get_obj_id());
    std::cout << "here id " << trackedObjects[i].get_obj_id();
    res->dx.push_back(trackedObjects[i].get_obj_dx());
    res->dy.push_back(trackedObjects[i].get_obj_dy());
    res->vx.push_back(trackedObjects[i].get_obj_vx());
    res->vy.push_back(trackedObjects[i].get_obj_vy());
    
    res->timestamp.push_back(trackedObjects[i].get_obj_timestamp());
    // printf("returned %f, %f for %lu\n", res.dx[i], res.dy[i], res.id[i]);
    // res.count.push_back(trackedObjects[i].get_obj_count());
  }
  // return true;
}
