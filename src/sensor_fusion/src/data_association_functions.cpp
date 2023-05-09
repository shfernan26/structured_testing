#include "data_association.h"
#include <algorithm>
#include <ctime>


DataAssociation::DataAssociation(std::shared_ptr<rclcpp::Node> node) : node(node) {
    client = node->create_client<common::srv::EnvStateSrv>(ENV_SERVICE);

    // temp_sensor_diagnostic_callback_ch2 = std::bind(&DataAssociation::sensor_diagnostic_callback_ch2, this, std::placeholders::_1);
    // srv_ch2 = node->create_service<common::srv::SensorDiagnosticFlagCh2>(CH2_SERVICE, &DataAssociation::sensor_diagnostic_callback_ch2);   
    // temp_sensor_diagnostic_callback_ch4 = std::bind(&DataAssociation::sensor_diagnostic_callback_ch4, this, std::placeholders::_1);
    // srv_ch4 = node->create_service<common::srv::SensorDiagnosticFlagCh4>(CH4_SERVICE, &DataAssociation::sensor_diagnostic_callback_ch4);     

    // temp_sensor_me_data_obj_callback = std::bind(&DataAssociation::sensor_me_data_obj_callback, this, std::placeholders::_1);
    sensor_me_data_obj_sub = node->create_subscription<common::msg::MobileyeObjectDataMsg>(
        MOBILEYE_TOPIC, MESSAGE_BUFFER_SIZE, 
        std::bind(&DataAssociation::sensor_me_data_obj_callback, this, std::placeholders::_1)
    );

    // temp_sensor_radar_data_obj_callback = std::bind(&DataAssociation::sensor_radar_data_obj_callback, this, std::placeholders::_1);
    sensor_front_radar_data_obj_sub = node->create_subscription<common::msg::RadarObjectDataMsg>(
        RADAR_FRONT_TOPIC, MESSAGE_BUFFER_SIZE,
        std::bind(&DataAssociation::sensor_radar_data_obj_callback, this, std::placeholders::_1)
    );

    association_to_kf_pub = node->create_publisher<common::msg::AssociatedObjectMsg>(CLUSTERED_OBJECT_TOPIC, 10);

    next_id = 1;
    global_clk = 0;

    // diagnostics (assume true, service call if false)
    FRONT_RADAR = 1;
    MOBILEYE = 1;
}

std::vector<RadarObject> DataAssociation::filter_radar(std::vector<RadarObject>& radar_char_data){

    std::vector<RadarObject> filtered_radar_obj;

    // size_t filtered_index = 0;
    for (size_t r_index = 0; r_index < radar_char_data.size(); r_index++){
        // dx and dy limits
        if (radar_char_data[r_index].radar_dx < MIN_DX || radar_char_data[r_index].radar_dx > MAX_DX
            || abs(radar_char_data[r_index].radar_dy) > MAX_DY) continue;

        // Stationary objects
        if (((radar_char_data[r_index].veh_v_ego + abs(radar_char_data[r_index].radar_vx)) < MIN_MOVING_VELOCITY)) continue;


        // Exist probability flag - needs more testing to confirm threshold
        if (radar_char_data[r_index].radar_w_exist < EXIST) continue;

        // Valid flag - 1 is valid
        if (radar_char_data[r_index].radar_flag_valid == 0) continue;

        // Dz should be in range (needs calibration likely)
        if (radar_char_data[r_index].radar_dz > MAX_DZ || radar_char_data[r_index].radar_dz < -MAX_DZ) continue;

        // printf("Success filtering radar data: %f, %f, %f, %f", recvd_data.radar_dx[r_index], recvd_data.radar_dy[r_index], recvd_data.radar_vx[r_index], recvd_data.radar_vy[r_index]);
        // ROS_INFO_STREAM("Object passed radar filters");

        RadarObject filtered_radar_temp;

        filtered_radar_temp.radar_dx = radar_char_data[r_index].radar_dx;
        filtered_radar_temp.radar_dy = radar_char_data[r_index].radar_dy;
        filtered_radar_temp.radar_vx = radar_char_data[r_index].radar_vx;
        filtered_radar_temp.radar_ax = radar_char_data[r_index].radar_ax;
        filtered_radar_temp.radar_dx_sigma = radar_char_data[r_index].radar_dx_sigma;
        filtered_radar_temp.radar_dy_sigma = radar_char_data[r_index].radar_dy_sigma;
        filtered_radar_temp.radar_vx_sigma = radar_char_data[r_index].radar_vx_sigma;
        filtered_radar_temp.radar_ax_sigma = radar_char_data[r_index].radar_ax_sigma;
        filtered_radar_temp.radar_w_exist = radar_char_data[r_index].radar_w_exist;
        filtered_radar_temp.radar_w_obstacle = radar_char_data[r_index].radar_w_obstacle;
        filtered_radar_temp.radar_flag_valid = radar_char_data[r_index].radar_flag_valid; 
        filtered_radar_temp.radar_w_non_obstacle = radar_char_data[r_index].radar_w_non_obstacle;
        filtered_radar_temp.flag_meas = radar_char_data[r_index].flag_meas; 
        filtered_radar_temp.flag_hist = radar_char_data[r_index].flag_hist; 
        filtered_radar_temp.d_length = radar_char_data[r_index].d_length; 
        filtered_radar_temp.radar_dz = radar_char_data[r_index].radar_dz; 
        filtered_radar_temp.moving_state = radar_char_data[r_index].moving_state; 
        filtered_radar_temp.radar_w_class = radar_char_data[r_index].radar_w_class; 
        filtered_radar_temp.radar_obj_class = radar_char_data[r_index].radar_obj_class; 
        filtered_radar_temp.dx_rear_loss = radar_char_data[r_index].dx_rear_loss; 
        filtered_radar_temp.radar_num = radar_char_data[r_index].radar_num; 
        filtered_radar_temp.radar_timestamp = radar_char_data[r_index].radar_timestamp; 
        filtered_radar_temp.radar_vy = radar_char_data[r_index].radar_vy;

        filtered_radar_obj.push_back(filtered_radar_temp);
    }
    return filtered_radar_obj;
}


std::vector<MobileyeObject> DataAssociation::filter_me(std::vector<MobileyeObject>& me_char_data){

    std::vector<MobileyeObject> filtered_me_obj;
    double old_timestamp = 0;

    for (size_t f_index = 0; f_index < me_char_data.size(); f_index++){
        // dx and dy threshold
        if (me_char_data[f_index].me_dx < MIN_DX || me_char_data[f_index].me_dx > MAX_DX ||
            abs(me_char_data[f_index].me_dy) > MAX_DY){
                continue;               
            }
          
        if (me_char_data[f_index].me_timestamp == old_timestamp){
            continue;  
        }  

        std::cout << "Type " << me_char_data[f_index].me_type << std::endl;
        if (me_char_data[f_index].me_type != 0){
            continue;
        }      

        MobileyeObject filtered_me_temp;

        filtered_me_temp.me_dx = me_char_data[f_index].me_dx;
        filtered_me_temp.me_dy = me_char_data[f_index].me_dy;
        filtered_me_temp.me_vx = me_char_data[f_index].me_vx;
        filtered_me_temp.me_ax = me_char_data[f_index].me_ax;
        filtered_me_temp.me_type = me_char_data[f_index].me_type;
        filtered_me_temp.me_status = me_char_data[f_index].me_status;
        filtered_me_temp.me_valid = me_char_data[f_index].me_valid;
        filtered_me_temp.me_cut_in_cut_out = me_char_data[f_index].me_cut_in_cut_out;
        filtered_me_temp.me_age = me_char_data[f_index].me_age;
        filtered_me_temp.me_lane = me_char_data[f_index].me_lane;
        filtered_me_temp.me_cipv_flag = me_char_data[f_index].me_cipv_flag;
        filtered_me_temp.me_timestamp = me_char_data[f_index].me_timestamp;
    
        filtered_me_obj.push_back(filtered_me_temp);
    }



    if (me_char_data.size() > 0){
        old_timestamp = me_char_data[0].me_timestamp;
    }
        
    return filtered_me_obj;
}

void DataAssociation::pub_object(common::msg::AssociatedObjectMsg &associated_object_msg, ObjectState &obj_match){
    // std::cout << "Pub object "<< obj_match.vx << std::endl;
    associated_object_msg.obj_id = obj_match.id;
	associated_object_msg.obj_dx = obj_match.dx;
    associated_object_msg.obj_dy = obj_match.dy;
    associated_object_msg.obj_vx = obj_match.vx;
    associated_object_msg.obj_vy = obj_match.vy;
    associated_object_msg.obj_ax = obj_match.ax;
    associated_object_msg.obj_timestamp = obj_match.timestamp;
    associated_object_msg.obj_source = obj_match.source;
    associated_object_msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(associated_object_msg.obj_timestamp * 1e9));
}

void DataAssociation::delete_potential_objects() {
    for (unsigned i = 0; i < potential_objs.size(); i++){
      if ((global_clk - potential_objs[i].timestamp > secondsToDelete)) {
        // std::cout << "Potential object deleted because too old in temp array" << std::endl;
        // std::cout << "Global clock: " << +global_clk << std::endl;
        // std::cout << "Pot obj "<< +i << " time " << +potential_objs[i].timestamp << std::endl;
        potential_objs.erase(potential_objs.begin() + i);
        }
    }
}


bool DataAssociation::sensor_match(ObjectState obj1, ObjectState obj2) {
    // filter dx, dy, vx
    // printf("Object 1 with dx of %.2f, dy of %.2f, and vx of %.2f\n", obj1.dx, obj1.dy, obj1.vx);
    // printf("Object 2 with dx of %.2f, dy of %.2f, and vx of %.2f\n", obj2.dx, obj2.dy, obj2.vx);
    if ((abs(obj1.vx - obj2.vx) < NEW_OBJ_DX_TOL) && (abs(obj1.dy - obj2.dy) < NEW_OBJ_DY_TOL)) {
    //   printf("Object matched with dx of %.2f, dy of %.2f, and vx of %.2f\n",
    //   obj1.dx - obj2.dx, obj1.dy - obj2.dy, obj1.vx - obj2.vx);
      return 1;
    }
    return 0;
}

bool DataAssociation::algo_match(ObjectState obj1, ObjectState obj2) {
    // filter dx, dy, vx
    // printf("Algo obj 1 with dx of %.2f, dy of %.2f, and vx of %.2f\n", obj1.dx, obj1.dy, obj1.vx);
    // printf("Algo 2 with dx of %.2f, dy of %.2f, and vx of %.2f\n", obj2.dx, obj2.dy, obj2.vx);
    
    if ((abs(obj1.dx - obj2.dx) < EXST_OBJ_DX_TOL) && (abs(obj1.dy - obj2.dy) < EXST_OBJ_DY_TOL) && (abs(obj1.vx - obj2.vx) < EXST_OBJ_VX_TOL)) {
      return 1;
    }
    return 0;
}


std::vector<MobileyeObject> DataAssociation::characterize_me(const common::msg::MobileyeObjectDataMsg::SharedPtr recvd_data){

    std::vector<MobileyeObject> characterized_me_obj;
    
    for (size_t me_index = 0; me_index < ME_OBJ; me_index++){
        
        MobileyeObject characterized_me_temp;
        characterized_me_temp.me_dx = recvd_data->me_dx[me_index];
        // gt_dy = me_dy - p2 - p1*me_dx; from https://uwaterloo.atlassian.net/wiki/spaces/UWAFT/pages/42976871088/Mobileye+-+DY
        if (recvd_data->me_dx[me_index] < 10){
            characterized_me_temp.me_dy = - recvd_data->me_dy[me_index];
        }
        else{
            characterized_me_temp.me_dy = - recvd_data->me_dy[me_index] + 0.2623 + 0.0813*recvd_data->me_dx[me_index];
        }
        
        characterized_me_temp.me_vx = recvd_data->me_vx[me_index];
        characterized_me_temp.me_ax = recvd_data->me_ax[me_index];
        characterized_me_temp.me_type = recvd_data->me_type[me_index];
        characterized_me_temp.me_status = recvd_data->me_status[me_index];
        characterized_me_temp.me_valid = recvd_data->me_valid[me_index];
        characterized_me_temp.me_cut_in_cut_out = recvd_data->me_cut_in_cut_out[me_index];
        characterized_me_temp.me_age = recvd_data->me_age[me_index];
        characterized_me_temp.me_lane = recvd_data->me_lane[me_index];
        characterized_me_temp.me_cipv_flag = recvd_data->me_cipv_flag[me_index];
        characterized_me_temp.me_timestamp = recvd_data->me_timestamp;

        characterized_me_obj.push_back(characterized_me_temp);
    }

    return characterized_me_obj;
}

std::vector<RadarObject> DataAssociation::characterize_radar(const common::msg::RadarObjectDataMsg::SharedPtr recvd_data){

    std::vector<RadarObject> radar_me_obj;
    
    for (size_t radar_index = 0; radar_index < RADAR_OBJ; radar_index++){
        
        RadarObject characterized_radar_temp;
        // from https://uwaterloo.atlassian.net/wiki/spaces/UWAFT/pages/edit-v2/43015799321
        if (recvd_data->radar_dx[radar_index] < 5){
            characterized_radar_temp.radar_dx = recvd_data->radar_dx[radar_index];  //UNCOMMENT SURROUNDING BLOCK AFTER ROSBAG RECORD 
        }
        else{
            characterized_radar_temp.radar_dx = recvd_data->radar_dx[radar_index] - 1.84 + 0.056*recvd_data->radar_dx[radar_index];
        }

        characterized_radar_temp.radar_dy = recvd_data->radar_dy[radar_index];
        characterized_radar_temp.radar_vx = recvd_data->radar_vx[radar_index];
        characterized_radar_temp.radar_ax = recvd_data->radar_ax[radar_index];
        characterized_radar_temp.radar_dx_sigma = recvd_data->radar_dx_sigma[radar_index];
        characterized_radar_temp.radar_dy_sigma = recvd_data->radar_dy_sigma[radar_index];
        characterized_radar_temp.radar_vx_sigma = recvd_data->radar_vx_sigma[radar_index];
        characterized_radar_temp.radar_ax_sigma = recvd_data->radar_ax_sigma[radar_index];
        characterized_radar_temp.radar_w_exist = recvd_data->radar_w_exist[radar_index];
        characterized_radar_temp.radar_w_obstacle = recvd_data->radar_w_obstacle[radar_index];
        characterized_radar_temp.radar_flag_valid = recvd_data->radar_flag_valid[radar_index];
        characterized_radar_temp.radar_w_non_obstacle = recvd_data->radar_w_non_obstacle[radar_index];
        characterized_radar_temp.flag_meas = recvd_data->flag_meas[radar_index];
        characterized_radar_temp.flag_hist = recvd_data->flag_hist[radar_index];
        characterized_radar_temp.d_length = recvd_data->d_length[radar_index];
        characterized_radar_temp.radar_dz = recvd_data->radar_dz[radar_index];
        characterized_radar_temp.moving_state = recvd_data->moving_state[radar_index];
        characterized_radar_temp.radar_w_class = recvd_data->radar_w_class[radar_index];
        characterized_radar_temp.radar_obj_class = recvd_data->radar_obj_class[radar_index];
        characterized_radar_temp.dx_rear_loss = recvd_data->dx_rear_loss[radar_index];
        characterized_radar_temp.radar_num = recvd_data->radar_num;
        characterized_radar_temp.radar_timestamp = recvd_data->radar_timestamp;
        characterized_radar_temp.radar_vy = recvd_data->radar_vy[radar_index];
        characterized_radar_temp.veh_v_ego = recvd_data->veh_a_ego;
        

        radar_me_obj.push_back(characterized_radar_temp);
    }

    return radar_me_obj;
}

std::vector<ObjectState> DataAssociation::convert_radar_to_std_obj(std::vector<RadarObject>& radar_data){
    
    std::vector<ObjectState> radar_objs;
 
    for (size_t rad_idx = 0; rad_idx < radar_data.size(); rad_idx++){
        ObjectState current_obj;


        current_obj.dx = radar_data[rad_idx].radar_dx;
        current_obj.dy = radar_data[rad_idx].radar_dy;
        current_obj.vx = radar_data[rad_idx].radar_vx;
        current_obj.vy = radar_data[rad_idx].radar_dy;
        current_obj.ax = radar_data[rad_idx].radar_ax;
        current_obj.timestamp = radar_data[rad_idx].radar_timestamp;
        current_obj.source = 2; 
        radar_objs.push_back(current_obj);

    }

    return radar_objs;

}

std::vector<ObjectState> DataAssociation::convert_me_to_std_obj(std::vector<MobileyeObject>& me_data){
    
    std::vector<ObjectState> me_objs;
 
    for (size_t me_idx = 0; me_idx < me_data.size(); me_idx++){
        ObjectState current_obj;

        
        current_obj.dx = me_data[me_idx].me_dx;
        current_obj.dy = me_data[me_idx].me_dy;
        current_obj.vx = me_data[me_idx].me_vx;
        current_obj.vy = 0;
        current_obj.ax = me_data[me_idx].me_ax;
        current_obj.timestamp = me_data[me_idx].me_timestamp;
        current_obj.source = 1; 
        me_objs.push_back(current_obj);

    }

    return me_objs;

}

void DataAssociation::sensor_radar_data_obj_callback(const common::msg::RadarObjectDataMsg::SharedPtr recvd_data) {

    global_clk = recvd_data->radar_timestamp;
    last_fr_callback = node->now();
    // ROS_INFO_STREAM("Potential objs size: " << potential_objs.size());
    //std::cout << "Potential objs size: " << potential_objs.size() << std::endl;

    std::vector<RadarObject> characterized_radar_obj;
    std::vector<RadarObject> filtered_radar_obj;

    // filter detections
    if(FRONT_RADAR){ 
        
        characterized_radar_obj = characterize_radar(recvd_data);
        // std::cout << "char fr" << characterized_radar_obj.size() << std::endl;

        filtered_radar_obj = filter_radar(characterized_radar_obj);
        // std::cout << "filt fr objs" << filtered_radar_obj.size() << std::endl;

        std_radar_objects = convert_radar_to_std_obj(filtered_radar_obj);
        // std::cout << "std_fr_objects" << std_radar_objects.size() << std::endl;

    }
    else{
        // std::cout<< "Invalid radar service call" << std::endl;
    }
}

void DataAssociation::sensor_me_data_obj_callback(const common::msg::MobileyeObjectDataMsg::SharedPtr recvd_data) {

    global_clk = recvd_data->me_timestamp;
    last_me_callback = node->now();
    // std::cout << "Me count: " << me_count++ << std::endl;
    // std::cout << "Potential objs size: " << potential_objs.size() << std::endl;

    std::vector<MobileyeObject> filtered_me_obj;
    std::vector<MobileyeObject> characterized_me_obj;


    // filter detections
    if(MOBILEYE){

      // Characterization of mobileye inputs based on previous data analysis
        characterized_me_obj = characterize_me(recvd_data);
        // std::cout << "char me" << characterized_me_obj.size() << std::endl;

        filtered_me_obj = filter_me(characterized_me_obj);
        // std::cout << "filt objs" << filtered_me_obj.size() << std::endl;
        
        std_me_objects = convert_me_to_std_obj(filtered_me_obj);
        // std::cout << "std_me_objects" << std_me_objects.size() << std::endl;

    }
    else{
        // ROS_INFO_STREAM("Mobileye message invalid");
    }

}

void DataAssociation::match_radar_me(){
    ObjectState curr_obj;
    curr_time = node->now();

    // std::cout << "me objs" << std_me_objects.size() << std::endl;
    // std::cout << "fr objs" << std_radar_objects.size() << std::endl;

    if ((std_me_objects.size() > 0) && (std_radar_objects.size() > 0)) { // both me and radar data available
        // std::cout << "case a--" << std_me_objects.size() << " -- " << std_radar_objects.size() << std::endl;
        for (size_t a = 0; a < std_me_objects.size(); a++){
            curr_obj = std_me_objects[a]; // by default use ME object
            // std::cout << "Ass count: " << ass_count++ << std::endl;
            
            if (curr_obj.dx > 60){
                std::cout << "Over 60 ---"<< curr_obj.dx << std::endl;
                matched_std_objs.push_back(curr_obj); // WHAT IS THIS===================================
            }

            else{
                bool matched_me = 0;
                for (size_t b = 0; b < std_radar_objects.size(); b++){              
                    // std::cout << "me " << curr_obj.dx << "--" << curr_obj.dy << std::endl;
                    // std::cout << "fr " << std_radar_objects[b].dx << "--" << std_radar_objects[b].dy << std::endl;
                    if (sensor_match(curr_obj, std_radar_objects[b]) ){ // if sensor objects match fuse them
                        std::cout << "Issa match -- " << curr_obj.dx << " -&- "<< std_radar_objects[b].dx << std::endl;
                        curr_obj.dx = std_me_objects[a].dx;
                        curr_obj.dy = std_me_objects[a].dy;
                        curr_obj.vx = std_radar_objects[b].vx;
                        curr_obj.vy = std_radar_objects[b].vy; 
                        matched_std_objs.push_back(curr_obj);
                        matched_me = 1;
                    }
                    curr_obj = std_me_objects[a];
                }
                if (!matched_me) {
                    matched_std_objs.push_back(curr_obj);
                }
            } 
        }
        std_me_objects.clear();
        std_radar_objects.clear();
    }

    else if (std_me_objects.size() > 0) {
        // std::cout << "case b" << std_me_objects.size() << std::endl;
        for (size_t a = 0; a < std_me_objects.size(); a++){
            matched_std_objs.push_back(std_me_objects[a]); 

        }
        std_me_objects.clear();
    }

    else if ((std_radar_objects.size() > 0)  && (curr_time.seconds() - last_me_callback.seconds() > SENSOR_TIMEOUT)) {
        // std::cout << "case c" << std_radar_objects.size() << std::endl;
        for (size_t a = 0; a < std_radar_objects.size(); a++){
            curr_obj = std_radar_objects[a];

            for (size_t b = 0; b < std_radar_objects.size(); b++){
                if (  (!sensor_match(curr_obj, std_radar_objects[b])) || (matched_std_objs.size() == 0)  ){ // if sensor objects are unique pass them on to matched objs list
                    std::cout << "FR match -- " << curr_obj.dx << std::endl;
                    matched_std_objs.push_back(curr_obj);
                }
                
            }
        }

        std_radar_objects.clear();
    }

    else {
    }

}


void DataAssociation::match_dataAssoc_envState(){

    // Initalize service
    std::vector<ObjectState> envState;
    common::msg::AssociatedObjectMsg associated_object_msg;

    // Call service everytime new me callback message received
    auto request = std::make_shared<common::srv::EnvStateSrv::Request>();
    auto result = client->async_send_request(request);
    
    // Spin until future is blocking till environment state request completed
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS){
        // std::cout << "Service called successfully\n";
        for (size_t srv_index = 0; srv_index < result.get()->id.size(); srv_index++) {
          ObjectState someObj(result.get()->id[srv_index], result.get()->dx[srv_index], result.get()->dy[srv_index], result.get()->vx[srv_index], result.get()->vy[srv_index],
                              result.get()->timestamp[srv_index]);
          envState.push_back(someObj);
        }
    } 
    else {
        std::cout << ("Failed to call service, but continuing to the already stored potential objects, maybe it matches up there!?") << std::endl;
    }

    // std::cout << "Env size" << envState.size() << std::endl;
    // Loop through each object in the filtered list
    for (size_t sensor_index = 0; sensor_index < matched_std_objs.size(); sensor_index++){
        bool sensor_matched = 0;

        // create object
        ObjectState data_obj = matched_std_objs.at(sensor_index);
        
        // if the object we received is already in the envState, send it to kf
        for (auto env_obj : envState) {
            
            if (algo_match(env_obj, data_obj)) {
                // std::cout << "in env state" << std::endl;
                std::cout << "env state id \n" << env_obj.id;
                associated_object_msg.obj_id = env_obj.id;
                data_obj.id = next_id;
                pub_object(associated_object_msg, data_obj);
                association_to_kf_pub->publish(associated_object_msg);
                // std::cout << "Pub In env state\n";
                sensor_matched = 1;
                break;
            }
        }     

        // now check if it matches any of the potential objects
        if (!sensor_matched) {
            for (auto obj_iterator = potential_objs.begin(); obj_iterator != potential_objs.end(); obj_iterator++) {
                std::cout << "\n Check match-- " << std::endl;             
                if (algo_match(*obj_iterator, data_obj)) {
                    obj_iterator->timestamp = global_clk;
                    obj_iterator->dx = data_obj.dx;
                    obj_iterator->dy = data_obj.dy;
                    obj_iterator->vx = data_obj.vx;
                    obj_iterator->vx = data_obj.vy;
                    obj_iterator->count++;

                    // Once iterator > threshold, publish
                    if (obj_iterator->count > POTENTIAL_THRESHOLD) {
                        next_id++;
                        associated_object_msg.obj_id = next_id;
                        data_obj.id = next_id;
                        pub_object(associated_object_msg, data_obj);
                        association_to_kf_pub->publish(associated_object_msg);
                        potential_objs.erase(obj_iterator);
                        std::cout << "Pub in potentials " << next_id << std::endl;
                    }
                    sensor_matched = 1;
                    break;
                }
            }
            delete_potential_objects();
        }

        // Not match env state vector or temporary array
        if (!sensor_matched) {
            potential_objs.emplace_back(ObjectState(data_obj.dx, data_obj.vx, data_obj.dy, data_obj.vy, data_obj.timestamp));
        }            
    }
    matched_std_objs.clear();

}

// bool DataAssociation::sensor_diagnostic_callback_ch2(const std::shared_ptr<common::srv::SensorDiagnosticFlagCh2::Request> req, 
//                                                     std::shared_ptr<common::srv::SensorDiagnosticFlagCh2::Response> res) {
//     // service callback for front radar
//     FRONT_RADAR = req->front_radar;
//     return true;
// }

// bool DataAssociation::sensor_diagnostic_callback_ch4(const std::shared_ptr<common::srv::SensorDiagnosticFlagCh4::Request> req, 
//                                                     std::shared_ptr<common::srv::SensorDiagnosticFlagCh4::Response> res) {
//     // service callback for mobileye
//     MOBILEYE = req->mobileye;
//     return true;
// }

std::vector<ObjectState> DataAssociation::get_matched_std_objs(){

    return matched_std_objs;

}
