#include <gtest/gtest.h>
#include "data_association.h"

TEST(SensorMatch, TestBounds){
    std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("SensorMatch_TestBounds");
    DataAssociation data_association_test(nh);

    ObjectState valid_object(4,5,3,6,10); // dx vx dy vy time
    ObjectState invalid_object(100,100,100,100,100);

    ObjectState new_radar_obj(3,4,4,4,1);

    ASSERT_EQ(data_association_test.sensor_match(valid_object, new_radar_obj), 1);
    ASSERT_EQ(data_association_test.sensor_match(invalid_object, new_radar_obj), 0);
}

/**
 * ALL TESTS BELOW MUST BE CONVERTED FROM ROS TO ROS2
 */

// TEST(ObjectsMatchMobileye, validLogic){
//     ros::NodeHandle nh;
//     DataAssociation data_association_test(&nh);

//     ObjectState new_env_state_obj_1(4,5,3,6,10);
//     ObjectState new_env_state_obj_2(100,100,100,100,100);

//     MobileyeObject new_me_obj(1,4,4,4,1,1,1,1,1,1,1,1,1); //id dx dy vx ax 

//     ASSERT_EQ(data_association_test.objects_match_me(new_env_state_obj_1, new_me_obj), 1);
//     ASSERT_EQ(data_association_test.objects_match_me(new_env_state_obj_2, new_me_obj), 0);
// }

// TEST(FilterRadar, validLogic){
//     ros::NodeHandle nh;
//     DataAssociation data_association_test(&nh);

//     data_association_test.set_front_radar(1);
//     data_association_test.set_right_corner_radar(1);
//     data_association_test.set_left_corner_radar(1);

//     ASSERT_EQ(data_association_test.FRONT_RADAR, 1);
//     ASSERT_EQ(data_association_test.LEFT_CORNER_RADAR, 1);
//     ASSERT_EQ(data_association_test.RIGHT_CORNER_RADAR, 1);

//     common::msg::radar_object_data recvd_data;
    
//     for(int i = 0; i < 32; i++){
//         recvd_data.radar_dx[i] = i;
//         recvd_data.radar_dy[i] = i;
//         recvd_data.radar_vx[i] = i;
//         recvd_data.radar_vy[i] = i;
//         recvd_data.radar_ax[i] = i;
//         recvd_data.radar_dx_sigma[i] = i;
//         recvd_data.radar_dy_sigma[i] = i;
//         recvd_data.radar_vx_sigma[i] = i;
//         recvd_data.radar_ax_sigma[i] = i;
//         recvd_data.radar_w_exist[i] = i;
//         recvd_data.radar_w_obstacle[i] = i;
//         recvd_data.radar_flag_valid[i] = 1;
//         recvd_data.radar_w_non_obstacle[i] = i;
//         recvd_data.flag_meas[i] = 1;
//         recvd_data.flag_hist[i] = 1;
//         recvd_data.d_length[i] = i;
//         recvd_data.radar_dz[i] = i;
//         recvd_data.moving_state[i] = i;
//         recvd_data.radar_w_class[i] = i;
//         recvd_data.radar_obj_class[i] = i;
//         recvd_data.dx_rear_loss[i] = i;
//         recvd_data.radar_num = 1;
//         recvd_data.radar_timestamp = i;
//     }

//     data_association_test.filter_radar(recvd_data);
//     ASSERT_EQ(data_association_test.filtered_radar_obj[1].radar_dx, recvd_data.radar_dx[1]);
//     ASSERT_EQ(data_association_test.filtered_radar_obj[5].moving_state, recvd_data.moving_state[5]);
//     ASSERT_EQ(data_association_test.filtered_radar_obj[1].radar_timestamp, recvd_data.radar_timestamp);

// }

// TEST(FilterMobileye, validLogic){
//     ros::NodeHandle nh;
//     DataAssociation data_association_test(&nh);

//     data_association_test.set_me(1);
//     ASSERT_EQ(data_association_test.MOBILEYE, 1);

//     common::msg::MobileyeObjectData recvd_data;
    
//     for(int i = 0; i < 10; i++){
//         recvd_data.me_dx[i] = i;
//         recvd_data.me_dy[i] = i;
//         recvd_data.me_vx[i] = i;
//         recvd_data.me_ax[i] = i;
//         recvd_data.me_type[i] = i;
//         recvd_data.me_status[i] = i;
//         recvd_data.me_valid[i] = 1;
//         recvd_data.me_cut_in_cut_out[i] = i;
//         recvd_data.me_age[i] = i;
//         recvd_data.me_lane[i] = i;
//         recvd_data.me_cipv_flag[i] = 1;
//         recvd_data.me_timestamp = 1;
//     }

//     data_association_test.filter_me(recvd_data);
//     ASSERT_EQ(data_association_test.filtered_me_obj[1].me_dx, recvd_data.me_dx[1]);
//     ASSERT_EQ(data_association_test.filtered_me_obj[5].me_cipv_flag, recvd_data.me_cipv_flag[5]);
//     ASSERT_EQ(data_association_test.filtered_me_obj[1].me_timestamp, recvd_data.me_timestamp);

// }

// TEST(PubRadarSignals, validLogic){
//     ros::NodeHandle nh;
//     DataAssociation data_association_test(&nh);

//     common::msg::associated_radar_msg matched;
//         matched.radar_dx = 1;
//         matched.radar_dy = 1;
//         matched.radar_vx = 1;
//         matched.radar_vy = 1;
//         matched.radar_ax = 1;
//         matched.radar_dx_sigma = 1;
//         matched.radar_dy_sigma = 1;
//         matched.radar_vx_sigma = 1;
//         matched.radar_ax_sigma = 1;
//         matched.radar_timestamp = 1;
    
//     RadarObject radar_obj(1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1);

//     data_association_test.pub_radar_signals(matched, radar_obj);

//     ASSERT_EQ(matched.radar_timestamp, radar_obj.radar_timestamp);
//     ASSERT_EQ(matched.radar_dx_sigma, radar_obj.radar_dx_sigma);
//     ASSERT_EQ(matched.radar_ax, radar_obj.radar_ax);

// }

// TEST(PubMobileyeSignals, validLogic){
//     ros::NodeHandle nh;
//     DataAssociation data_association_test(&nh);

//     common::associated_me_msg matched;
//     matched.me_dx = 1;
//     matched.me_dy = 1;
//     matched.me_vx = 1;
//     matched.me_lane = 1;
//     matched.me_timestamp = 1;

//     MobileyeObject me_obj(1,1,1,1,1,1,1,1,1,1,1,1,1);

//     data_association_test.pub_me_signals(matched, me_obj);

//     ASSERT_EQ(matched.me_dy, me_obj.me_dy);
//     ASSERT_EQ(matched.me_timestamp, me_obj.me_timestamp);
//     ASSERT_EQ(matched.me_lane, me_obj.me_lane);

// }

// TEST(DeletePotentialObjects, validLogic){
//     ros::NodeHandle nh;
//     DataAssociation data_association_test(&nh);
    
//     ObjectState new_env_state_obj_1(4,5,3,6,10);
//     ObjectState new_env_state_obj_2(8,7,9,6,56);
//     ObjectState new_env_state_obj_3(2,2,3,6,3);
//     ObjectState new_env_state_obj_4(100,100,100,100,100);

//     data_association_test.global_clk = 1;
    
//     ASSERT_EQ(data_association_test.potential_objs.size(),0);

//     data_association_test.potential_objs.push_back(new_env_state_obj_1);
//     data_association_test.potential_objs.push_back(new_env_state_obj_2);
//     data_association_test.potential_objs.push_back(new_env_state_obj_3);
//     data_association_test.potential_objs.push_back(new_env_state_obj_4);

//     ASSERT_EQ(data_association_test.potential_objs.size(),4);

//     data_association_test.delete_potential_objects();
//     ASSERT_EQ(data_association_test.potential_objs.size(),1);

// }

// TEST(RadarObjCallback, validLogic){
//     ros::NodeHandle nh;
//     DataAssociation data_association_test(&nh);

//     data_association_test.set_front_radar(1);
//     data_association_test.set_right_corner_radar(1);
//     data_association_test.set_left_corner_radar(1);

    
//     common::msg::radar_object_data expected_data;

//     for(int i = 0; i < 32; i++){
//         expected_data.radar_dx[i] = i;
//         expected_data.radar_dy[i] = i;
//         expected_data.radar_vx[i] = i;
//         expected_data.radar_vy[i] = i;
//         expected_data.radar_ax[i] = i;
//         expected_data.radar_dx_sigma[i] = i;
//         expected_data.radar_dy_sigma[i] = i;
//         expected_data.radar_vx_sigma[i] = i;
//         expected_data.radar_ax_sigma[i] = i;
//         expected_data.radar_timestamp = i;
//     }

//     // case where directly go to potential objects
//     ASSERT_EQ(data_association_test.potential_objs.size(), 0);
    
//     data_association_test.sensor_radar_data_obj_callback(expected_data);
//     common::msg::associated_radar_msg received_data = data_association_test.get_associated_radar_msg();

//     ASSERT_EQ(expected_data.radar_dx[1], received_data.radar_dx);
//     ASSERT_EQ(expected_data.radar_dy[1], received_data.radar_dy);
//     ASSERT_EQ(expected_data.radar_vx[1], received_data.radar_vx);
//     ASSERT_EQ(data_association_test.potential_objs.size(), 32);
//     ASSERT_EQ(data_association_test.next_id, 0);

//     // diagnostics
//     data_association_test.set_left_corner_radar(0);
//     data_association_test.sensor_radar_data_obj_callback(expected_data);
//     ASSERT_EQ(data_association_test.potential_objs.size(), 0);

// }

// TEST(RadarObjCallback, validCallback){
//     ros::NodeHandle nh;
//     DataAssociation data_association_test(&nh);

//     ros::NodeHandle nh2;
//     rclcpp:Publisher pub = nh2.advertise<common::msg::radar_object_data>(RADAR_TOPIC, MESSAGE_BUFFER_SIZE);

//     common::msg::radar_object_data expected_data;

//     for(int i = 0; i < 32; i++){
//         expected_data.radar_dx[i] = i;
//         expected_data.radar_dy[i] = i;
//         expected_data.radar_vx[i] = i;
//         expected_data.radar_vy[i] = i;
//         expected_data.radar_ax[i] = i;
//         expected_data.radar_dx_sigma[i] = i;
//         expected_data.radar_dy_sigma[i] = i;
//         expected_data.radar_vx_sigma[i] = i;
//         expected_data.radar_ax_sigma[i] = i;
//         expected_data.radar_timestamp = i;
//     }

//     pub->publish(expected_data);
//     ros::spinOnce();

//     common::msg::associated_radar_msg rec_data = data_association_test.get_associated_radar_msg();
   
//     ASSERT_EQ(expected_data.radar_dx[1], rec_data.radar_dx);
//     ASSERT_EQ(expected_data.radar_dy[1], rec_data.radar_dy);
//     ASSERT_EQ(expected_data.radar_vx[1], rec_data.radar_vx);
//     ASSERT_EQ(data_association_test.potential_objs.size(), 32);
//     ASSERT_EQ(data_association_test.next_id, 0);

// }

// TEST(MobileyeObjCallback, validLogic){
//     ros::NodeHandle nh;
//     DataAssociation data_association_test(&nh);

//     data_association_test.set_me(1);
    
//     common::msg::MobileyeObjectData expected_data;

//     for(int i = 0; i < 32; i++){
//         expected_data.me_dx[i] = i;
//         expected_data.me_dy[i] = i;
//         expected_data.me_vx[i] = i;
//         expected_data.me_lane[i] = i;
//         expected_data.me_timestamp = 1;
//     }

//     // case where directly go to potential objects
//     ASSERT_EQ(data_association_test.potential_objs.size(), 0);
    
//     data_association_test.sensor_me_data_obj_callback(expected_data);
//     common::associated_me_msg received_data = data_association_test.get_associated_me_msg();

//     ASSERT_EQ(expected_data.me_dx[1], received_data.me_dx);
//     ASSERT_EQ(expected_data.me_dy[1], received_data.me_dy);
//     ASSERT_EQ(expected_data.me_vx[1], received_data.me_vx);
//     ASSERT_EQ(expected_data.me_lane[1], received_data.me_lane);
//     ASSERT_EQ(expected_data.me_timestamp, received_data.me_timestamp);
//     ASSERT_EQ(data_association_test.potential_objs.size(), 10);
//     ASSERT_EQ(data_association_test.next_id, 0);

//     // diagnostics
//     data_association_test.set_me(0);
//     data_association_test.sensor_me_data_obj_callback(expected_data);
//     ASSERT_EQ(data_association_test.potential_objs.size(), 0);

// }

// TEST(MobileyeObjCallback, validCallback){
//     ros::NodeHandle nh;
//     DataAssociation data_association_test(&nh);

//     ros::NodeHandle nh2;
//     rclcpp:Publisher pub = nh2.advertise<common::msg::MobileyeObjectData>(MOBILEYE_TOPIC, MESSAGE_BUFFER_SIZE);

//     common::msg::MobileyeObjectData expected_data;

//     for(int i = 0; i < 32; i++){
//         expected_data.me_dx[i] = i;
//         expected_data.me_dy[i] = i;
//         expected_data.me_vx[i] = i;
//         expected_data.me_lane[i] = i;
//         expected_data.me_timestamp = 1;
//     }

//     pub->publish(expected_data);
//     ros::spinOnce();

//     common::associated_me_msg received_data = data_association_test.get_associated_me_msg();

//     ASSERT_EQ(expected_data.me_dx[1], received_data.me_dx);
//     ASSERT_EQ(expected_data.me_dy[1], received_data.me_dy);
//     ASSERT_EQ(expected_data.me_vx[1], received_data.me_vx);
//     ASSERT_EQ(expected_data.me_lane[1], received_data.me_lane);
//     ASSERT_EQ(expected_data.me_timestamp, received_data.me_timestamp);
//     ASSERT_EQ(data_association_test.potential_objs.size(), 10);
//     ASSERT_EQ(data_association_test.next_id, 0);
// }

// TEST(FrontRadarDiagCallback, validLogic){
//     ros::NodeHandle nh;
//     DataAssociation data_association_test(&nh);

//     common::msg::associated_radar_msg assoc_radar_exp;
//     assoc_radar_exp.radar_dx = 1;
//     assoc_radar_exp.radar_dy = 2;
//     assoc_radar_exp.radar_vx = 3;

//     common::sensor_diagnostic_flag_CH2 ch2_diag;
//     ch2_diag.request.front_radar = 1;
    
//     data_association_test.sensor_diagnostic_callback_CH2(ch2_diag.request, ch2_diag.response);

//     ASSERT_EQ(ch2_diag.request.front_radar, data_association_test.FRONT_RADAR);
// }

// TEST(CornerRadarDiagCallback, validLogic){
//     ros::NodeHandle nh;
//     DataAssociation data_association_test(&nh);

//     common::msg::associated_radar_msg assoc_radar_exp;
//     assoc_radar_exp.radar_dx = 1;
//     assoc_radar_exp.radar_dy = 2;
//     assoc_radar_exp.radar_vx = 3;

//     common::sensor_diagnostic_flag_CH3 ch3_diag;
//     ch3_diag.request.left_corner_radar = 1;
//     ch3_diag.request.right_corner_radar = 1;

//     data_association_test.sensor_diagnostic_callback_CH3(ch3_diag.request, ch3_diag.response);

//     ASSERT_EQ(ch3_diag.request.left_corner_radar, data_association_test.LEFT_CORNER_RADAR);
//     ASSERT_EQ(ch3_diag.request.right_corner_radar, data_association_test.RIGHT_CORNER_RADAR);

// }


// TEST(MobileyeDiagCalback, validLogic){
//     ros::NodeHandle nh;
//     DataAssociation data_association_test(&nh);

//     common::associated_me_msg assoc_me_exp;
//     assoc_me_exp.me_dx = 1;
//     assoc_me_exp.me_dy = 2;
//     assoc_me_exp.me_vx = 3;

//     common::srv::SensorDiagnosticFlagCh4 ch4_diag;
//     ch4_diag.request.mobileye = 1;

//     data_association_test.sensor_diagnostic_callback_CH4(ch4_diag.request, ch4_diag.response);

//     ASSERT_EQ(ch4_diag.request.mobileye, data_association_test.MOBILEYE);
// }


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    // std::shared_ptr<rclcpp::Node> test_data_association = rclcpp::Node::make_shared("test_data_association");
    
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    rclcpp::shutdown();
}