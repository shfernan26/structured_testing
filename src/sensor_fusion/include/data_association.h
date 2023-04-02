#ifndef __DATA_ASSOCIATION_H__
#define __DATA_ASSOCIATION_H__

#include "rclcpp/rclcpp.hpp"
#include "object_state_da.h"
#include "radar_obj.h"
#include "mobileye_obj.h"
#include <vector>

#include "common/srv/env_state_srv.hpp"
#include "common/msg/associated_object_msg.hpp"
#include "common/msg/mobileye_object_data_msg.hpp"
#include "common/msg/radar_object_data_msg.hpp"

#define MOBILEYE_TOPIC "Mobileye_CAN_Rx"
#define RADAR_FRONT_TOPIC "Front_Radar_CAN_Rx"
#define CLUSTERED_OBJECT_TOPIC "associated_object"
#define SENSOR_DIAG_TOPIC "sensor_diagnostic_flags"
#define ENV_SERVICE "env_service_topic"
#define CH2_SERVICE "sensor_diagnostic_ch2"
#define CH4_SERVICE "sensor_diagnostic_ch4"

#define NEW_OBJ_DX_TOL 30.0
#define NEW_OBJ_DY_TOL 20.0
#define NEW_OBJ_VX_TOL 15.0

#define EXST_OBJ_DX_TOL 20.0
#define EXST_OBJ_DY_TOL 10.0
#define EXST_OBJ_VX_TOL 10.0

#define POTENTIAL_THRESHOLD 2
#define secondsToDelete 3
#define MESSAGE_BUFFER_SIZE 10
#define RADAR_OBJ 32
#define ME_OBJ 4  // how many objects ME is set to track
#define MIN_DX 1.0   
#define MAX_DX 130.00  
#define MAX_DY 25 // To account for detections in adjacent lanes   
#define MAX_DZ 10.00 // Avoiding conservative filtering
#define MIN_MOVING_VELOCITY 0.5
#define EXIST 0.95
#define SENSOR_TIMEOUT 0.05

/**
 * Class containing all relevant member functions and variables for Data Association node. 
 */
class DataAssociation {
	public:
        unsigned long me_count = 0;
        unsigned long ass_count = 0;
		DataAssociation(std::shared_ptr<rclcpp::Node> node);
		void delete_potential_objects(); /*!<  Used to delete potential objects (in temporary array) if havn't seen for a long time. */

		friend class ObjectState;

        /** \brief  Parent data association function for radar data .
        * 
        * This callback is called whenever the radar topics are published to. The order of functionality is as follows:
        * 1 - Filter detections using hardware based flags and reading thresholds
        * 2 - Loop through filtered objects and check if there is a match with the prexisting env state vector
        * 3 - If no match, check temp array
        * 4 - If still not a match, add to end of temp array (potential_objs)
        * 
        * Objects already in env state vector or those that appear consectively 'POTENTIAL_THRESHOLD' times are passed to the Kalman Filter.
        */
		void sensor_radar_data_obj_callback(const common::msg::RadarObjectDataMsg::SharedPtr sensor_data); 
		
        /** \brief  Parent data association function for mobileye data .
        * 
        * This callback is called whenever the mobileye topic is published to. The order of functionality is as follows:
        * 1 - Filter detections using hardware based flags and reading thresholds
        * 2 - Loop through filtered objects and check if there is a match with the prexisting env state vector
        * 3 - If no match, check temp array
        * 4 - If still not a match, add to end of temp array (potential_objs)
        * 
        * Objects already in env state vector or those that appear consectively 'POTENTIAL_THRESHOLD' times are passed to the Kalman Filter.
        */
        void sensor_me_data_obj_callback(const common::msg::MobileyeObjectDataMsg::SharedPtr sensor_data);
       
        // bool sensor_diagnostic_callback_ch2(const std::shared_ptr<common::srv::SensorDiagnosticFlagCh2::Request> req, std::shared_ptr<common::srv::SensorDiagnosticFlagCh2::Response> res);
        // bool sensor_diagnostic_callback_ch3(const std::shared_ptr<common::srv::SensorDiagnosticFlagCh3::Request> req, std::shared_ptr<common::srv::SensorDiagnosticFlagCh3::Response> res);
        // bool sensor_diagnostic_callback_ch4(const std::shared_ptr<common::srv::SensorDiagnosticFlagCh4::Request> req, std::shared_ptr<common::srv::SensorDiagnosticFlagCh4::Response> res);

        std::vector<ObjectState> potential_objs;

        std::vector<RadarObject> filter_radar(std::vector<RadarObject>& radar_char_data); /*!<  Check radar flags and filter data. */
        std::vector<MobileyeObject> filter_me(std::vector<MobileyeObject>& me_char_data); /*!<  Check mobileye flags and filter data. */

        std::vector<RadarObject> characterize_radar(const common::msg::RadarObjectDataMsg::SharedPtr recvd_data); /*!<  Characterize mobileye data. */
        std::vector<MobileyeObject> characterize_me(const common::msg::MobileyeObjectDataMsg::SharedPtr recvd_data); /*!<  Characterize mobileye data. */

        void pub_object(common::msg::AssociatedObjectMsg &associated_object_msg, ObjectState &obj_match); /*!<  Assign current radar values to associated_radar_msg topic. */

        // for unit testing
        void set_front_radar(bool fr){FRONT_RADAR = fr;}
        void set_right_corner_radar(bool rc){RIGHT_CORNER_RADAR = rc;}
        void set_left_corner_radar(bool lc){LEFT_CORNER_RADAR = lc;}
        void set_me(bool me){MOBILEYE = me;}
        
        std::vector<ObjectState> convert_radar_to_std_obj(std::vector<RadarObject>& radar_data); 
        std::vector<ObjectState> convert_me_to_std_obj(std::vector<MobileyeObject>& me_data);

        bool sensor_match(ObjectState obj1, ObjectState obj2); /*!<  Used to check if two arguments are same things based on DX_TOL, DY_TOL, and VX_TOL. */
        bool algo_match(ObjectState obj1, ObjectState obj2); 

        void match_radar_me();
        void match_dataAssoc_envState();

        std::vector<ObjectState> get_matched_std_objs();

        std::vector<ObjectState> std_me_objects;
        std::vector<ObjectState> std_radar_objects;
        std::vector<ObjectState> matched_std_objs;

        bool FRONT_RADAR;
        bool LEFT_CORNER_RADAR;
        bool RIGHT_CORNER_RADAR;
        bool MOBILEYE;

        double global_clk;
        unsigned long long next_id;
        rclcpp::Time last_me_callback; 
        rclcpp::Time last_fr_callback;
        rclcpp::Time curr_time;


	private:
		std::shared_ptr<rclcpp::Node> node;

		rclcpp::Client<common::srv::EnvStateSrv>::SharedPtr client;
        // std::function<void(const common::srv::SensorDiagnosticFlagCh2::SharedPtr)> temp_sensor_diagnostic_callback_ch2;
        // rclcpp::Service<common::srv::SensorDiagnosticFlagCh2>::SharedPtr srv_ch2;
        // rclcpp::Service<common::srv::SensorDiagnosticFlagCh3>::SharedPtr srv_ch3;
        // std::function<void(const common::srv::SensorDiagnosticFlagCh4::SharedPtr)> temp_sensor_diagnostic_callback_ch4;
        // rclcpp::Service<common::srv::SensorDiagnosticFlagCh4>::SharedPtr srv_ch4;

        rclcpp::Publisher<common::msg::AssociatedObjectMsg>::SharedPtr association_to_kf_pub;
        
        std::function<void(const common::msg::MobileyeObjectDataMsg::SharedPtr)> temp_sensor_me_data_obj_callback;
        rclcpp::Subscription<common::msg::RadarObjectDataMsg>::SharedPtr sensor_front_radar_data_obj_sub;
        std::function<void(const common::msg::RadarObjectDataMsg::SharedPtr)> temp_sensor_radar_data_obj_callback;
        rclcpp::Subscription<common::msg::MobileyeObjectDataMsg>::SharedPtr sensor_me_data_obj_sub;

        common::msg::AssociatedObjectMsg associated_object_msg;
        
};

#endif  // __DATA_ASSOCIATION_H__