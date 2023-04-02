#ifndef __OJBECT_STATE_H__
#define __OBJECT_STATE_H__
#include "rclcpp/rclcpp.hpp"
#include "common/msg/filtered_object_msg.hpp"

class ObjectState {
    public:
        ObjectState();

        ObjectState(uint64_t set_obj_id, double set_obj_dx, uint8_t set_obj_lane, double set_obj_vx, 
            double set_obj_dy, double set_obj_ax, bool set_obj_path, double set_obj_vy, double set_obj_timestamp, 
            uint8_t set_obj_track_num); //, uint8_t set_obj_count) {
        
        virtual ~ObjectState();
        
        uint64_t get_obj_id() const;
        double get_obj_dx() const;
        uint8_t get_obj_lane() const;
        double get_obj_vx() const;
        double get_obj_dy() const;
        double get_obj_ax() const;
        bool get_obj_path() const;
        double get_obj_vy() const;
        double get_obj_timestamp() const;
        //	uint8_t get_obj_count() const;

        void copy_info(const common::msg::FilteredObjectMsg::SharedPtr filtered_msg);
  
    private:
        uint64_t obj_id; // object ID
        double obj_dx; // longitudinal range
        uint8_t obj_lane; // lane assignment: 0-ego, 1-left, 2-right
        double obj_vx; // relaive longitudinal velocity
        double obj_dy; // lateral range
        double obj_ax; // relative longitudinal accel
        bool obj_path; // 1:object in vehicle path 2:object not in path
        double obj_vy; // lateral velocity
        double obj_timestamp; //time last object detection
        uint8_t obj_track_num; // for CAN DBC file
        //	uint8_t obj_count; // count used for obj association
};

#endif  // __OBJECT_STATE_H__