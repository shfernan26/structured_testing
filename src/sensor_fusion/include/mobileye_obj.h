#ifndef __MOBILEYE_OBJECT_H__
#define __MOBILEYE_OBJECT_H__

class MobileyeObject {
    public:
        MobileyeObject(){}   
        MobileyeObject(uint8_t set_me_id, double set_me_dx, double set_me_dy, 
        double set_me_vx, double set_me_ax, uint8_t set_me_type, uint8_t set_me_status, uint8_t set_me_valid,
        uint8_t set_me_cut, double set_me_age, uint8_t set_me_lane, bool set_me_cipv,
        double set_me_timestamp)
            :me_id(set_me_id),
            me_dx(set_me_dx),
            me_dy(set_me_dy),
            me_vx(set_me_vx),
            me_ax(set_me_ax),
            me_type(set_me_type),
            me_status(set_me_status),
            me_valid(set_me_valid),
            me_cut_in_cut_out(set_me_cut),
            me_age(set_me_age),
            me_lane(set_me_lane),
            me_cipv_flag(set_me_cipv),

            me_timestamp(set_me_timestamp)
         
        {}

        uint8_t me_id;
        double me_dx;
        double me_dy;
        double me_vx;
        double me_ax;
        uint8_t me_type;
        uint8_t me_status;
        uint8_t me_valid;
        uint8_t me_cut_in_cut_out;
        double me_age;
        uint8_t me_lane;
        bool me_cipv_flag;

        double me_timestamp;
    
};

#endif  // __MOBILEYE_OBJECT_H__
