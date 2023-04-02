#ifndef __RADAR_OBJ_H__
#define __RADAR_OBJ_H__
#include <rclcpp/rclcpp.hpp>
#include "common/msg/radar_object_data_msg.hpp"

class RadarObject {
    public:
        RadarObject(){}

        RadarObject(double set_dx, double set_dy, double set_vx, double set_vy, double set_ax, double set_dx_s,
                    double set_dy_s, double set_vx_s, double set_ax_s, double set_w_exist, double set_w_obstacle,
                    bool set_flag_valid, double set_w_non_obs, bool set_flag_meas, bool set_flag_hist,
                    double set_length, double set_dz, double set_moving_state, double set_w_class, double set_obj_class,
                    double set_rear_loss, uint8_t set_radar_num, double set_timestamp, double set_veh_v_ego)
            : radar_dx(set_dx),

              radar_dy(set_dy),
              radar_vx(set_vx),
              radar_vy(set_vy),
              radar_ax(set_ax),
              radar_dx_sigma(set_dx_s),
              radar_dy_sigma(set_dy_s),
              radar_vx_sigma(set_vx_s),
              radar_ax_sigma(set_ax_s),
              radar_w_exist(set_w_exist),
              radar_w_obstacle(set_w_obstacle),
              radar_flag_valid(set_flag_valid),
              radar_w_non_obstacle(set_w_non_obs),
              flag_meas(set_flag_meas),
              flag_hist(set_flag_hist),
              d_length(set_length),
              radar_dz(set_dz),
              moving_state(set_moving_state),
              radar_w_class(set_w_class),
              radar_obj_class(set_obj_class),
              dx_rear_loss(set_rear_loss),
              radar_num(set_radar_num),
              radar_timestamp(set_timestamp),
              veh_v_ego(set_veh_v_ego)

        {}

        double radar_dx;
        double radar_dy;
        double radar_vx;
        double radar_vy;
        double radar_ax;
        double radar_dx_sigma;
        double radar_dy_sigma;
        double radar_vx_sigma;
        double radar_ax_sigma;
        double radar_w_exist;
        double radar_w_obstacle;
        bool radar_flag_valid;
        double radar_w_non_obstacle;
        bool flag_meas;
        bool flag_hist;
        double d_length;
        double radar_dz;
        double moving_state;
        double radar_w_class;
        double radar_obj_class;
        double dx_rear_loss;

        uint8_t radar_num;
        double radar_timestamp;
        double veh_v_ego;
};

#endif  // __RADAR_OBJ_H__
