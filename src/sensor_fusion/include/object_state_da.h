#ifndef __OJBECT_STATE_H__
#define __OBJECT_STATE_H__

class ObjectState {
    public:

    ObjectState(){}

    ObjectState(double in_dx, double in_vx, double in_dy, double in_vy, double in_timestamp)
        : dx(in_dx),
          vx(in_vx),
          dy(in_dy),
          vy(in_vy),
          timestamp(in_timestamp)

    // ADD MORE MEMBERS
    // THIS IS THE SRV FILE :
    // uint8[] id
    // float64[] dx
    // uint8[] lane
    // float64[] vx
    // float64[] dy
    // float64[] ax
    // bool[] path
    // float64[] vy
    // float64[] timestamp
    // uint8[] count

    {
      count = 0;
    }

    ObjectState(double in_x, double in_y) : dx(in_x), dy(in_y) { count = 0; }

    ObjectState(uint64_t in_id, double in_dx, double in_dy, double in_vx, double in_vy, double in_time)
        : id(in_id), dx(in_dx), dy(in_dy), vx(in_vx), vy(in_vy), timestamp(in_time) {}

    friend class DataAssociation;

    // virtual ~ObjectState();

    uint64_t id; // object ID
    double dx; // longitudinal range
    uint8_t lane; // lane assignment
    double vx; // relaive longitudinal velocity
    double dy; // lateral range
    double ax; // relative longitudinal accel
    bool path; // 1:object in path 2:object not in path
    double vy; // lateral velocity
    double timestamp; //time last object detection
    int count;
    int source; // me = 1, radar = 2

};
#endif  // __OBJECT_STATE_H__