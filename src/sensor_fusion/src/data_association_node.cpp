#include "data_association.h"
#include <algorithm>
#include <ctime>

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> data_association_node = rclcpp::Node::make_shared("data_association");
    DataAssociation data_assc = DataAssociation(data_association_node);

    while (rclcpp::ok()) {
        data_assc.match_radar_me();

        if (data_assc.get_matched_std_objs().size() > 0){
            data_assc.match_dataAssoc_envState();
        }

        rclcpp::spin_some(data_association_node);
        rclcpp::Rate(200).sleep();  // the sleep must be less than 5 ms
    }
    return 0;
}
