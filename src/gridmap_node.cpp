#include "f110_occgrid/gridmap.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "gridmap");
    ros::NodeHandle nh;
    // class init here
    Gridmap grid(nh);
    ros::spin();
    return 0;
}
