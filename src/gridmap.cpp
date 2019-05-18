#include "f110_occgrid/gridmap.h"

Gridmap::~Gridmap() {
    ROS_INFO("Gridmap node shutting down");
}
Gridmap::Gridmap(ros::NodeHandle &nh) : nh_(nh) {
    // publishers
    env_pub = nh_.advertise<nav_msgs::OccupancyGrid>("env_layer", 10);
    static_pub = nh_.advertise<nav_msgs::OccupancyGrid>("static_layer", 10);
    dynamic_pub = nh_.advertise<nav_msgs::OccupancyGrid>("dynamic_layer", 10);

    // subscribers
    scan_sub = nh_.subscribe("/scan", 10, &Gridmap::scan_callback, this);
    map_sub = nh_.subscribe("/map", 10, &Gridmap::map_callback, this);
}
void Gridmap::map_metadata_callback(const nav_msgs::MapMetaData::ConstPtr& metadata_msg){
    INIT = true;
    map_resolution = metadata_msg->resolution;
    map_width = metadata_msg->width;
    map_height = metadata_msg->height;
    map_origin = metadata_msg->origin;
    ROS_INFO("Map loaded.");
}
void Gridmap::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {
    if (!INIT) return;
    std::vector<int> map_data = map_msg->data;
    // reroute to env_layer
    env_pub.publish(map_msg);
    // save data to attribute
    
}
void Gridmap::scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    std::vector<float> ranges = scan_msg->ranges;
}
Eigen::MatrixXf Gridmap::get_env_layer() {
    return env_layer;
}
Eigen::MatrixXf Gridmap::get_static_layer() {
    return static_layer;
}
Eigen::MatrixXf Gridmap::get_dynamic_layer() {
    return dynamic_layer;
}
float Gridmap::get_value_at_position(int x, int y) {
    return 0;
}
float Gridmap::get_value_at_index(int row, int column) {
    return 0;
}