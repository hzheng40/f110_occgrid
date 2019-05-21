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

    // called once
    // nav_msgs::OccupancyGrid env_map_msg = ros::topic::waitForMessage("/map")
    boost::shared_ptr<nav_msgs::MapMetaData const> env_metadata_ptr;
    nav_msgs::MapMetaData env_metadata_msg;
    env_metadata_ptr = ros::topic::waitForMessage<nav_msgs::MapMetaData>("/map_metadata");
    if (env_metadata_ptr != NULL) {
        env_metadata_msg = *env_metadata_ptr;
    }
    map_resolution = env_metadata_msg.resolution;
    map_width = env_metadata_msg.width;
    map_height = env_metadata_msg.height;
    map_origin = env_metadata_msg.origin;
    geometry_msgs::Point origin = map_origin.position;
    origin_x = origin.x;
    origin_y = origin.y;
    ROS_INFO("Map Metadata Loaded");
    INIT = true;
}

// callbacks
void Gridmap::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {
    // reroute to env_layer
    env_pub.publish(map_msg);
    // if (!INIT)? if slow only run this once with flag
    std::vector<int8_t> map_data = map_msg->data;
    // convert to int
    std::vector<int> map_data_int(map_data.begin(), map_data.end());
    // save data to attribute
    int* data_start = map_data_int.data();
    Eigen::Map<Eigen::MatrixXi>(data_start, env_layer.rows(), env_layer.cols()) = env_layer;
}

void Gridmap::scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    // fill laser params if it's the first time receiving info
    if (!LASER_INIT) {
        std::vector<float> ranges = scan_msg->ranges;
        SCAN_COUNT = ranges.size();
        angles_vector.reserve(SCAN_COUNT);
        for (int i=0; i<SCAN_COUNT; i++) {
            angles_vector[i] = scan_msg->angle_min + scan_msg->angle_increment*i;
        }
        LASER_INIT = true;
    }
    // steps in laser callback:
    // 1. put everything in dynamic layer
    // 2. find overlap between dynamic and env, remove overlaps in dynamic
    // 3. find overlap between dynamic and static, increment value in static, and if the value over threshold, remove overlaps in dynamic.
    std::vector<float> ranges = scan_msg->ranges;
    
}

// utils
Eigen::MatrixXi Gridmap::get_env_layer() {
    return env_layer;
}

Eigen::MatrixXi Gridmap::get_static_layer() {
    return static_layer;
}

Eigen::MatrixXi Gridmap::get_dynamic_layer() {
    return dynamic_layer;
}

std::vector<int> Gridmap::ind_2_rc(int ind) {
    //[row, col]
    std::vector<int> rc(2);
    rc.push_back(floor(ind/map_width));
    rc.push_back(ind%map_width + floor(ind/map_width));
    return rc;
}

// via r c
geometry_msgs::Point Gridmap::cell_2_coord(int row, int col) {
    geometry_msgs::Point coord;
    coord.x = origin_x + col*map_resolution;
    coord.y = origin_y - row*map_resolution;
    return coord;
}
// via 1D index
geometry_msgs::Point Gridmap::cell_2_coord(int ind) {
    std::vector<int> rc = ind_2_rc(ind);
    geometry_msgs::Point coord;
    coord.x = origin_x + rc[1]*map_resolution;
    coord.y = origin_y - rc[0]*map_resolution;
    return coord;
}
// returns 1D index
int Gridmap::coord_2_cell_rc(float x, float y){
    return 0;
}
// returns row col index
std::vector<int> Gridmap::coord_2_cell_1d(float x, float y) {
    int ind = coord_2_cell_rc(x, y);
    std::vector<int> rc = ind_2_rc(ind);
    return rc;
}