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
    ROS_INFO("Map Metadata Loaded.");

    // params/attr init
    INIT = false;
    INFLATION = 3;
    STATIC_THRESH = 3; // if seen n times then considered static obs
    
    env_layer.resize(map_height, map_width);
    env_layer.setZero();
    
    static_layer.resize(map_height, map_width);
    static_layer.setZero();
    
    dynamic_layer.resize(map_height, map_width);
    dynamic_layer.setZero();

    ROS_INFO("Gridmap node object init done.");
}

// callbacks
void Gridmap::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {
    // reroute to env_layer
    env_pub.publish(map_msg);
    ROS_INFO("Map rerouted.");
    // if (INIT)? if slow only run this once with flag
    if (INIT) return;
    std::vector<int8_t> map_data = map_msg->data;
    // convert to int
    std::vector<int> map_data_int(map_data.begin(), map_data.end());
    // save data to attribute
    // map value 100 if occupied, 0 if free
    int* data_start = map_data_int.data();
    Eigen::Map<Eigen::MatrixXi>(data_start, env_layer.rows(), env_layer.cols()) = env_layer;
    INIT = true;
    ROS_INFO("Map in Eigen.");
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
    // put scan into dynamic layer
    for (int i=0; i<SCAN_COUNT; i++) {
        double range = ranges[i];
        if (std::isnan(range) || std::isinf(range)) continue;
        double x = range*cos(angles_vector[i]), y = range*sin(angles_vector[i]);
        int grid_x = x/map_resolution, grid_y = y/map_resolution;
        // check bounds
        if (out_of_bounds(grid_x, grid_y)) continue;
        // add inflation
        for (int i_f=-INFLATION; i_f<=INFLATION; i_f++) {
            for (int j_f=-INFLATION; j_f<=INFLATION; j_f++) {
                int current_x = grid_x - i_f, current_y = grid_y - j_f;
                if (out_of_bounds(current_x, current_y)) continue;
                dynamic_layer(current_y, current_x) = 100;
            }
        }
    }

    // find overlap between dynamic and env, 1 if true, 0 if false
    Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> env_mask = (env_layer.array() > 0);
    Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> dynamic_mask = (dynamic_layer.array() > 0);
    Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> env_dynamic_overlap = env_mask && dynamic_mask;
    std::vector<int> env_dynamic_overlap_ind = find_nonzero(env_dynamic_overlap);

    // removing hits from dynamic layer if overlapped
    for (int i_o=0; i_o<env_dynamic_overlap_ind.size(); i_o++) {
        std::vector<int> current_ind = ind_2_rc(env_dynamic_overlap_ind[i_o]);
        dynamic_layer(current_ind[0], current_ind[1]) = 0;
    }
    // find overlap between new dynamic and static, 1 if true, 0 if false
    Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> dynamic_mask_new = (dynamic_layer.array() > 0);
    Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> static_mask = (static_layer.array() > 0);
    Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> dynamic_static_overlap = dynamic_mask_new && static_mask;
    std::vector<int> dynamic_static_overlap_ind = find_nonzero(dynamic_static_overlap);
    // increment static layer values for overlap, remove hits over thresh from dynamic
    for (int i_d=0; i_d<dynamic_static_overlap_ind.size(); i_d++) {
        std::vector<int> current_ind = ind_2_rc(dynamic_static_overlap_ind[i_d]);
        static_layer(current_ind[0], current_ind[1])++;
        if (static_layer(current_ind[0], current_ind[1]) >= STATIC_THRESH) {
            dynamic_layer(current_ind[0], current_ind[1]) = 0;
        }
    }

    // publish to the topics
    pub_layers();

}

// utils
// publish layers, default: no argument, publishes all current layer via attr
void Gridmap::pub_layers() {
    // env layer not needed, already done in map callback
    // static layer
    nav_msgs::OccupancyGrid static_layer_msg;
    std::vector<int> static_data(static_layer.data(), static_layer.data()+static_layer.size());
    // convert to int8
    std::vector<int8_t> static_data_int8(static_data.begin(), static_data.end());
    static_layer_msg.data = static_data_int8;

    // dynamic layer
    nav_msgs::OccupancyGrid dynamic_layer_msg;
    std::vector<int> dynamic_data(dynamic_layer.data(), dynamic_layer.data()+dynamic_layer.size());
    // convert to int8
    std::vector<int8_t> dynamic_data_int8(dynamic_data.begin(), dynamic_data.end());
    dynamic_layer_msg.data = dynamic_data_int8;

    // publish
    static_pub.publish(static_layer_msg);
    dynamic_pub.publish(dynamic_layer_msg);
}

// publish layers overload, specify layer and topic
void Gridmap::pub_layers(Eigen::MatrixXi layer, ros::Publisher publisher) {
    nav_msgs::OccupancyGrid layer_msg;
    std::vector<int> layer_data;
    Eigen::Map<Eigen::MatrixXi>(layer_data.data(), map_height, map_width) = layer;
    std::vector<int8_t> layer_data_int8(layer_data.begin(), layer_data.end());
    layer_msg.data = layer_data_int8;
    publisher.publish(layer_msg);
}

std::vector<int> Gridmap::find_nonzero(Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> arr) {
    std::vector<int> ind;
    for (int i=0; i<arr.size(); i++) {
        if (arr(i)) {
            ind.push_back(i);
        }
    }
    return ind;
}

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

bool Gridmap::out_of_bounds(int x, int y) {
    return (x < 0 || y < 0 || x >= map_height || y >= map_width);
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