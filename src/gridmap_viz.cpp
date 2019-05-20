#include "f110_occgrid/gridmap_viz.h"
#include "f110_occgrid/gridmap.h"

GridmapViz::~GridmapViz() {
    ROS_INFO("Occupancy grid visualization shutting down");
}

GridmapViz::GridmapViz(ros::NodeHandle &nh) : nh_(nh) {
    env_viz_pub = nh_.advertise<visualization_msgs::Marker>("env_viz", 10);
    statc_viz_pub = nh_.advertise<visualization_msgs::Marker>("static_viz", 10);
    dynamic_viz_pub = nh_.advertise<visualization_msgs::Marker>("dynamic_viz", 10);

    env_sub = nh_.subscribe("env_layer", 10, &GridmapViz::env_callback, this);
    static_sub = nh_.subscribe("static_layer", 10, &GridmapViz::static_callback, this);
    dynamic_sub = nh_.subscribe("dynamic_layer", 10, &GridmapViz::dynamic_callback, this);

    // called once
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
    // don't think orientation is used here
    ROS_INFO("Map Metadata Loaded");
}

// callbacks
void GridmapViz::env_callback(const nav_msgs::OccupancyGrid::ConstPtr& env_layer_msg) {
    std::vector<int8_t> env_layer_raw = env_layer_msg->data;
    std::vector<int> env_layer(env_layer_raw.begin(), env_layer_raw.end());
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.type = marker.CUBE_LIST;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;

    std_msgs::ColorRGBA col;
    col.a = 1.0;
    col.r = 1.0;
    col.g = 1.0;
    col.b = 1.0;

    for (int i=0; i<env_layer.size(); i++){
        geometry_msgs::Point cube;
        cube.x = msg->data[i*3];
        cube.y = msg->data[i*3+1];
        marker.points.push_back(cube);
        marker.colors.push_back(col);
    }
    env_viz_pub.publish(marker);
}

void GridmapViz::static_callback(const nav_msgs::OccupancyGrid::ConstPtr& staic_layer_msg){
    std::vector<int8_t> static_layer_raw = static_layer_msg->data;
    std::vector<int> static_layer(static_layer_raw.begin(), static_layer_raw.end());
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.type = marker.CUBE_LIST;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;

    std_msgs::ColorRGBA col;
    col.a = 1.0;
    col.r = 1.0;
    col.g = 0.0;
    col.b = 0.0;

    for (int i=0; i<static_layer.size(); i++){
        geometry_msgs::Point cube;
        cube.x = msg->data[i*3];
        cube.y = msg->data[i*3+1];
        marker.points.push_back(cube);
        marker.colors.push_back(col);
    }
    static_viz_pub.publish(marker);
}

void GridmapViz::dynamic_callback(const nav_msgs::OccupancyGrid::ConstPtr& staic_layer_msg){
    std::vector<int8_t> dynamic_layer_raw = dynamic_layer_msg->data;
    std::vector<int> dynamic_layer(dynamic_layer_raw.begin(), dynamic_layer_raw.end());
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.type = marker.CUBE_LIST;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;

    std_msgs::ColorRGBA col;
    col.a = 1.0;
    col.r = 0.0;
    col.g = 0.0;
    col.b = 1.0;

    for (int i=0; i<dynamic_layer.size(); i++){
        geometry_msgs::Point cube;
        cube.x = msg->data[i*3];
        cube.y = msg->data[i*3+1];
        marker.points.push_back(cube);
        marker.colors.push_back(col);
    }
    dynamic_viz_pub.publish(marker);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gridmap_viz");
    ros::NodeHandle nh;
    RRTVIS rrt_vis(nh);
    ros::spin();
    return 0;
}