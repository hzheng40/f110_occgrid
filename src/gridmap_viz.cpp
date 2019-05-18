#include "f110_occgrid/gridmap_viz.h"

GridmapViz::~GridmapViz() {
    ROS_INFO("Occupancy grid visualization shutting down");
}

GridmapViz::GridmapViz(ros::NodeHandle &nh) : nh_(nh) {
    grid_pub = nh_.advertise<visualization_msgs::Marker>("grid_vis", 10);

    env_sub = nh_.subscribe("env_layer", 10, &GridmapViz::env_callback, this);
    static_sub = nh_.subscribe("static_layer", 10, &GridmapViz::static_callback, this);
    dynamic_sub = nh_.subscribe("dynamic_layer", 10, &GridmapViz::dynamic_callback, this);
}

void GridmapViz::env_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/laser";
    marker.type = marker.CUBE_LIST;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;

    std_msgs::ColorRGBA col_r;
    col_r.a = 1.0;
    col_r.r = 1.0;
    col_r.g = 0.0;
    col_r.b = 0.0;

    std_msgs::ColorRGBA col_g;
    col_g.a = 1.0;
    col_g.r = 0.0;
    col_g.g = 1.0;
    col_g.b = 0.0;

    for (int i=0; i<msg->data.size()/3; i++){
        geometry_msgs::Point cube;
        cube.x = msg->data[i*3];
        cube.y = msg->data[i*3+1];

        if (msg->data[i*3+2] == 1.0) {
            marker.points.push_back(cube);
            marker.colors.push_back(col_r);
        } else if (msg->data[i*3+2] == 0.0) {
            marker.points.push_back(cube);
            marker.colors.push_back(col_g);
        }
    }
    g_pub.publish(marker);
}

void GridmapViz::

int main(int argc, char** argv) {
    ros::init(argc, argv, "vis_tree");
    ros::NodeHandle nh;
    RRTVIS rrt_vis(nh);
    ros::spin();
    return 0;
}