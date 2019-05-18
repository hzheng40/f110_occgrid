// ros stuff
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/LaserScan>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>

// standard stuff
#include <math.h>
#include <vector>
#include <array>
#include <iostream>

//Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

class Gridmap {
public:
    Gridmap(ros::NodeHandle &nh);
    virtual ~Gridmap();
private:
    // ros stuff
    ros::NodeHandle nh_;
    ros::Publisher env_pub;
    ros::Publisher static_pub;
    ros::Publisher dynamic_pub;

    ros::Subscriber scan_sub;
    ros::Subscriber map_sub;

    // underlying data structures
    Eigen::MatrixXf env_layer;
    Eigen::MatrixXf static_layer;
    Eigen::MatrixXf dynamic_layer;

    bool INIT;
    float map_resolution; // m/cell
    int map_width, map_height; // cells
    geometry_msgs::Pose map_origin; // cell(0,0), [m, m, rad]

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    Eigen::MatrixXf get_env_layer();
    Eigen::MatrixXf get_static_layer();
    Eigen::MatrixXf get_dynamic_layer();

    float get_value_at_position(int x, int y);
    float get_value_at_index(int row, int column);
};