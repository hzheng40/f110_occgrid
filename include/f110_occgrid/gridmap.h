// ros stuff
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/LaserScan.h>
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
    Eigen::MatrixXi env_layer;
    Eigen::MatrixXi static_layer;
    Eigen::MatrixXi dynamic_layer;

    // map params
    bool INIT;
    float map_resolution; // m/cell
    int map_width, map_height; // cells
    geometry_msgs::Pose map_origin; // cell(0,0), [m, m, rad]
    float origin_x, origin_y;

    // laser params
    bool LASER_INIT;
    std::vector<float> angles_vector;
    int SCAN_COUNT;

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    Eigen::MatrixXi get_env_layer();
    Eigen::MatrixXi get_static_layer();
    Eigen::MatrixXi get_dynamic_layer();

    std::vector<int> ind_2_rc(int ind);
    geometry_msgs::Point cell_2_coord(int row, int col);
    geometry_msgs::Point cell_2_coord(int ind);

    int coord_2_cell_rc(float x, float y);
    std::vector<int> coord_2_cell_1d(float x, float y);
};