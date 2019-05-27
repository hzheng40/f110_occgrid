// ros stuff
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
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

    // tf stuff
    tf::TransformListener listener;

    // underlying data structures
    // Eigen::MatrixXi env_layer;
    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> env_layer;
    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> static_layer;
    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> dynamic_layer;
    // Eigen::MatrixXi static_layer;
    // Eigen::MatrixXi dynamic_layer;

    // map params
    bool INIT;
    float map_resolution; // m/cell
    int map_width, map_height; // cells
    geometry_msgs::Pose map_origin; // cell(0,0), [m, m, rad]
    float origin_x, origin_y;
    nav_msgs::MapMetaData all_map_metadata;
    nav_msgs::OccupancyGrid env_layer_msg;
    int STATIC_THRESH; // threshold for value that's considered static obs

    // laser params
    bool LASER_INIT;
    std::vector<float> angles_vector;
    int SCAN_COUNT;

    // occgrid params
    int INFLATION;

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    void pub_layers();
    void pub_layers(Eigen::MatrixXi layer, ros::Publisher publisher);

    std::vector<int> find_nonzero(Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> arr);

    Eigen::MatrixXi get_env_layer();
    Eigen::MatrixXi get_static_layer();
    Eigen::MatrixXi get_dynamic_layer();

    std::vector<int> ind_2_rc(int ind);
    int rc_2_ind(int r, int c);
    bool out_of_bounds(int x, int y);
    geometry_msgs::Point cell_2_coord(int row, int col);
    geometry_msgs::Point cell_2_coord(int ind);

    int coord_2_cell_ind(double x, double y);
    std::vector<int> coord_2_cell_rc(double x, double y);
};