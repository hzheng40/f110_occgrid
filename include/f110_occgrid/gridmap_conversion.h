// ros
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/image_encodings.h>
// service
#include "f110_occgrid/ConvertMap.h"
// image
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
// standard
#include <vector>

static const std::string OPENCV_WINDOW = "Converted Map";

class GridmapConverter {
public:
    GridmapConverter(ros::NodeHandle &nh);
    virtual ~GridmapConverter();
    // service function
    bool get_converted_image(f110_occgrid::ConvertMap::Request &req, f110_occgrid::ConvertMap::Response &res);
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it;
    image_transport::Publisher image_pub;

    ros::Subscriber laser_sub;

    float map_resolution;
    int map_width, map_height;
    float origin_x, origin_y;
    geometry_msgs::Pose map_origin;

    int GRID_LENGTH, IMG_WIDTH, IMG_HEIGHT;

    std::vector<int> occgrid;

    // callbacks
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

};