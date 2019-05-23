// ros
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/image_encodings.h>
// image
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
// standard
#include <vector>


class GridmapConverter {
public:
    GridmapConverter(ros::NodeHandle &nh);
    virtual ~GridmapConverter();
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it;
    image_transport::Publisher image_pub;

    ros::Subscriber env_sub;
    ros::Subscriber static_sub;
    ros::Subscriber dynamic_sub;

    float map_resolution;
    int map_width, map_height;
    float origin_x, origin_y;
    geometry_msgs::Pose map_origin;

    int STATIC_THRESH;

    void env_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void static_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void dynamic_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
};