#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>

#include <vector>

class GridmapViz {
public:
    GridmapViz(ros::NodeHandle &nh);
    virtual ~GridmapViz();
private:
    ros::NodeHandle nh_;
    ros::Publisher grid_pub;

    ros::Subscriber env_sub;
    ros::Subscriber static_sub;
    ros::Subscriber dynamic_sub;

    float map_resolution;
    int map_width, map_height;
    float origin_x, origin_y;
    geometry_msgs::Pose map_origin;

    void env_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void static_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void dynamic_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
};