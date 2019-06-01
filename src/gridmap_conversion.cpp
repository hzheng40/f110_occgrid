#include <sensor_msgs/LaserScan.h>
#include "f110_occgrid/gridmap_conversion.h"

GridmapConverter::~GridmapConverter() {
    cv::destroyWindow(OPENCV_WINDOW);
    ROS_INFO("Gridmap Converter shutting down.");
}
GridmapConverter::GridmapConverter(ros::NodeHandle &nh) : nh_(nh), it(nh) {
    // image publisher
    image_pub = it.advertise("/converted_map", 1);
    // layer subscribers
    laser_sub = nh_.subscribe("/laser", 10, &GridmapConverter::laser_callback, this);

    // params
    GRID_LENGTH = IMG_HEIGHT*IMG_WIDTH;
    // 1d grid
    occgrid.reserve(GRID_LENGTH);
    std::fill(occgrid.begin(), occgrid.begin()+GRID_LENGTH, 0);
}
void GridmapConverter::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {

}

bool GridmapConverter::get_converted_image(f110_occgrid::ConvertMap::Request &req, f110_occgrid::ConvertMap::Response &res) {

    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gridmap_conversion");
    ros::NodeHandle nh;
    GridmapConverter gridmap_converter(nh);
    ros::ServiceServer service = nh.advertiseService("gridmap_conversion", &GridmapConverter::get_converted_image, &gridmap_converter);
    ros::spin();
    return 0;
}