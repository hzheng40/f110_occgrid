#include "f110_occgrid/gridmap_conversion.h"
#include "f110_occgrid/gridmap.h"

GridmapConverter::~GridmapConverter() {
    cv::destroyWindow(OPENCV_WINDOW);
    ROS_INFO("Gridmap Converter shutting down.");
}

GridmapConverter::GridmapConverter() {
    // laser init toggle
    LASER_INIT = false;
    // image params
    img_width = 200;
    img_height = 200;
    img_res = 0.5; // m/cell
    car_x = img_width/2;
    car_y = 50;
    car_width = 20;
    car_length = 30;

    ROS_INFO("Gridmap Converter created.");
}

void GridmapConverter::update_scan(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    // only first time updating scan
    if (!LASER_INIT) {
        std::vector<float> ranges = scan_msg->ranges;
        scan = ranges;
        SCAN_COUNT = scan.size();
        angles_vector.reserve(SCAN_COUNT);
        for (int i=0; i<SCAN_COUNT; i++) {
            angles_vector[i] = scan_msg->angle_min + scan_msg->angle_increment*i;
        }
        LASER_INIT = true;
    }
    // update scan and draw image
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Point> road_contour;
    scan = scan_msg->ranges;
    for (int i=0; i<SCAN_COUNT; i++) {
        double range = scan[i];
        if (std::isnan(range) || std::isinf(range)) continue;
        double x = range*cos(angles_vector[i]), y = range*sin(angles_vector[i]);
        double img_x = car_x + x/img_res, img_y = car_y + y/img_res;
        if (out_of_bounds(img_x, img_y)) continue;
        cv::Point current_point(img_x, img_y);
        road_contour.push_back(current_point);
    }
    contours.push_back(road_contour);
    // create blank green image
    cv::Mat img(img_height, img_width, CV_8UC3, cv::Scalar(0, 255, 0));
    // draw gray contour of road
    cv::Scalar road_color(100, 100, 100);
    cv::drawContours(img, contours, 0, road_color, CV_FILLED);
    // draw red car as box
    cv::Point car_top_left(car_x-car_width/2, car_y+car_length/2);
    cv::Point car_bot_right(car_x+car_width/2, car_y-car_length/2);
    cv::rectangle(img, car_top_left, car_bot_right, cv::Scalar(0, 0, 255));
    current_img = img.clone();
    cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_AUTOSIZE);
    cv::imshow(OPENCV_WINDOW, img);
}

bool GridmapConverter::out_of_bounds(int img_x, int img_y) {
    return (img_x < 0 || img_x >= img_width || img_y < 0 || img_y >= img_height);
}





// GridmapConverter::GridmapConverter(ros::NodeHandle &nh) : nh_(nh), it(nh) {
//     // image publisher
//     image_pub = it.advertise("/converted_map", 1);
//     // layer subscribers
//     laser_sub = nh_.subscribe("/laser", 10, &GridmapConverter::laser_callback, this);

//     // params
//     GRID_LENGTH = IMG_HEIGHT*IMG_WIDTH;
//     // 1d grid
//     occgrid.reserve(GRID_LENGTH);
//     std::fill(occgrid.begin(), occgrid.begin()+GRID_LENGTH, 0);
// }
// void GridmapConverter::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {

// }

// bool GridmapConverter::get_converted_image(f110_occgrid::ConvertMap::Request &req, f110_occgrid::ConvertMap::Response &res) {

//     return true;
// }


// int main(int argc, char** argv) {
//     ros::init(argc, argv, "gridmap_conversion");
//     ros::NodeHandle nh;
//     GridmapConverter gridmap_converter(nh);
//     ros::ServiceServer service = nh.advertiseService("gridmap_conversion", &GridmapConverter::get_converted_image, &gridmap_converter);
//     ros::spin();
//     return 0;
// }