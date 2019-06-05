#include "f110_occgrid/gridmap.h"

Gridmap::~Gridmap() {
    ROS_INFO("Gridmap node shutting down");
}
Gridmap::Gridmap(ros::NodeHandle &nh) : nh_(nh), it(nh) {
    // publishers
    env_pub = nh_.advertise<nav_msgs::OccupancyGrid>("env_layer", 10);
    static_pub = nh_.advertise<nav_msgs::OccupancyGrid>("static_layer", 10);
    dynamic_pub = nh_.advertise<nav_msgs::OccupancyGrid>("dynamic_layer", 10);

    // subscribers
    scan_sub = nh_.subscribe("/scan", 10, &Gridmap::scan_callback, this);
    map_sub = nh_.subscribe("/map", 10, &Gridmap::map_callback, this);

    // called once
    // nav_msgs::OccupancyGrid env_map_msg = ros::topic::waitForMessage("/map")
    boost::shared_ptr<nav_msgs::MapMetaData const> env_metadata_ptr;
    nav_msgs::MapMetaData env_metadata_msg;
    env_metadata_ptr = ros::topic::waitForMessage<nav_msgs::MapMetaData>("/map_metadata");
    if (env_metadata_ptr != NULL) {
        env_metadata_msg = *env_metadata_ptr;
    }
    all_map_metadata = env_metadata_msg;
    map_resolution = env_metadata_msg.resolution;
    map_width = env_metadata_msg.width;
    map_height = env_metadata_msg.height;
    map_origin = env_metadata_msg.origin;
    geometry_msgs::Point origin = map_origin.position;
    origin_x = origin.x;
    origin_y = origin.y;
    ROS_INFO("Map Metadata Loaded.");

    // params/attr init
    INIT = false;
    INFLATION = 1;
    STATIC_THRESH = 50; // if seen n times then considered static obs
    img_size = 200;

    image_pub = it.advertise("/converted_map", 1);
    
    env_layer.resize(map_height, map_width);
    env_layer.setZero();
    
    static_layer.resize(map_height, map_width);
    static_layer.setZero();
    
    dynamic_layer.resize(map_height, map_width);
    dynamic_layer.setZero();

    // making sure tf between map and laser is published before running
    ros::Time now = ros::Time::now();
    listener.waitForTransform("/map", "/laser", now, ros::Duration(3.0));
    ROS_INFO("Transform arrived.");

    ROS_INFO("Gridmap node object init done.");
}

// callbacks
void Gridmap::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {
    // reroute to env_layer
    env_layer_msg = *map_msg;
    ROS_INFO("Map rerouted.");
    // if (INIT)? if slow only run this once with flag
    if (INIT) return;
    std::vector<int8_t> map_data = map_msg->data;
    // convert to int
    std::vector<int> map_data_int(map_data.begin(), map_data.end());
    // save data to attribute
    // map value 100 if occupied, 0 if free
    int* data_start = map_data_int.data();
    Eigen::Map<Eigen::MatrixXi>(data_start, env_layer.rows(), env_layer.cols()) = env_layer;
    INIT = true;
    ROS_INFO("Map in Eigen.");
}

void Gridmap::scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    // fill laser params if it's the first time receiving info
    if (!LASER_INIT) {
        std::vector<float> ranges = scan_msg->ranges;
        SCAN_COUNT = ranges.size();
        angles_vector.reserve(SCAN_COUNT);
        for (int i=0; i<SCAN_COUNT; i++) {
            angles_vector[i] = scan_msg->angle_min + scan_msg->angle_increment*i;
        }
        LASER_INIT = true;
    }
    // steps in laser callback:
    // 1. put everything in dynamic layer
    // 2. find overlap between dynamic and env, remove overlaps in dynamic
    // 3. find overlap between dynamic and static, increment value in static,
    // and if the value over threshold, remove overlaps in dynamic.
    // NEW 3. previous doesn't work because there's no static to start with
    // new approach: before checking overlap, increment value that's in dynamic
    std::vector<float> ranges = scan_msg->ranges;
    // put scan into dynamic layer
    for (int i=0; i<SCAN_COUNT; i++) {
        double range = ranges[i];
        if (std::isnan(range) || std::isinf(range)) continue;
        // these are in the frame of /laser
        double x = range*cos(angles_vector[i]), y = range*sin(angles_vector[i]);
        // transform into map frame
        geometry_msgs::PointStamped before_tf;
        before_tf.point.x = x;
        before_tf.point.y = y;
        before_tf.header.frame_id = "/laser";
        geometry_msgs::PointStamped after_tf;
        after_tf.header.frame_id = "/map";
        listener.transformPoint("/map", before_tf, after_tf);
        std::vector<int> laser_rc = coord_2_cell_rc(after_tf.point.x, after_tf.point.y);
        int laser_r = laser_rc[0];
        int laser_c = laser_rc[1];
        // check bounds
        if (out_of_bounds(laser_r, laser_c)) continue;
        // add inflation
        for (int i_f=-INFLATION; i_f<=INFLATION; i_f++) {
            for (int j_f=-INFLATION; j_f<=INFLATION; j_f++) {
                int current_r = laser_r - i_f, current_c = laser_c - j_f;
                if (out_of_bounds(current_r, current_c)) continue;
                dynamic_layer(current_r, current_c) = 100;
            }
        }
    }
    
    // find overlap between dynamic and env, 1 if true, 0 if false

    // layers should have the same sizes
    // looping over data pointer, way faster
    // ros::Time b4overlap = ros::Time::now();
    for (size_t i_layer=0, layer_size = env_layer.size(); i_layer<layer_size; i_layer++) {
        int *env = env_layer.data();
        int *dyn = dynamic_layer.data();
        int *stat = static_layer.data();
        if (env[i_layer] > 0 && dyn[i_layer] > 0) {
            dyn[i_layer] = 0;
        }
        if (dyn[i_layer] > 0 && stat[i_layer] < 100) {
            stat[i_layer]++;
        }
        if (dyn[i_layer] > 0 && stat[i_layer] > STATIC_THRESH) {
            dyn[i_layer] = 0;
        }
    }
    // ros::Time aftoverlap = ros::Time::now();
    // ros::Duration diff = aftoverlap - b4overlap;
    // std::cout << "overlap time diff: " << diff << std::endl;


    // Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> env_mask = (env_layer.array() > 0);
    // Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> dynamic_mask = (dynamic_layer.array() > 0);
    // Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> env_dynamic_overlap = env_mask && dynamic_mask;
    
    // std::vector<int> env_dynamic_overlap_ind = find_nonzero(env_dynamic_overlap);
    // // removing hits from dynamic layer if overlapped
    // for (int i_o=0; i_o<env_dynamic_overlap_ind.size(); i_o++) {
    //     std::vector<int> current_ind = ind_2_rc(env_dynamic_overlap_ind[i_o]);
    //     dynamic_layer(current_ind[0], current_ind[1]) = 0;
    // }



    // find overlap between new dynamic and static, 1 if true, 0 if false
//     Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> dynamic_mask_new = (dynamic_layer.array() > 0);
//     std::vector<int> dynamic_ind = find_nonzero(dynamic_mask_new);
//     // increment static layer values for those in dynamic
//     for (int i_dy=0; i_dy<dynamic_ind.size(); i_dy++) {
//         std::vector<int> current_ind = ind_2_rc(dynamic_ind[i_dy]);
//         if (static_layer(current_ind[1], current_ind[0]) < 100) {
//             static_layer(current_ind[1], current_ind[0])++;
//         }
//     }
//     Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> static_mask = (static_layer.array() >= STATIC_THRESH);
//     Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> dynamic_static_overlap = dynamic_mask_new && static_mask;
//     std::vector<int> dynamic_static_overlap_ind = find_nonzero(dynamic_static_overlap);
//     // increment static layer values for overlap, remove hits over thresh from dynamic
//     for (int i_d=0; i_d<dynamic_static_overlap_ind.size(); i_d++) {
//         std::vector<int> current_ind = ind_2_rc(dynamic_static_overlap_ind[i_d]);
// //        static_layer(current_ind[0], current_ind[1])++;
//         if (static_layer(current_ind[0], current_ind[1]) >= STATIC_THRESH) {
//             dynamic_layer(current_ind[0], current_ind[1]) = 0;
//         }
//     }

    // publish to the topics
    pub_layers();
    // clear dynamic layer
    dynamic_layer.setZero();
}

// publish layers, default: no argument, publishes all current layer via attr
void Gridmap::pub_layers() {
    // env layer not needed, already done in map callback
    // static layer
    nav_msgs::OccupancyGrid static_layer_msg;
    static_layer_msg.info = all_map_metadata;
    std::vector<int> static_data(static_layer.data(), static_layer.data()+static_layer.size());
    // convert to int8
    std::vector<int8_t> static_data_int8(static_data.begin(), static_data.end());
    static_layer_msg.data = static_data_int8;

    // dynamic layer
    nav_msgs::OccupancyGrid dynamic_layer_msg;
    dynamic_layer_msg.info = all_map_metadata;
    std::vector<int> dynamic_data(dynamic_layer.data(), dynamic_layer.data()+dynamic_layer.size());
    // convert to int8
    std::vector<int8_t> dynamic_data_int8(dynamic_data.begin(), dynamic_data.end());
    dynamic_layer_msg.data = dynamic_data_int8;

    // publish
    static_pub.publish(static_layer_msg);
    dynamic_pub.publish(dynamic_layer_msg);
    if (INIT) env_pub.publish(env_layer_msg);
}

// publish layers overload, boolean specifies if images are published
void Gridmap::pub_layers(bool pub_image) {
    // env layer not needed, already done in map callback
    // static layer
    nav_msgs::OccupancyGrid static_layer_msg;
    static_layer_msg.info = all_map_metadata;
    std::vector<int> static_data(static_layer.data(), static_layer.data()+static_layer.size());
    // convert to int8
    std::vector<int8_t> static_data_int8(static_data.begin(), static_data.end());
    static_layer_msg.data = static_data_int8;

    // dynamic layer
    nav_msgs::OccupancyGrid dynamic_layer_msg;
    dynamic_layer_msg.info = all_map_metadata;
    std::vector<int> dynamic_data(dynamic_layer.data(), dynamic_layer.data()+dynamic_layer.size());
    // convert to int8
    std::vector<int8_t> dynamic_data_int8(dynamic_data.begin(), dynamic_data.end());
    dynamic_layer_msg.data = dynamic_data_int8;

    // publish
    static_pub.publish(static_layer_msg);
    dynamic_pub.publish(dynamic_layer_msg);
    if (INIT) env_pub.publish(env_layer_msg);

    if (pub_image) {
        cv::Mat img = layers_2_cv_img();
        cv::Mat transformed_img = transform_img(img);
        sensor_msgs::ImagePtr ros_img = cv_2_ros_img(transformed_img);
        image_pub.publish(ros_img);
    }
}

// publish layers overload, specify layer and topic
void Gridmap::pub_layers(Eigen::MatrixXi &layer, ros::Publisher &publisher) {
    nav_msgs::OccupancyGrid layer_msg;
    layer_msg.info = all_map_metadata;
    std::vector<int> layer_data;
    Eigen::Map<Eigen::MatrixXi>(layer_data.data(), map_height, map_width) = layer;
    std::vector<int8_t> layer_data_int8(layer_data.begin(), layer_data.end());
    layer_msg.data = layer_data_int8;
    publisher.publish(layer_msg);
}

// service function for conversion
bool Gridmap::get_converted_image(f110_occgrid::ConvertMap::Request &req, f110_occgrid::ConvertMap::Response &res) {
    cv::Mat img = layers_2_cv_img();
    cv::Mat transformed_img = transform_img(img);
    sensor_msgs::ImagePtr ros_img = cv_2_ros_img(transformed_img);
    res.image = *ros_img;
    update_img(transformed_img);
    return true;
}
// utils for conversion
// convert from layers mat to cv img
sensor_msgs::ImagePtr Gridmap::layers_2_img() {
    cv::Mat img = layers_2_cv_img();
    sensor_msgs::ImagePtr ros_img = cv_2_ros_img(img);
    return ros_img;
}
cv::Mat Gridmap::layers_2_cv_img() {
    cv::Mat img = cv::Mat::zeros(map_height, map_width, CV_8UC3);
    cv::Mat blue = cv::Mat(map_height, map_width, CV_8UC1, dynamic_layer.data());
    cv::Mat green = cv::Mat(map_height, map_width, CV_8UC1, static_layer.data());
    cv::Mat red = cv::Mat(map_height, map_width, CV_8UC1, env_layer.data());
    cv::Mat channels[] = {blue, green, red};
    cv::merge(channels, 3, img);
    return img;
}
// transform and crop image around car using tf
cv::Mat Gridmap::transform_img(cv::Mat &full_img) {
    cv::Mat img;
    tf::StampedTransform transform;
    try {
        listener.lookupTransform("/laser", "/map", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
    }
    tf::Vector3 origin = transform.getOrigin();
    tf::Quaternion rotation = transform.getRotation();
    double origin_x = origin.x();
    double origin_y = origin.y();
    // by default the axis angle representation should be the z-axis and the rotation angle
    double angle = rotation.getAngle();
    // get car location in map
    boost::shared_ptr<nav_msgs::Odometry const> odom_ptr;
    nav_msgs::Odometry odom_msg;
    odom_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom");
    if (odom_ptr != NULL) {
        odom_msg = *odom_ptr;
    }
    double car_x = odom_msg.pose.pose.position.x;
    double car_y = odom_msg.pose.pose.position.y;
    std::vector<int> car_grid = coord_2_cell_rc(car_x, car_y);
    cv::Point car_center = cv::Point(car_grid[1], car_grid[0]);
    // rotate image by angle, and crop such that car position is at fixed location
    cv::Mat rot_mat = cv::getRotationMatrix2D(car_center, angle, 1.0);
    cv::warpAffine(full_img, img, rot_mat, full_img.size(), cv::INTER_LINEAR);
    // crop around car position
    // cv::Rect car_roi = get_roi(car_grid);
    cv::Rect car_roi(car_grid[1] - img_size/2, car_grid[0]-(3*img_size)/4, img_size, img_size);
    // overlay image of car on top
    img = img(car_roi);
    return img;
}
// get roi from car center
cv::Rect Gridmap::get_roi(std::vector<int> &car_center) {
    cv::Rect car_roi(0,0,0,0);
    int car_x = car_center[1];
    int car_y = car_center[0];
    int x_min = car_x - img_size/2;
    int x_max = car_x + img_size/2;
    int y_min = car_y - (3*img_size)/4;
    int y_max = car_y + img_size/4;
    // if (x_min < 0 || y_min < 0 || )
    return car_roi;
}
// updates current frame image
void Gridmap::update_img(cv::Mat &img) {
    current_img = cv_2_ros_img(img);
}
void Gridmap::update_img(sensor_msgs::ImagePtr &img) {
    current_img = img;
}
// conversions between ros and cv imgs
sensor_msgs::ImagePtr Gridmap::cv_2_ros_img(cv::Mat &img) {
    sensor_msgs::ImagePtr img_msg;
    if (!img.empty()) {
        img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    }
    return img_msg;
}
void Gridmap::cv_2_ros_img(cv::Mat &img, ros::Publisher &img_pub) {
    sensor_msgs::ImagePtr img_msg;
    if (!img.empty()) {
        img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    }
    img_pub.publish(img_msg);
    return;
}
cv::Mat Gridmap::ros_2_cv_img(sensor_msgs::ImagePtr &img) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    return cv_ptr->image;
}
// returns current frame ros image
sensor_msgs::Image Gridmap::get_img() {
    return *current_img;
}
// converts given image to cv image
cv::Mat Gridmap::get_cv_img(sensor_msgs::ImagePtr &image) {
    return ros_2_cv_img(image);
}
// overload, get current frame cv image
cv::Mat Gridmap::get_cv_img() {
    return ros_2_cv_img(current_img);
}

// utils
std::vector<int> Gridmap::find_nonzero(Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> &arr) {
    std::vector<int> ind;
    for (int i=0; i<arr.size(); i++) {
        if (arr(i)) {
            ind.push_back(i);
        }
    }
    return ind;
}

Eigen::MatrixXi Gridmap::get_env_layer() {
    return env_layer;
}

Eigen::MatrixXi Gridmap::get_static_layer() {
    return static_layer;
}

Eigen::MatrixXi Gridmap::get_dynamic_layer() {
    return dynamic_layer;
}

std::vector<int> Gridmap::ind_2_rc(int ind) {
    //[row, col]
    std::vector<int> rc;
    int row = floor(ind/map_width);
    int col = ind%map_width-1;
    rc.push_back(row);
    rc.push_back(col);
    return rc;
}

int Gridmap::rc_2_ind(int r, int c) {
    return r*map_width+c;
}

bool Gridmap::out_of_bounds(int r, int c) {
    return (r < 0 || c < 0 || r >= map_height || c >= map_width);
}

// via r c
geometry_msgs::Point Gridmap::cell_2_coord(int row, int col) {
    geometry_msgs::Point coord;
    coord.x = origin_x + col*map_resolution;
    coord.y = origin_y + row*map_resolution;
    return coord;
}
// via 1D index
geometry_msgs::Point Gridmap::cell_2_coord(int ind) {
    std::vector<int> rc = ind_2_rc(ind);
    geometry_msgs::Point coord;
    coord.x = origin_x + rc[1]*map_resolution;
    coord.y = origin_y + rc[0]*map_resolution;
    return coord;
}
// returns 1D index
int Gridmap::coord_2_cell_ind(double x, double y){
    int col = static_cast<int>((x - origin_x) / map_resolution);
    int row = static_cast<int>((y - origin_y) / map_resolution);
    return rc_2_ind(row, col);
}
// returns rc index
std::vector<int> Gridmap::coord_2_cell_rc(double x, double y) {
    std::vector<int> rc;
    int col = static_cast<int>((x - origin_x) / map_resolution);
    int row = static_cast<int>((y - origin_y) / map_resolution);
    rc.push_back(row);
    rc.push_back(col);
    return rc;
}