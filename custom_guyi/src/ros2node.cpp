#include "custom_guyi/ros2node.hpp"

bool fileExists(const std::string& directory, const std::string& extension) {
    try {
        boost::filesystem::recursive_directory_iterator it(directory), end;
        while (it != end) {
            if (boost::filesystem::is_regular_file(*it) && it->path().extension() == extension) {
                return true;
            }
            ++it;
        }
    } catch (const boost::filesystem::filesystem_error& e) {
        std::cerr << "Filesystem error: " << e.what() << std::endl;
    }
    return false;
}

std::string jetsonCheck(const std::string& device) {
    if (device == "jetson_1") {
        int x = system("ping -c1 -s1 192.168.0.100  > /dev/null 2>&1");
        return (x == 0) ? "Active" : "Not Active";
    } else if (device == "jetson_2") {
        int x = system("ping -c1 -s1 192.168.0.150  > /dev/null 2>&1");
        return (x == 0) ? "Active" : "Not Active";
    } else {
        return "Not Active";
    }
}

Ros2Node::Ros2Node()
  : Node("ros2_node"), depth(0.0f)
{
    imu = "Not Active";
    orin = "Not Active";
    sonar = "Not Active";

    declare_parameter<std::string>("device", "");
    get_parameter("device", device);

    declare_parameter<std::string>("cam_topic", "flir_camera/image_raw/compressed");
    declare_parameter<std::string>("depth_topic", "bar30/depth");
    declare_parameter<std::string>("sonar_topic", "imagenex831l/sonar_health");
    declare_parameter<std::string>("imu_topic", "imu/data");

    std::string cam_topic;
    std::string depth_topic;
    std::string sonar_topic;
    std::string imu_topic;

    get_parameter("cam_topic", cam_topic);
    get_parameter("depth_topic", depth_topic);
    get_parameter("sonar_topic", sonar_topic);
    get_parameter("imu_topic", imu_topic);

    cam_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        cam_topic, 10, std::bind(&Ros2Node::cam_callback, this, std::placeholders::_1));

    depth_sub_ = create_subscription<std_msgs::msg::Float32>(
        depth_topic, 10, std::bind(&Ros2Node::depth_callback, this, std::placeholders::_1));

    sonar_sub_ = create_subscription<std_msgs::msg::String>(
        sonar_topic, 10, std::bind(&Ros2Node::sonar_callback, this, std::placeholders::_1));
    
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, 10, std::bind(&Ros2Node::imu_callback, this, std::placeholders::_1));
}

Ros2Node::~Ros2Node()
{
    // Reset subscriptions to release resources
    cam_sub_.reset();
    depth_sub_.reset();
    sonar_sub_.reset();
    imu_sub_.reset();
}

cv::Mat Ros2Node::getRosMsg() {
    return cv_ptr;
}

std::string Ros2Node::getDepth() {
    return (depth_sub_->get_publisher_count() > 0) ? std::to_string(depth) : "Not Active";
}

std::string Ros2Node::getSonar() {
    return (sonar_sub_->get_publisher_count() > 0) ? sonar : "Not Active";
}

std::string Ros2Node::getIMU() {
    return (imu_sub_->get_publisher_count() > 0) ? imu : "Not Active";
}

std::string Ros2Node::getOrin() {
    orin = jetsonCheck(device);
    return orin;
}

std::string Ros2Node::getBag() {
    std::string directory = ".";
    std::string file = ".db3";
    return fileExists(directory, file) ? "Active" : "Not Active";
}

void Ros2Node::cam_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {

    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
}

void Ros2Node::depth_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    depth = msg->data;
}

void Ros2Node::sonar_callback(const std_msgs::msg::String::SharedPtr msg) {
    sonar = msg->data;
}

void Ros2Node::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    imu = (msg != nullptr) ? "Active" : "Not Active";
}
