#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "oculus_interfaces/msg/ping.hpp"

#include "cv_bridge/cv_bridge.h"

#include <opencv2/opencv.hpp>

#include <cmath>
#include <chrono>
#include <fstream>

// 生成当前时间的字符串，格式为 YYYYMMDD_HHMMSS
std::string getCurrentTimeString() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
    return ss.str();
}

void saveVectorToCSV(const std::vector<std::vector<uint8_t>>& data, const std::string& filename) {
    // 打开文件流
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return;
    }

    // 遍历二维向量
    for (const auto& row : data) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << static_cast<int>(row[i]);  // 将uint8_t转换为int写入文件
            if (i < row.size() - 1) {
                file << ",";  // 用逗号分隔每个元素
            }
        }
        file << "\n";  // 换行
    }

    // 关闭文件流
    file.close();
    std::cout << "文件已保存到: " << filename << std::endl;
}

void saveVectorToBeams(const std::vector<int16_t>& data, const std::string& filename) {
    // 打开文件流
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return;
    }

    // 遍历向量，将每个元素写入文件
    for (const auto& value : data) {
        file << value << "\n";  // 每个值占一行
    }

    // 关闭文件流
    file.close();
    std::cout << "文件已保存到: " << filename << std::endl;
}


class m750d_save_ping : public rclcpp::Node
{
private:
    // 声明
    rclcpp::Subscription<oculus_interfaces::msg::Ping>::SharedPtr m750d_ping_subscribe_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m750d_pointcloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m750d_edge_pointcloud_publisher_;
    // rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m750d_laserscan_publisher_;

public:
    // 构造函数,有一个参数为节点名称
    m750d_save_ping(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());

        m750d_ping_subscribe_ = this->create_subscription<oculus_interfaces::msg::Ping>("/sonar/ping", 10, \
                                    std::bind(&m750d_save_ping::m750d_ping_callback, this, std::placeholders::_1));
        m750d_pointcloud_publisher_ =  this->create_publisher<sensor_msgs::msg::PointCloud2>("/sonar/points2", 10); 
        m750d_edge_pointcloud_publisher_ =  this->create_publisher<sensor_msgs::msg::PointCloud2>("/sonar/edge_points", 10); 
        // m750d_laserscan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/m750d/scan", 10);
    }

    // 收到 m750d ping 的回调函数
    void m750d_ping_callback(const oculus_interfaces::msg::Ping::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "save ping ");
        const int n_beams = msg->n_beams;
        int n_ranges = msg -> n_ranges;
        int n_step = msg -> step;
        double scanRange = msg->range;
        int offset = 2048;

        std::vector<int16_t> bearings(n_beams);
        memcpy((void*)bearings.data(), (void*)(msg->bearings).data(), n_beams*sizeof(int16_t));
        std::vector<std::vector<uint8_t>> range_beam_map;
        std::vector<uint8_t> one_range(n_beams);
        for(int i = 0; i<n_ranges;i++){
            memcpy((void*)one_range.data(), (void*)((msg->ping_data).data()+ offset + 4 +n_step*i ), n_beams);
            range_beam_map.push_back(one_range);
        }

        // 生成带时间戳的文件名
        std::string current_time = getCurrentTimeString();
        std::string filename = "data_" + current_time + ".csv";
        std::string filename2 = "beams_"+current_time + ".csv";
        // 保存为CSV文件
        saveVectorToCSV(range_beam_map, filename);
        saveVectorToBeams(bearings,filename2);

        rclcpp::shutdown();

    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<m750d_save_ping>("m750d_save_ping");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

