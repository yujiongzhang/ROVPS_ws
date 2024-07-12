#include "rclcpp/rclcpp.hpp"

// #include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <cmath>


class sts1000_ping2pc : public rclcpp::Node
{
private:
    // 声明
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sts1000_ping_subscribe_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sts1000_pointcloud_publisher_;

public:
    // 构造函数,有一个参数为节点名称
    sts1000_ping2pc(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
        sts1000_ping_subscribe_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/sts1000/ping", 10, \
                                    std::bind(&sts1000_ping2pc::sts1000_ping_callback, this, std::placeholders::_1));
        sts1000_pointcloud_publisher_ =  this->create_publisher<sensor_msgs::msg::PointCloud2>("/sts1000/edge_points", 10); 
    }

    // 收到 sts1000 ping 的回调函数
    void sts1000_ping_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "receive /sts1000/ping");
        // const int n_beams = msg->n_beams;
        int n_ranges = (msg->intensities).size();
        double angle = msg->angle_min;

        std::vector<std::pair<double, double>> points_map;//存储符合要求的点的map


        bool is_edge = true;
        for (int range_i = 0; range_i < n_ranges; range_i++)
        {
            if ((msg->intensities)[range_i]>60) // 能量阈值设置
            {
                double range = (msg->ranges)[range_i];
                double px = cos(angle) * range;
                double py = sin(angle) * range;
                if (is_edge && range > 1.5)
                {
                    points_map.push_back(std::pair<double, double>(px, py));
                    is_edge = false;
                    break;
                }
            }
        }
        

        sensor_msgs::msg::PointCloud2 pc2_msg;
        pc2_msg.header = msg->header;
        pc2_msg.header.frame_id = "sts1000_link";
        
        pc2_msg.height = 1;
        pc2_msg.width = points_map.size();
        pc2_msg.fields.resize(3);
        pc2_msg.fields[0].name = "x";
        pc2_msg.fields[0].offset = 0;
        pc2_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pc2_msg.fields[0].count = 1;
        pc2_msg.fields[1].name = "y";
        pc2_msg.fields[1].offset = 4;
        pc2_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pc2_msg.fields[1].count = 1;
        pc2_msg.fields[2].name = "z";
        pc2_msg.fields[2].offset = 8;
        pc2_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pc2_msg.fields[2].count = 1;
        // pc2_msg.fields[3].name = "intensity";
        // pc2_msg.fields[3].offset = 12;
        // pc2_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        // pc2_msg.fields[3].count = 1;

        pc2_msg.point_step = 12;
        pc2_msg.row_step = pc2_msg.point_step * pc2_msg.width;
        pc2_msg.is_bigendian = false;
        pc2_msg.is_dense = true;
        pc2_msg.data.resize(pc2_msg.width * pc2_msg.point_step);

        float* data_ptr = reinterpret_cast<float*>(&pc2_msg.data[0]);

        for (int i=0; i<(int) points_map.size(); i++) {
            int index = i * 3;
            data_ptr[index] = points_map[i].first;
            data_ptr[index+1] = points_map[i].second;
            data_ptr[index+2] = 0.0;
        }

        sts1000_pointcloud_publisher_->publish(pc2_msg);

        RCLCPP_INFO(this->get_logger(), "n_ranges: %d ,angle: %f",n_ranges, angle);

    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<sts1000_ping2pc>("sts1000_ping2pc");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

