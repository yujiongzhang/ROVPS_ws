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

    std::pair<double, double> last_point;

public:
    // 构造函数,有一个参数为节点名称
    sts1000_ping2pc(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
        sts1000_ping_subscribe_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/sts1000/ping", 10, \
                                    std::bind(&sts1000_ping2pc::sts1000_ping_callback, this, std::placeholders::_1));
        sts1000_pointcloud_publisher_ =  this->create_publisher<sensor_msgs::msg::PointCloud2>("/sts1000/edge_points", 10); 
        // sts1000_ping_subscribe_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/sonar_micron_ros", 10, \
        //                             std::bind(&sts1000_ping2pc::sts1000_ping_callback, this, std::placeholders::_1));
        // sts1000_pointcloud_publisher_ =  this->create_publisher<sensor_msgs::msg::PointCloud2>("/sonar_micron_point", 10); 
        last_point = std::make_pair(-1,-1);
    }

    // 收到 sts1000 ping 的回调函数
    void sts1000_ping_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // const int n_beams = msg->n_beams;
        int n_ranges = (msg->intensities).size();
        double angle = msg->angle_min;


        double resolution = (msg->range_max)/ static_cast<float>(n_ranges);
        RCLCPP_INFO(this->get_logger(), "n_ranges: %d ,angle: %f",n_ranges, angle);

        std::pair<double,double> cur_point = std::make_pair(-1,-1);
        std::vector<std::pair<double, double>> points_map;//存储符合要求的点的map


        for (int range_i = 0; range_i < n_ranges; range_i++)
        {
            if ((msg->intensities)[range_i]>55) // 能量阈值设置
            {
                double range = (msg->ranges)[range_i];
                if ( range > 1.0)
                {
                    double px = cos(angle) * range;
                    double py = sin(angle) * range;
                    cur_point.first = px;
                    cur_point.second = py;
                    break;
                }
            }
        }

        if(cur_point.first != -1){//这次找到边缘点
            points_map.push_back(cur_point);

            if(last_point.first != -1 && std::fabs(cur_point.first - last_point.first)<1 && std::fabs(cur_point.second - last_point.second)<1 ){//上次找到边缘点 并且满足要求
                // 根据实际情况进行插值
                double delta_len = std::sqrt(std::pow(last_point.first-cur_point.first,2) + std::pow(last_point.second-cur_point.second,2));
                int n = static_cast<int>(delta_len / resolution) + 1;
                for (int i = 0; i < n; i++)
                {
                    std::pair<double,double> interpolation = std::make_pair(((n-i)*(cur_point.first)+(i+1.0)*last_point.first)/static_cast<double>(n+1.0),\
                                                                        ((n-i)*(cur_point.second)+(i+1.0)*last_point.second)/static_cast<double>(n+1.0));
                    points_map.push_back(interpolation);
                }
                RCLCPP_INFO(this->get_logger(), "interpolation n: %d ,delta_len: %f",n, delta_len);
            }

            last_point = cur_point;
        }
        else{//这次没有找到边缘点
            last_point = std::make_pair(-1,-1);
            return;
        }

        sensor_msgs::msg::PointCloud2 pc2_msg;
        pc2_msg.header = msg->header;
        
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

