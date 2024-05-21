#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "define_sts1000.h"

#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>
#include<string.h>
#include<arpa/inet.h>
#include <cmath>

#include <fcntl.h>


class sts1000_dc : public rclcpp::Node
{
private:
    // 声明话题发布者
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr sts1000_raw_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sts1000_pointcloud_publisher_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr  on_set_parameters_callback_handle_;

    // 声名udp接收定时器指针
    rclcpp::TimerBase::SharedPtr udp_rec_timer_;

    int fd;//udp 文件描述符
    uint8_t buf[2048];//接收的数据缓存在buf中
    struct sockaddr_in seraddr;
    struct sockaddr_in cliaddr;

    ANSHEAD_STS1000 anshead;//机械扫描声纳帧头
    CMD_STS1000 cmd_sts1000;//下发的指令

    float scanRange;
    int startGain;
    int diffusion;
    bool is_running;

    int count;


public:
    sts1000_dc(std::string name) : Node(name){
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
        // 创建发布者
        sts1000_raw_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("sts1000_raw", 10);
        sts1000_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sts1000_raw_points", 10);
        
        this->declare_parameter<bool>("is_running", false);     /*声明参数*/
        this->get_parameter("is_running", is_running); /*获取参数*/
        this->declare_parameter<double>("scanRange", SCANRANGE);     /*声明参数*/
        this->get_parameter("scanRange", scanRange); /*获取参数*/
        this->declare_parameter<int>("startGain", STARTGAIN);     /*声明参数*/
        this->get_parameter("startGain", startGain); /*获取参数*/
        this->declare_parameter<int>("diffusion", DIFFUSION);     /*声明参数*/
        this->get_parameter("diffusion", diffusion); /*获取参数*/
        on_set_parameters_callback_handle_ = this-> add_on_set_parameters_callback(std::bind(&sts1000_dc::setConfigCallback, this,std::placeholders::_1));

        memset(&cmd_sts1000,0,sizeof(cmd_sts1000));
        init_cmd(&cmd_sts1000);

        count = 0;

        // 1. 创建通信的套接字
        fd = socket(AF_INET, SOCK_DGRAM, 0);
        if(fd == -1)
        {
            perror("socket");
            exit(0);
        }
        
        // 初始化服务器地址信息
        seraddr.sin_family = AF_INET;
        seraddr.sin_port = htons(5555);    // 大端
        inet_pton(AF_INET, "192.168.1.7", &seraddr.sin_addr.s_addr);

        
        cliaddr.sin_family = AF_INET;
        cliaddr.sin_addr.s_addr = INADDR_ANY;
        cliaddr.sin_port = htons(6666);
        // inet_pton(AF_INET, "127.0.0.1", &seraddr.sin_addr.s_addr);
        int ret = bind(fd, (struct sockaddr*)&cliaddr, sizeof(cliaddr));//客户端可以绑定为固定端口也可以不
        if(ret == -1)
        {
            perror("bind");
            exit(0);
        }

        // 设置非阻塞
        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);

        void* voidPtr = reinterpret_cast<void*>(&cmd_sts1000); // 将结构体指针转换为 void* 类型
        sendto(fd, voidPtr, sizeof(cmd_sts1000), 0 ,(struct sockaddr*)&seraddr,sizeof(seraddr));

        udp_rec_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&sts1000_dc::udp_rec_callback, this));
    }

    ~sts1000_dc(){
        close(fd);
    }

    void udp_rec_callback(){

        // RCLCPP_INFO(this->get_logger(), "udp_rec_callback %d", count);

        if (count >= 20)
        {
            // RCLCPP_INFO(this->get_logger(), "setsonar");
            sendto(fd, (void*)(&cmd_sts1000), sizeof(cmd_sts1000), 0 ,(struct sockaddr*)&seraddr,sizeof(seraddr));
            count = 0;
        }
        count++;

        // 接收数据
        memset(buf, 0, sizeof(buf));
        int recvlen = recvfrom(fd, buf, sizeof(buf), 0, NULL, NULL);// 接收数据, 如果没有数据,该函数阻塞

        if(recvlen == 0){return;}

        if(recvlen % ANSPKTSIZE == 0)
        {
            memcpy(&anshead, buf, sizeof(ANSHEAD_STS1000));
            // recvlen -= ANSPKTSIZE;
            if( anshead.ucHeader == 0xfe && anshead.ucID == 0x01 && anshead.ucRollHi == 0x00 && anshead.ucRollLo == 0x00 )
            {

                int data_len = static_cast<uint>(((anshead.ucDataHi&0x7f)<<7)|(anshead.ucDataLo&0x7f));
                int glb_nAngle = static_cast<uint>((((anshead.ucAngleHi&0x7f)<<7)|(anshead.ucAngleLo&0x7f))/80);

                // RCLCPP_INFO(this->get_logger(), "data_len: %d, glb_nAngle: %d",data_len, glb_nAngle);
                
                // 创建消息
                sensor_msgs::msg::LaserScan sts1000_ping;
                sts1000_ping.header.stamp = rclcpp::Clock().now();
                sts1000_ping.header.frame_id = "sts1000";
                sts1000_ping.angle_min = glb_nAngle /200.0 * 2* M_PI;//弧度
                sts1000_ping.angle_max = glb_nAngle /200.0 * 2* M_PI;
                sts1000_ping.angle_increment = 0.0;
                sts1000_ping.time_increment = 0.0;
                sts1000_ping.scan_time = 0.0;
                sts1000_ping.range_min = 0.0;
                sts1000_ping.range_max = this->scanRange;
                float range_resolution = (this->scanRange) / 500.0;

                std::vector<std::pair<double, double>> scan_map;//存储符合要求的点的map
                double angle = sts1000_ping.angle_min;

                for (int i = 0; i < 500; ++i) {
                    sts1000_ping.ranges.push_back( range_resolution*(1+i));
                    sts1000_ping.intensities.push_back(buf[ sizeof(ANSHEAD_STS1000)+ i]);

                    if(buf[ sizeof(ANSHEAD_STS1000)+ i] > 100){//满足点云提取要求
                        double px = cos(angle) * sts1000_ping.ranges[i];
                        double py = sin(angle) * sts1000_ping.ranges[i];
                        scan_map.push_back(std::pair<double, double>(px, py));
                    }
                }
                sts1000_raw_publisher_->publish(sts1000_ping);


                sensor_msgs::msg::PointCloud2 pc2_msg;
                pc2_msg.header = sts1000_ping.header;
                pc2_msg.height = 1;
                pc2_msg.width = scan_map.size();
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

                for (int i=0; i<(int) scan_map.size(); i++) {
                    int index = i * 3;
                    data_ptr[index] = scan_map[i].first;
                    data_ptr[index+1] = scan_map[i].second;
                    data_ptr[index+2] = 0.0;
                }

                sts1000_pointcloud_publisher_->publish(pc2_msg);
            }
        }
    }


    void diffusion_callback(const rclcpp::Parameter & p) {
        if( p.as_int() <=0 || p.as_int() >= 7)
        {
            RCLCPP_INFO(this->get_logger(), "startGain must be  {0,1,2,3,4,5,6,7}!");
            return;
        }
        this->diffusion = p.as_int();
        RCLCPP_INFO(this->get_logger(), "diffusion is %d",this->diffusion);
    }

    rcl_interfaces::msg::SetParametersResult setConfigCallback(const std::vector<rclcpp::Parameter>& parameters){
        RCLCPP_INFO(this->get_logger(), "setConfigCallback");

        if (parameters.size() != 1) {
            RCLCPP_WARN(get_logger(), "You should set parameters one by one.");
            RCLCPP_INFO_STREAM(get_logger(), "parameters = " << parameters);
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = false;
            result.reason = "Parameters should be set one by one";
            return result;
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "";

        for (const rclcpp::Parameter& param : parameters) {

            if (param.get_name() == "is_running") {
                this->is_running = param.as_bool();
                if (this->is_running)
                {
                    cmd_sts1000.ucWorkStatus &= ~(1<<3); //将ucWorkStatus的 bit3 置 0
                    updateCheckSum(&cmd_sts1000);
                }
                else{
                    cmd_sts1000.ucWorkStatus |= 1<<3;//将ucWorkStatus的 bit3 置 1
                    updateCheckSum(&cmd_sts1000);
                }
                RCLCPP_INFO(this->get_logger(), "is_running is %d",this->is_running);
            }
            else if (param.get_name() == "scanRange") {
                if( param.as_double() < 2 || param.as_double() > 40)
                {
                    RCLCPP_INFO(this->get_logger(), "scanRange must be 3~40!");
                    result.successful = false;
                    result.reason = "scanRange must be 3~40!";
                    return result;
                }
                this->scanRange = param.as_double();
                cmd_sts1000.ucRange = this->scanRange;
                updateCheckSum(&cmd_sts1000);
                RCLCPP_INFO(this->get_logger(), "scanRange is %f",this->scanRange);
            }
            else if (param.get_name() == "startGain") {
                if( param.as_int() < 2 || param.as_int() > 40)
                {
                    RCLCPP_INFO(this->get_logger(), "startGain must be 21 挡 {0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40}!");
                    result.successful = false;
                    result.reason = "startGain must be 21 挡 {0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40}!";
                    return result;
                }
                this->startGain = param.as_int();
                cmd_sts1000.ucStartGain = (cmd_sts1000.ucStartGain & 0xe0) + (this->startGain)/2; // 前3位是声扩散系数
                updateCheckSum(&cmd_sts1000);
                RCLCPP_INFO(this->get_logger(), "startGain is %d",this->startGain);
            }
            else if (param.get_name() == "diffusion") {
                if( param.as_int() < 0 || param.as_int() > 8)
                {
                    RCLCPP_INFO(this->get_logger(), "diffusion must be {0,1,2,3,4,5,6,7}!");
                    result.successful = false;
                    result.reason = "diffusion must be {0,1,2,3,4,5,6,7}!";
                    return result;
                }
                this->diffusion = param.as_int();
                cmd_sts1000.ucStartGain = (cmd_sts1000.ucStartGain & 0x1F) + ((this->diffusion)<<5); // 前3位是声扩散系数
                updateCheckSum(&cmd_sts1000);
                RCLCPP_INFO(this->get_logger(), "diffusion is %d",this->diffusion);
            }
            
        }

        return result;
    }

};


int main(int argc, char **argv)
{
    /* 初始化rclcpp  */
    rclcpp::init(argc, argv);
    /*产生一个node_01的节点*/
    auto node = std::make_shared<sts1000_dc>("sts1000_dc");
    /* 运行节点，并检测退出信号 Ctrl+C*/
    rclcpp::spin(node);
    /* 停止运行 */
    rclcpp::shutdown();
    return 0;
}
