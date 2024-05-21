#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("my_node")
    {
        // 创建一个参数回调函数并绑定到类的成员函数
        auto callback = std::bind(&MyNode::onParameterSet, this, std::placeholders::_1);

        // 将参数回调函数添加到节点的参数服务中
        this->declare_parameter("my_parameter", rclcpp::ParameterValue(0));
        on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(callback);
    }

private:

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr  on_set_parameters_callback_handle_;
    // 参数设置回调函数
    rcl_interfaces::msg::SetParametersResult onParameterSet(const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& parameter : parameters)
        {
            if (parameter.get_name() == "my_parameter")
            {
                // 在此处执行参数设置时的自定义操作
                RCLCPP_INFO(this->get_logger(), "Parameter 'my_parameter' was set to: %d", parameter.as_int());
            }
        }

        return result;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}