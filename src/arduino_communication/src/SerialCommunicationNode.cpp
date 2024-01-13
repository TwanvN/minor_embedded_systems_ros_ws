#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int8.hpp>

#include <boost/asio.hpp>

using std::placeholders::_1;

class SerialCommunication : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr newTargetPositionSubscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr newAxisPositionSubscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr homeAxisSubscriber;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr dataGatheringSubscriber;

    boost::asio::io_service io;
    boost::asio::serial_port serial;

    void newTargetPositionCallback(const std_msgs::msg::Int16MultiArray &msg) {
        RCLCPP_INFO(this->get_logger(), std::to_string(msg.data[0]).c_str());

        std::string commandString = "newTargetPos " + std::to_string(msg.data[0]) + "," + std::to_string(msg.data[1]);
        RCLCPP_INFO(this->get_logger(), commandString.c_str());

        boost::asio::write(this->serial, boost::asio::buffer(commandString));
    }

    void newAxisPositionCallback(const std_msgs::msg::String &msg) {
        RCLCPP_INFO(this->get_logger(), msg.data.c_str());

        std::string charString(1, msg.data.at(0));
        std::string commandString = "newAxisPos " + charString + " " + msg.data.substr(2, msg.data.size());
        RCLCPP_INFO(this->get_logger(), commandString.c_str());

        boost::asio::write(this->serial, boost::asio::buffer(commandString));
    }

    void homeAxisCallback(const std_msgs::msg::String &msg) {
        RCLCPP_INFO(this->get_logger(), msg.data.c_str());

        std::string commandString = "homeAxis " + msg.data;
        RCLCPP_INFO(this->get_logger(), commandString.c_str());

        boost::asio::write(this->serial, boost::asio::buffer(commandString));
    }

    void gatherDataCallback(const std_msgs::msg::Int8 &msg) {
        RCLCPP_INFO(this->get_logger(), std::to_string(msg.data).c_str());
    }

public:
    SerialCommunication() : Node("serial_communication_node"), io(), serial(io)
    {
        RCLCPP_INFO(this->get_logger(), "New serial node started");

        this->newTargetPositionSubscriber = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "/target_position",
            10,
            std::bind(&SerialCommunication::newTargetPositionCallback, this, _1));

        this->newAxisPositionSubscriber = this->create_subscription<std_msgs::msg::String>(
            "/axis_target",
            10,
            std::bind(&SerialCommunication::newAxisPositionCallback, this, _1));

        this->homeAxisSubscriber = this->create_subscription<std_msgs::msg::String>(
            "/home_axis",
            10,
            std::bind(&SerialCommunication::homeAxisCallback, this, _1));

        this->dataGatheringSubscriber = this->create_subscription<std_msgs::msg::Int8>(
            "/data_gathering_command",
            10,
            std::bind(&SerialCommunication::gatherDataCallback, this, _1));

        // this->serial.open("/dev/arduino");
        // this->serial.set_option(boost::asio::serial_port_base::baud_rate(115200));
    };

    ~SerialCommunication() {}
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialCommunication>());
    rclcpp::shutdown();

    return 0;
}