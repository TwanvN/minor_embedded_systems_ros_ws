#include "rclcpp/rclcpp.hpp"

#include <boost/asio.hpp>

class SerialCommunication : public rclcpp::Node
{
    private:
        
    public:
        SerialCommunication() : Node("serial_communication_node") {
            RCLCPP_INFO(this->get_logger(), "New serial node started");
        };

        ~SerialCommunication() {}
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialCommunication>());
    rclcpp::shutdown();

    return 0;
}