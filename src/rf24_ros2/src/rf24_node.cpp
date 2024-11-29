#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <RF24/RF24.h>

class RF24Node : public rclcpp::Node {
public:
    RF24Node(int CE, int CSN) : Node("rf24_node"), radio(CE, CSN) { // CE and CSN pins
        if (!radio.begin()) {
            RCLCPP_ERROR(this->get_logger(), "Radio initialization failed");
            return;
        }

        init_radio();

        publisher_ = this->create_publisher<std_msgs::msg::String>("rf24_data", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&RF24Node::read_data, this));
    }

private:
    struct RC_Data_Package {
        uint8_t joy1_X;
        uint8_t joy1_Y;
        uint8_t joy2_X;
        uint8_t joy2_Y;
        uint8_t toggle1;
        uint8_t toggle2;
        uint8_t toggle3;
        uint8_t toggle4;
    };

    RC_Data_Package received_data;

    void init_radio() {
        radio.begin();
        radio.openReadingPipe(1, address[0]); 
        radio.setPALevel(RF24_PA_HIGH);
        radio.setDataRate(RF24_1MBPS);
        radio.setPayloadSize(sizeof(received_data));
        radio.setChannel(112);
        radio.startListening();
        RCLCPP_INFO(this->get_logger(), "RF24 radio initialized and listening!");
    }

    void read_data() {
        if (radio.available()) {
            radio.read(&received_data, sizeof(RC_Data_Package));

            // Corrected RCLCPP_INFO with format argument
            RCLCPP_INFO(this->get_logger(), "Received JoyX: %d", received_data.joy1_X);
            RCLCPP_INFO(this->get_logger(), "Received JoyY: %d", received_data.joy1_Y);
        }
        else {
            //RCLCPP_INFO(this->get_logger(), "No data available");
        }
    }

    uint8_t address[2][6] = {"1Node", "2Node"};
    RF24 radio;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RF24Node>(25,0));
    rclcpp::shutdown();
    return 0;
}
