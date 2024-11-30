#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joy.hpp"
#include <RF24/RF24.h>

class RF24Node : public rclcpp::Node {
public:
    RF24Node(int CE, int CSN) : Node("rf24_node"), radio(CE, CSN) {
        RCLCPP_INFO(this->get_logger(), "Attempting to initialize radio on CE: %d, CSN: %d", CE, CSN);
        
        if (!radio.begin()) {
            RCLCPP_ERROR(this->get_logger(), "Radio initialization FAILED - check hardware connections!");
            throw std::runtime_error("Radio initialization failed");
        }

        try {
            init_radio();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Radio initialization error: %s", e.what());
            throw;
        }

        publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("joy", 1); 
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
        bool toggle1;
        bool toggle2;
        bool toggle3;
        bool toggle4;
    } __attribute__((packed));  // Ensure tight packing
    
    RC_Data_Package received_data;

    void init_radio() {
        radio.setPALevel(RF24_PA_HIGH);
        radio.setDataRate(RF24_1MBPS);
        radio.setPayloadSize(sizeof(received_data));
        radio.setChannel(112);

        // Print radio configuration for debugging
        RCLCPP_INFO(this->get_logger(), "Radio Configuration:");
        RCLCPP_INFO(this->get_logger(), "Payload Size: %d", radio.getPayloadSize());
        RCLCPP_INFO(this->get_logger(), "Channel: %d", radio.getChannel());

        radio.openReadingPipe(1, address[0]); 
        radio.startListening();
    

        RCLCPP_INFO(this->get_logger(), "RF24 radio initialized and listening successfully!");
    }

    float apply_deadzone(float value, float deadzone) {
        if (value < deadzone && value > -deadzone) {
            return 0.0f;  // within deadzone, return 0
        }
        return value; // outside deadzone, return original value
    }

    void read_data() {
        if (radio.available()) {
            size_t payload_size = radio.getDynamicPayloadSize();
            //RCLCPP_INFO(this->get_logger(), "Received payload size: %zu", payload_size);

            if (payload_size != sizeof(RC_Data_Package)) {
                RCLCPP_WARN(this->get_logger(), 
                    "Payload size mismatch! Expected %zu, got %zu", 
                    sizeof(RC_Data_Package), payload_size);
                return;
            }

            radio.read(&received_data, sizeof(RC_Data_Package));
            auto joy_message = sensor_msgs::msg::Joy();

            joy_message.axes = {
                apply_deadzone(static_cast<float>(received_data.joy1_X) / 255.0f * 2.0f - 1.0f, 0.2f),
                apply_deadzone(static_cast<float>(received_data.joy1_Y) / 255.0f * 2.0f - 1.0f, 0.2f),
                apply_deadzone(static_cast<float>(received_data.joy2_X) / 255.0f * 2.0f - 1.0f, 0.2f),
                apply_deadzone(static_cast<float>(received_data.joy2_Y) / 255.0f * 2.0f - 1.0f, 0.2f)
            };

            joy_message.buttons = {
                received_data.toggle1 ? 0 : 1,
                received_data.toggle2 ? 0 : 1,
                received_data.toggle3 ? 0 : 1,
                received_data.toggle4 ? 0 : 1
            };

            joy_message.header.stamp = this->get_clock()->now();
            joy_message.header.frame_id = "rf24_joystick";

            publisher_->publish(joy_message);

            //RCLCPP_INFO(this->get_logger(), 
             //   "Published Joy message - Axes: [%f, %f, %f, %f], Buttons: [%d, %d, %d, %d]",
             //   joy_message.axes[0], joy_message.axes[1],
             //   joy_message.axes[2], joy_message.axes[3],
             //   joy_message.buttons[0], joy_message.buttons[1],
             //   joy_message.buttons[2], joy_message.buttons[3]);
        }
        else {
            // Optional: Add a less frequent "no data" log to prevent log spam
            static int no_data_count = 0;
            if (++no_data_count % 20 == 0) {
                RCLCPP_WARN(this->get_logger(), "No RF24 data received for %d cycles", no_data_count);
            }
        }
    }

    uint8_t address[2][6] = {"1Node", "2Node"};
    RF24 radio;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<RF24Node>(25, 0));
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Unhandled exception: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}