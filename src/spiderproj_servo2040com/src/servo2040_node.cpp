#include <rclcpp/rclcpp.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <string>
#include <vector>
#include <spiderproj_msgs/msg/joint_angles.hpp>
#include <spiderproj_msgs/msg/servo2040_data.hpp>
#include <spiderproj_msgs/msg/relay.hpp>

class Servo2040Node : public rclcpp::Node {
public:
    Servo2040Node() : Node("servo2040_node") {
        serial_port = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_port < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("SERVO2040"), "Failed to open port");
            rclcpp::shutdown();
        }

        relay_sub_ = this->create_subscription<spiderproj_msgs::msg::Relay>(
        "/servo2040/relay", 1, std::bind(&Servo2040Node::relay_callback, this, std::placeholders::_1));

        joint_angles_sub_ = this->create_subscription<spiderproj_msgs::msg::JointAngles>(
        "servo2040/joint_angles", 1, std::bind(&Servo2040Node::publish_joints_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<spiderproj_msgs::msg::Servo2040Data>("servo2040Data", 1); 

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&Servo2040Node::servo2040_timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Servo2040 Node initialized.");
    }

    ~Servo2040Node() {
        if (serial_port >= 0) {
            close(serial_port);
        }
    }

private:
    int serial_port;
    bool power_on;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<spiderproj_msgs::msg::Servo2040Data>::SharedPtr publisher_;
    rclcpp::Subscription<spiderproj_msgs::msg::Relay>::SharedPtr relay_sub_;
    rclcpp::Subscription<spiderproj_msgs::msg::JointAngles>::SharedPtr joint_angles_sub_;

    typedef enum {
        SERVO1, SERVO2, SERVO3, SERVO4, SERVO5, SERVO6, 
        SERVO7, SERVO8, SERVO9, SERVO10, SERVO11, SERVO12, 
        SERVO13, SERVO14, SERVO15, SERVO16, SERVO17, SERVO18,
        TS1, TS2, TS3, TS4, TS5, TS6, 
        CURR, VOLT, RELAY, A1, A2, cmdPin_num
    } cmdPins;

    void servo2040_timer_callback() {
        //read_sensors(serial_port, TS1 ,2);
    }

    void publish_joints_callback(const spiderproj_msgs::msg::JointAngles::SharedPtr joint_angles_msg) {
        
    }

    void relay_callback(const spiderproj_msgs::msg::Relay::SharedPtr relay_msg) {
        if (relay_msg->enabled) {
            toggleRelayOn(serial_port);
        } else {
            toggleRelayOff(serial_port);
        }
        
        RCLCPP_INFO(this->get_logger(), "Relay toggled");
    }

    void toggleRelayOn(int serial_port) {
        unsigned char txbuff[6];
        uint16_t value = 1;
        txbuff[0] = 0xD3;                      // Command to set values
        txbuff[1] = RELAY;                     // Start index (e.g., SERVO18)
        txbuff[2] = 1;                        // Number of values to send
        txbuff[3] = value & 0x7F;                 // Lower 7 bits of value
        txbuff[4] = (value >> 7) & 0x7F;          // Upper 7 bits of value
        txbuff[5] = '\n';      

        write(serial_port, txbuff, sizeof(txbuff));
    }   

    void toggleRelayOff(int serial_port) {
        unsigned char txbuff[6];
        uint16_t value = 0;
        txbuff[0] = 0xD3;                      // Command to set values
        txbuff[1] = RELAY;                     // Start index (e.g., SERVO18)
        txbuff[2] = 1;                        // Number of values to send
        txbuff[3] = value & 0x7F;                 // Lower 7 bits of value
        txbuff[4] = (value >> 7) & 0x7F;          // Upper 7 bits of value
        txbuff[5] = '\n';      

        write(serial_port, txbuff, sizeof(txbuff));
    }

    void send_command(int serial_port, uint8_t startIdx, uint8_t count, uint16_t* values) {
        std::vector<uint8_t> txbuff;
        txbuff.reserve(3 + count * 2 + 1);

        txbuff.push_back(0xD3);
        txbuff.push_back(startIdx);
        txbuff.push_back(count);

        for (uint8_t i = 0; i < count; i++) {
            txbuff.push_back(values[i] & 0x7F);          // Lower 7 bits
            txbuff.push_back((values[i] >> 7) & 0x7F);   // Upper 7 bits
        }

        ssize_t bytes_written = write(serial_port, txbuff.data(), txbuff.size());
        if (bytes_written < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port");
        }
    }

    void read_sensors(int serial_port, uint8_t startIdx, uint8_t count) {
        unsigned char txbuff[3];
        txbuff[0] = 0xC7;       // Command byte
        txbuff[1] = startIdx;   // Start index
        txbuff[2] = count;      // Number of values to read

        // Send the GET command
        if (write(serial_port, txbuff, sizeof(txbuff)) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("SERVO2040"), "Failed to send GET command: %s", strerror(errno));
            return;
        }

        // Prepare for response handling
        bool ok = true;
        int state = -1;
        int idx = startIdx;
        int count_remaining = count;
        unsigned char c;
        uint16_t value;
        unsigned char rxbuff[256]; // Buffer to collect entire packet (ensure it’s large enough)
        unsigned char *p = rxbuff;

        while (ok) {
            int res = read(serial_port, &c, 1);
            if (res == -1) {
                RCLCPP_ERROR(rclcpp::get_logger("SERVO2040"), "Read from serial failed: %s", strerror(errno));
                return;
            } else if (res == 1) {
                if (state == -1 && ((c & 0x80) == 0)) {
                    RCLCPP_WARN(rclcpp::get_logger("SERVO2040"), "Unexpected byte 0x%02X outside of a command packet", c);
                } else {
                    if ((c & 0x80) != 0) {  // Command byte detected
                        state = 0;
                        count_remaining = count;
                        idx = startIdx;
                        p = rxbuff;
                        *p++ = c;
                    } else {
                        switch (state) {
                        case 0: // Start index
                            if (c > 26) {
                                RCLCPP_ERROR(rclcpp::get_logger("SERVO2040"), "Index out of range: %d", c);
                                return;
                            }
                            idx = c;
                            state++;
                            break;
                        case 1: // Count of values to read
                            count_remaining = c;
                            state++;
                            break;
                        case 2: // First byte of 16-bit value
                            value = c & 0x7F;
                            state++;
                            break;
                        case 3: // Second byte of 16-bit value
                            value |= (c & 0x7F) << 7;
                            if (idx == TS1) { // Assuming 24 is the index for current
                                float TS1 = ((float)value/1000); // Example scalingstrerror(errno)
                                RCLCPP_INFO(rclcpp::get_logger("SERVO2040"), "TS1 (index %d): %6.3f", idx, TS1);
                            }
                            if (idx == TS2) { // Assuming 24 is the index for current
                                float TS2 = ((float)value/1000); // Example scalingstrerror(errno)
                                RCLCPP_INFO(rclcpp::get_logger("SERVO2040"), "TS2 (index %d): %6.3f", idx, TS2);
                            }
                            if (idx == TS3) { // Assuming 24 is the index for current
                                float TS3 = ((float)value/1000); // Example scalingstrerror(errno)
                                RCLCPP_INFO(rclcpp::get_logger("SERVO2040"), "TS3 (index %d): %6.3f", idx, TS3);
                            }
                            if (idx == TS4) { // Assuming 24 is the index for current
                                float TS4 = ((float)value/1000); // Example scalingstrerror(errno)
                                RCLCPP_INFO(rclcpp::get_logger("SERVO2040"), "TS4 (index %d): %6.3f", idx, TS4);
                            }
                            if (idx == TS5) { // Assuming 24 is the index for current
                                float TS5 = ((float)value/1000); // Example scalingstrerror(errno)
                                RCLCPP_INFO(rclcpp::get_logger("SERVO2040"), "TS5 (index %d): %6.3f", idx, TS5);
                            }
                            if (idx == TS6) { // Assuming 24 is the index for current
                                float TS6 = ((float)value/1000); // Example scalingstrerror(errno)
                                RCLCPP_INFO(rclcpp::get_logger("SERVO2040"), "TS6 (index %d): %6.3f", idx, TS6);
                            }
                            if (idx == CURR) { // Assuming 24 is the index for current
                                float current = ((float)value - 512.0) * 0.0814; // Example scaling
                                RCLCPP_INFO(rclcpp::get_logger("SERVO2040"), "Current (index %d): %6.3f A", idx, current);
                            }
                            if (idx == VOLT) { // Assume 25 is the index for voltage
                                float voltage = (float)value / 310.303; // Example scaling
                                RCLCPP_INFO(rclcpp::get_logger("SERVO2040"), "Voltage (index %d): %6.3f V", idx, voltage);
                            }
                            idx++;
                            count_remaining--;

                            if (count_remaining == 0) {
                                ok = false; // Exit loop after last value
                            } else {
                                state = 2; // Continue reading the next 16-bit value
                            }
                            break;
                        }
                        *p++ = c; // Store the byte in the buffer
                    }
                }
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Servo2040Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
