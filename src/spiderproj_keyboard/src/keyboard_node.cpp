#include <rclcpp/rclcpp.hpp>
#include <spiderproj_msgs/msg/joy_data.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <vector>

class KeyboardNode : public rclcpp::Node {
public:
    KeyboardNode() : Node("keyboard_node") {
        publisher_ = this->create_publisher<spiderproj_msgs::msg::JoyData>("joy", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&KeyboardNode::read_keyboard_input, this)
        );
        configure_terminal();
        buttons_.resize(5, 0);  // Initialize 5 buttons to 0
    }

    ~KeyboardNode() {
        reset_terminal();
    }

private:
    rclcpp::Publisher<spiderproj_msgs::msg::JoyData>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    struct termios old_terminal_;
    std::vector<int> buttons_;

    void configure_terminal() {
        struct termios new_terminal;
        tcgetattr(STDIN_FILENO, &old_terminal_);
        new_terminal = old_terminal_;
        new_terminal.c_lflag &= ~(ICANON | ECHO);  // Disable line buffering and echo
        tcsetattr(STDIN_FILENO, TCSANOW, &new_terminal);
        fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK); // Non-blocking
    }

    void reset_terminal() {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal_);
    }

    void read_keyboard_input() {
        char key = 0;
        int bytes_read = read(STDIN_FILENO, &key, 1);  // Use read() instead of getchar()

        auto joy_message = spiderproj_msgs::msg::JoyData();

        // Default joystick axes
        joy_message.axes = {0.0f, 0.0f, 0.0f, 0.0f};

        if (bytes_read > 0) {
            RCLCPP_INFO(this->get_logger(), "Key pressed: %c", key);

            // Handle joystick-like axes
            switch (key) {
                case 'w': joy_message.axes[1] = 1.0f; break;   // Forward
                case 's': joy_message.axes[1] = -1.0f; break;  // Backward
                case 'a': joy_message.axes[0] = -1.0f; break;  // Left
                case 'd': joy_message.axes[0] = 1.0f; break;   // Right

                case 'i': joy_message.axes[3] = 0.7f; break;   // Forward
                case 'k': joy_message.axes[3] = -0.7f; break;  // Backward
                case 'j': joy_message.axes[2] = 0.7f; break;  // Left
                case 'l': joy_message.axes[2] = -0.7f; break;   // Right

                // Handle toggle switches
                case 'b': toggle_button(0); break;
                case '2': toggle_button(1); break;
                case '3': toggle_button(2); break;
                case '4': toggle_button(3); break;
                case '1': toggle_button(4); break;
                // Quit the program
                case 'q': rclcpp::shutdown(); return;
            }
        }

        // Assign current button states
        joy_message.buttons = buttons_;

        publisher_->publish(joy_message);
    }

    void toggle_button(int index) {
        buttons_[index] = !buttons_[index];  // Toggle between 0 and 1
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardNode>());
    rclcpp::shutdown();
    return 0;
}
