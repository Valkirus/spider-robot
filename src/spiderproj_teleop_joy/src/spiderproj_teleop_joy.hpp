#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <spiderproj_msgs/msg/body_control.hpp>
#include <spiderproj_msgs/msg/hexapod_motion.hpp>
#include <spiderproj_msgs/msg/meta_commands.hpp>

#include <Eigen/Geometry>
#include <cmath>

#define DEFAULT_TIMESTEP 0.1

class TeleopJoy : public rclcpp::Node {
public:
    TeleopJoy();

private:

    std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client_;
    void set_params_from_global_param_server(std::shared_future<std::vector<rclcpp::Parameter>>);

    rclcpp::Publisher<spiderproj_msgs::msg::BodyControl>::SharedPtr   body_control_pub_;
    rclcpp::Publisher<spiderproj_msgs::msg::HexapodMotion>::SharedPtr hexapod_motion_pub_;
    rclcpp::Publisher<spiderproj_msgs::msg::MetaCommands>::SharedPtr  meta_cmd_pub_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    spiderproj_msgs::msg::BodyControl   body_control;
    spiderproj_msgs::msg::HexapodMotion hexapod_motion;
    spiderproj_msgs::msg::MetaCommands  meta_cmd;
 
    bool servo_power_flag;
    bool start_flag;
    bool imu_flag;
    bool is_dancing;

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr);

    // control methods
    void servoPowerOn();
    void servoPowerOff();
    void scanForPowerOn();
    void startStop();
    void imuOnOff();
    void setOmnidirectional();
    void setStreamlined();
    void setGaitWave();
    void setGaitRipple();
    void setGaitTripod();

    void sendHexapodMotionData(const sensor_msgs::msg::Joy::SharedPtr);

    void setBodyPose(double, double, int, bool);
    void setBodyTwist(double);

    void setStreamTwist(double, double);
    void setOmniTwist(double, double);

    // joy control flags
    bool servo_power_off_command_flag;
    bool servo_power_on_command_flag;
    bool start_command_flag;
    bool imu_on_off_command_flag;
    bool omnidirectional_command_flag;
    bool wave_gait_command_flag;
    bool ripple_gait_command_flag;
    bool tripod_gait_command_flag;

    // set joy control flags
    void setJoyFlags(const sensor_msgs::msg::Joy::SharedPtr&);

    std::chrono::time_point<std::chrono::steady_clock> startTime;

    double max_lin_velocity;
    double max_ang_velocity;
    double max_body_y_euler;
    double max_body_z_euler;
    double max_body_x;
    double max_body_y;
    double max_body_velocity_z;
    double press_to_power_on_duration;
};