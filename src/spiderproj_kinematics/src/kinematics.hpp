#pragma once

#include <rclcpp/rclcpp.hpp>
#include <kdl/frames.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_trap.hpp>

#include <spiderproj_msgs/msg/body_control.hpp>
#include <spiderproj_msgs/msg/hexapod_motion.hpp>
#include <spiderproj_msgs/msg/meta_commands.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <cmath>
#include <memory>
#include <stdexcept>

#include <vector>
#include "leg.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#define DEFAULT_TIMESTEP 0.02
#define TRAJ_RES 60
#define MAX_SWING_VEL 1

class Kinematics : public rclcpp::Node {
public:
    Kinematics();

private:
    typedef enum {
    SERVO1, SERVO2, SERVO3, SERVO4, SERVO5, SERVO6, 
    SERVO7, SERVO8, SERVO9, SERVO10, SERVO11, SERVO12, 
    SERVO13, SERVO14, SERVO15, SERVO16, SERVO17, SERVO18,
    TS1, TS2, TS3, TS4, TS5, TS6, 
    CURR, VOLT, RELAY, A1, A2, cmdPin_num
    } cmdPins;
    
    std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client_;
    void set_params_from_global_param_server(std::shared_future<std::vector<rclcpp::Parameter>>);

    int serial_port_fd_;
    int num_legs;
    int num_joints;
    std::vector<std::string> leg_suffixes;
    float coxa_length, femur_length, tibia_length;
    std::vector<Leg> legs;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr      joint_states_pub_;
    sensor_msgs::msg::JointState joint_states_msg;

    rclcpp::Subscription<spiderproj_msgs::msg::BodyControl>::SharedPtr      body_control_sub_;
    rclcpp::Subscription<spiderproj_msgs::msg::HexapodMotion>::SharedPtr    hexapod_motion_sub_;
    rclcpp::Subscription<spiderproj_msgs::msg::MetaCommands>::SharedPtr     meta_cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    void meta_cmd_callback(const spiderproj_msgs::msg::MetaCommands::SharedPtr);
    void hexapod_motion_callback(const spiderproj_msgs::msg::HexapodMotion::SharedPtr);
    void body_control_callback(const spiderproj_msgs::msg::BodyControl::SharedPtr);
    void timer_callback();

    int setup_serial_port(const std::string& port_name);
    uint16_t convert_angle_to_value(int servoNum ,double angle);
    void toggleRelayOn(int serial_port);
    void toggleRelayOff(int serial_port);
    void send_command(int serial_port, uint8_t startIdx, uint8_t count, uint16_t* values);

    void publish_joint_states();
    void checkFeetSensors(int serial_port, uint8_t startIdx, uint8_t count);

    bool is_power_on;
    bool is_ready;
    bool imu_status;
    bool is_dancing;

    bool is_sitting;

    KDL::Frame body_pose;
    KDL::Twist body_twist;

    KDL::Twist motion_twist; // motion velocity twist in virtual body frame
    int gait_type = 0; // 0 for wave, 1 for ripple, 2 for tripod
    bool is_omnidirectional; // false for streamlined, true or omnidirectional  

    double total_current;
    std::vector<bool> feet_states;

    bool params_set_flag;

    double max_lin_velocity;
    double max_ang_velocity;

    double transfer_height;

    bool is_in_null_pos;

    void sit_down();
    void stand_up();

    void move_robot();
    void move_wave();
    void move_ripple();
    void move_tripod();
    void move_triple_gait();

    void move_body();

    void center_legs();


};