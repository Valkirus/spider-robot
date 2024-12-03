#include "spiderproj_teleop_joy.hpp"

using namespace std::chrono_literals;

TeleopJoy::TeleopJoy() 
: Node("spiderproj_teleop_joy") {
    press_to_power_on_duration = 3.0;
    
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/global_parameter_server");
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for global parameter server to be available...");
    parameters_client_->wait_for_service();

    auto parameters_future = parameters_client_->get_parameters({"max_lin_velocity", 
                                                                "max_ang_velocity",
                                                                "max_body_y_euler",
                                                                "max_body_z_euler",
                                                                "max_body_x",
                                                                "max_body_y",
                                                                "max_body_velocity_z"},
            std::bind(&TeleopJoy::set_params_from_global_param_server, this, std::placeholders::_1));

    servo_power_flag     = false;
    start_flag           = false;
    imu_flag             = false;

    startTime = std::chrono::steady_clock::now();

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 1, std::bind(&TeleopJoy::joyCallback, this, std::placeholders::_1));

    body_control_pub_   = create_publisher<spiderproj_msgs::msg::BodyControl>("/teleop/body_control", 1);           // body pose
    hexapod_motion_pub_ = create_publisher<spiderproj_msgs::msg::HexapodMotion>("/teleop/hexapod_motion", 1); // low lever power, stand up, sit down stuff
    meta_cmd_pub_       = create_publisher<spiderproj_msgs::msg::MetaCommands>("/teleop/meta_cmd", 1);        // twist  of {b} or direction of {b}, motion type 
}
void TeleopJoy::set_params_from_global_param_server(std::shared_future<std::vector<rclcpp::Parameter>> future) {
    // get params from global param server
    max_lin_velocity    = future.get()[0].as_double();
    max_ang_velocity    = future.get()[1].as_double();
    max_body_y_euler    = future.get()[2].as_double();
    max_body_z_euler    = future.get()[3].as_double();
    max_body_x          = future.get()[4].as_double();
    max_body_y          = future.get()[5].as_double();
    max_body_velocity_z = future.get()[6].as_double();
}

// sets control flags depending on joy input
void TeleopJoy::setJoyFlags(const sensor_msgs::msg::Joy::SharedPtr &joy){
    servo_power_flag             = joy->buttons[4];
    start_command_flag           = joy->buttons[0]; // press
    omnidirectional_command_flag = joy->buttons[1]; // hold
    is_dancing                   = joy->buttons[3];
}

// DEFINITIONS OF CONTROL METHODS
void TeleopJoy::servoPowerOn() {
    servo_power_flag = true;
    meta_cmd.power = true;
    meta_cmd_pub_->publish(meta_cmd);
    rclcpp::sleep_for(1000ms);
}

void TeleopJoy::servoPowerOff() {
    servo_power_flag = false;
    meta_cmd.power = false;
    start_flag = false;
    meta_cmd.is_ready = false;
    imu_flag = false;
    meta_cmd.imu_status = false;
    meta_cmd_pub_->publish(meta_cmd);
    rclcpp::sleep_for(1000ms);
}

void TeleopJoy::scanForPowerOn() {
    auto currentTime = std::chrono::steady_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime);

    if (duration.count() >= press_to_power_on_duration) 
        servoPowerOn();
}

//stand up or sit down
void TeleopJoy:: startStop() {
    start_flag ^= 1;
    meta_cmd.is_ready ^= 1;
    if (!start_flag) {
        imu_flag = false;
        meta_cmd.imu_status =false;
    }
    meta_cmd_pub_->publish(meta_cmd);
    rclcpp::sleep_for(1000ms);
}

void TeleopJoy::imuOnOff() {
    imu_flag ^= 1;
    meta_cmd.imu_status ^= 1;
    meta_cmd_pub_->publish(meta_cmd);
    rclcpp::sleep_for(1000ms);
}

// set motion type (omni vs stream)
void TeleopJoy::setOmnidirectional() {
    hexapod_motion.motion_type = spiderproj_msgs::msg::HexapodMotion::MOTION_OMNI;
    hexapod_motion_pub_->publish(hexapod_motion);
    // sleep here ???
}

void TeleopJoy::setStreamlined() {
    hexapod_motion.motion_type = spiderproj_msgs::msg::HexapodMotion::MOTION_STREAM;
    hexapod_motion_pub_->publish(hexapod_motion);
    // sleep here ??
}

// set gait (wave, ripple, tripod) unused
void TeleopJoy::setGaitWave() {
    hexapod_motion.gait= spiderproj_msgs::msg::HexapodMotion::WAVE_GAIT;
    hexapod_motion_pub_->publish(hexapod_motion);
}

void TeleopJoy::setGaitRipple() {
    hexapod_motion.gait= spiderproj_msgs::msg::HexapodMotion::RIPPLE_GAIT;
    hexapod_motion_pub_->publish(hexapod_motion);
}

void TeleopJoy::setGaitTripod() {
    hexapod_motion.gait= spiderproj_msgs::msg::HexapodMotion::TRIPOD_GAIT;
    hexapod_motion_pub_->publish(hexapod_motion);
}

// find hexapod motion data
// find body control
void TeleopJoy::setBodyPose(double s_xR, double s_yR, int R_button_state, bool is_dancing) {
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sine interpolated value : %f", sine_interpolated_value);
    if (is_dancing) {
        static double progress = 0.0;
        double frequency = 1.0;  //oscillations per cycle

        double amplitude_y = s_xR * max_body_y_euler;
        double phase_offset_y = (M_PI / 2); 

        double amplitude_x = s_yR * max_body_z_euler;
        double phase_offset_x = (M_PI / 2); 

        // Calculate sine wave value Y-axis
        double sine_interpolated_value_y = amplitude_y * sin(2.0 * M_PI * frequency * progress);
        double sine_interpolated_value_y_delayed = amplitude_y * sin(2.0 * M_PI * frequency * progress + phase_offset_y);

        //x-axis
        double sine_interpolated_value_x = amplitude_x * sin(2.0 * M_PI * frequency * progress);
        double sine_interpolated_value_x_delayed = amplitude_x * sin(2.0 * M_PI * frequency * progress + phase_offset_x);

        progress += 0.04; 
        if (progress >= 1.0) {
            progress -= 1.0;
        }

        body_control.body_pose_euler_angles.euler_angles.z = 0.0;
        body_control.body_pose_euler_angles.euler_angles.y = sine_interpolated_value_y_delayed;
        body_control.body_pose_euler_angles.euler_angles.x = sine_interpolated_value_x_delayed;

        body_control.body_pose_euler_angles.position.x = sine_interpolated_value_y * 0.14;
        body_control.body_pose_euler_angles.position.y = -sine_interpolated_value_x * 0.14;
        body_control.body_pose_euler_angles.position.z = 0.0;
    }
    else if (R_button_state == 1.0) {
        body_control.body_pose_euler_angles.euler_angles.z = 0.0;
        body_control.body_pose_euler_angles.euler_angles.y = 0.0;
        body_control.body_pose_euler_angles.euler_angles.x = 0.0;

        body_control.body_pose_euler_angles.position.x = -s_xR * max_body_x;
        body_control.body_pose_euler_angles.position.y = s_yR * max_body_y;
        body_control.body_pose_euler_angles.position.z = 0.0;
    } else {
        body_control.body_pose_euler_angles.euler_angles.z = s_yR * max_body_z_euler;
        body_control.body_pose_euler_angles.euler_angles.y = s_xR * max_body_y_euler;
        body_control.body_pose_euler_angles.euler_angles.x = 0.0;

        body_control.body_pose_euler_angles.position.x = 0.0;
        body_control.body_pose_euler_angles.position.y = 0.0;
        body_control.body_pose_euler_angles.position.z = 0.0;
    }
}

// ZYX rotation, Quaterions in JPL convention
void TeleopJoy::setBodyTwist(double z_relative) {
    body_control.body_twist.angular.x = 0.0;
    body_control.body_twist.angular.y = 0.0;
    body_control.body_twist.angular.z = 0.0;

    body_control.body_twist.linear.x  = 0.0;
    body_control.body_twist.linear.y  = 0.0;
    body_control.body_twist.linear.z = z_relative * max_body_velocity_z;
}

// find motion twist
void TeleopJoy::setStreamTwist(double s_xL, double s_yL){
    hexapod_motion.motion_twist.angular.x = 0.0;
    hexapod_motion.motion_twist.angular.y = 0.0;
    hexapod_motion.motion_twist.angular.z = -s_yL * max_ang_velocity;

    hexapod_motion.motion_twist.linear.x  = s_xL * max_lin_velocity;
    hexapod_motion.motion_twist.linear.y  = 0.0;
    hexapod_motion.motion_twist.linear.z  = 0.0;
}

void TeleopJoy::setOmniTwist(double s_xL, double s_yL) {
    hexapod_motion.motion_twist.angular.x = 0.0;
    hexapod_motion.motion_twist.angular.y = 0.0;
    hexapod_motion.motion_twist.angular.z = 0.0;

    hexapod_motion.motion_twist.linear.x  = s_xL * max_lin_velocity;
    hexapod_motion.motion_twist.linear.y  = -s_yL * max_lin_velocity;
    hexapod_motion.motion_twist.linear.z  = 0.0;
}

void TeleopJoy::sendHexapodMotionData(const sensor_msgs::msg::Joy::SharedPtr joy) {
    setBodyPose(joy->axes[3],
                joy->axes[2],
                joy->buttons[2],
                is_dancing);

    //setBodyTwist(joy->axes[axis_cross_x]);

    omnidirectional_command_flag ?   setOmniTwist   (joy->axes[1], -joy->axes[0]) 
                                   : setStreamTwist (joy->axes[1], -joy->axes[0]);

    hexapod_motion_pub_->publish(hexapod_motion);
    body_control_pub_  ->publish(body_control);
}

// CALLBACK
void TeleopJoy::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy) {
    setJoyFlags(joy);
    //RCLCPP_INFO(this->get_logger(), "Received joyX: %f", joy->axes[0]);

    if (servo_power_flag) {
        meta_cmd.power = true;
        meta_cmd_pub_->publish(meta_cmd);
        // O pressed
        if (start_command_flag) {
            start_flag = 1;
            meta_cmd.is_ready = 1;
        } else {
            start_flag = 0;
            meta_cmd.is_ready = 0;
        }

        if (start_flag) {

            if (omnidirectional_command_flag)
                setOmnidirectional();
            else
                setStreamlined();

            setGaitTripod();

            sendHexapodMotionData(joy);
        }
    }
    else {
        meta_cmd.power = false;
        meta_cmd_pub_->publish(meta_cmd);
    }
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting Teleop");
	
	rclcpp::spin(std::make_shared<TeleopJoy>());
    rclcpp::shutdown();
    return 0;
}