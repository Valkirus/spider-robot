#include "kinematics.hpp"

Kinematics::Kinematics() 
: Node ("spiderproj_kinematics") {
    params_set_flag = false;

    serial_port_fd_ = setup_serial_port("/dev/ttyACM0");
    toggleRelayOn(serial_port_fd_);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Serial port initialized successfully.");
    

    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/global_parameter_server");
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for global parameter server to be available...");
    parameters_client_->wait_for_service();

    auto parameters_future = parameters_client_->get_parameters({"num_legs","num_leg_joints",
                                                                "leg_suffixes",
                                                                
                                                                "coxa_length", "femur_length", "tibia_length",

                                                                "R1_origin", "R2_origin", "R3_origin", "L3_origin", "L2_origin", "L1_origin",
                                                                "R1_yaw"   , "R2_yaw"   , "R3_yaw"   , "L3_yaw"   , "L2_yaw"   , "L1_yaw"   ,
                                                                
                                                                "R1_sitting_radius", 
                                                                "R2_sitting_radius",
                                                                "R3_sitting_radius",
                                                                "L3_sitting_radius",
                                                                "L2_sitting_radius",
                                                                "L1_sitting_radius",
                                                                "leg_sitting_z"    ,

                                                                "workspace_ellipse_origin", "workspace_ellipse_axes",
                                                                "max_lin_velocity", "max_ang_velocity",
                                                                "transfer_height"},

            std::bind(&Kinematics::set_params_from_global_param_server, this, std::placeholders::_1));
    
    meta_cmd_sub_       = this->create_subscription<spiderproj_msgs::msg::MetaCommands>
        ("/teleop/meta_cmd", 1, std::bind(&Kinematics::meta_cmd_callback, this, std::placeholders::_1));

    hexapod_motion_sub_ = this->create_subscription<spiderproj_msgs::msg::HexapodMotion>
        ("/teleop/hexapod_motion", 1, std::bind(&Kinematics::hexapod_motion_callback, this, std::placeholders::_1));

    body_control_sub_   = this->create_subscription<spiderproj_msgs::msg::BodyControl>
        ("/teleop/body_control", 1, std::bind(&Kinematics::body_control_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&Kinematics::timer_callback, this));
}

void Kinematics::set_params_from_global_param_server(std::shared_future<std::vector<rclcpp::Parameter>> future) {
    num_legs     = future.get()[0].as_int();
    num_joints   = future.get()[1].as_int() * num_legs;
    leg_suffixes = future.get()[2].as_string_array();

    coxa_length  = future.get()[3].as_double();
    femur_length = future.get()[4].as_double();
    tibia_length = future.get()[5].as_double();

    std::vector<double> ws_origin = future.get()[25].as_double_array();
    EllipsoidWorkspace::setCenterL(KDL::Vector(ws_origin[0], ws_origin[1], ws_origin[2]));
    
    EllipsoidWorkspace::setAxes(future.get()[26].as_double_array());

    // initialize legs from global parameter
    double sitting_z  = future.get()[24].as_double();

    max_lin_velocity = future.get()[27].as_double();
    max_ang_velocity = future.get()[28].as_double();

    transfer_height = future.get()[29].as_double();

    legs.reserve(num_legs);
    for (int i = 0; i < num_legs; i++) {
        KDL::Vector origin = KDL::Vector(future.get()[6+i].as_double_array()[0], 
                                         future.get()[6+i].as_double_array()[1], 
                                         future.get()[6+i].as_double_array()[2]);

        double yaw = future.get()[12+i].as_double();
        KDL::Frame T_origin = KDL::Frame(KDL::Rotation::RPY(0,0,yaw), origin);

        double sitting_radius = future.get()[18+i].as_double();
        
        KDL::Vector sitting_position = KDL::Vector(sitting_radius * std::cos(yaw), 
                                                   sitting_radius * std::sin(yaw), 
                                                   sitting_z);

        legs.push_back(Leg(i, 
                        coxa_length, femur_length, tibia_length, 
                        T_origin, 
                        sitting_position));
        
    }

    this->motion_twist = KDL::Twist::Zero();
    this->body_pose = KDL::Frame::Identity();

    is_sitting = true;
    params_set_flag = true;
}

void Kinematics::meta_cmd_callback(const spiderproj_msgs::msg::MetaCommands::SharedPtr meta_cmd_msg) {
    this->is_power_on = meta_cmd_msg->power;
    this->is_ready    = meta_cmd_msg->is_ready;
    this->imu_status  = meta_cmd_msg->imu_status;
}

void Kinematics::hexapod_motion_callback(const spiderproj_msgs::msg::HexapodMotion::SharedPtr hexapod_motion_msg) {
    this->gait_type          = hexapod_motion_msg->gait;
    this->is_omnidirectional = hexapod_motion_msg->motion_type;
    this->motion_twist = KDL::Twist(KDL::Vector(hexapod_motion_msg->motion_twist.linear.x,
                                                hexapod_motion_msg->motion_twist.linear.y,
                                                hexapod_motion_msg->motion_twist.linear.z),

                                    KDL::Vector(hexapod_motion_msg->motion_twist.angular.x,
                                                hexapod_motion_msg->motion_twist.angular.y,
                                                hexapod_motion_msg->motion_twist.angular.z));
    Leg::motion_twist = this->motion_twist;
}

void Kinematics::body_control_callback(const spiderproj_msgs::msg::BodyControl::SharedPtr body_control_msg) {
    this->body_twist = KDL::Twist(KDL::Vector(body_control_msg->body_twist.linear.x,
                                                body_control_msg->body_twist.linear.y,
                                                body_control_msg->body_twist.linear.z),

                                    KDL::Vector(body_control_msg->body_twist.angular.x,
                                                body_control_msg->body_twist.angular.y,
                                                body_control_msg->body_twist.angular.z));
    
    double body_z_angle = body_control_msg->body_pose_euler_angles.euler_angles.z;
    double body_y_angle = body_control_msg->body_pose_euler_angles.euler_angles.y;
    double body_x_angle = body_control_msg->body_pose_euler_angles.euler_angles.x;

    double body_x       = body_control_msg->body_pose_euler_angles.position.x;
    double body_y       = body_control_msg->body_pose_euler_angles.position.y;
    double body_z       = body_control_msg->body_pose_euler_angles.position.z;

    KDL::Rotation body_rotation = KDL::Rotation::EulerZYX(body_z_angle, body_y_angle, body_x_angle);
    KDL::Vector body_position = KDL::Vector(body_x, body_y, body_z);

    this->body_pose  = KDL::Frame(body_rotation, body_position);
}

void Kinematics::timer_callback() {
    publish_joint_states();
    if(!is_power_on) { 
        if (!is_sitting)
            sit_down();
            toggleRelayOff(serial_port_fd_);
        return;
    }
    if(is_ready) {
        if(is_sitting)
            stand_up();

        if(!KDL::Equal(motion_twist, KDL::Twist::Zero(), KDL::epsilon * 5e4))
            move_robot();
        else {
            if (!is_in_null_pos)
            center_legs();
        
            move_body();
        }
        
    }
    else {
        if(!is_sitting) {
            sit_down();
        }
    }
}

#include <vector>
#include <numeric>

void Kinematics::send_get_command(int serial_port, uint8_t startIdx, uint8_t count) {
    unsigned char txbuff[3];
    txbuff[0] = 0xC7;       // Command byte
    txbuff[1] = startIdx;   // Start index
    txbuff[2] = count;      // Number of values to read

    // Send the GET command
    if (write(serial_port, txbuff, sizeof(txbuff)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("Kinematics"), "Failed to send GET command: %s", strerror(errno));
        return;
    }

    // Prepare buffer to receive data
    unsigned char rxbuff[2 * count];  // Adjust buffer size to handle multiple values
    int bytes_read = 0;
    int total_bytes_read = 0;

    // Set a timeout duration (e.g., 1 second)
    auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(1);

    // Read the response, checking for timeout
    while (total_bytes_read < 2 * count) {
        bytes_read = read(serial_port, rxbuff + total_bytes_read, 2 * count - total_bytes_read);

        if (bytes_read < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("Kinematics"), "Failed to read from serial port: %s", strerror(errno));
            return;
        }

        total_bytes_read += bytes_read;

        // Check if we've exceeded the timeout
        if (std::chrono::steady_clock::now() > timeout) {
            RCLCPP_ERROR(rclcpp::get_logger("Kinematics"), "Read timeout occurred");
            return;
        }
    }

    // Process each value received and average readings to reduce noise
    for (uint8_t i = 0; i < count; i++) {
        std::vector<uint16_t> values;
        
        // Collect multiple samples for averaging
        for (int sample = 0; sample < 5; ++sample) {  // Adjust number of samples as needed
            uint16_t value = (rxbuff[2 * i] & 0x7F) | ((rxbuff[2 * i + 1] & 0x7F) << 7);
            values.push_back(value);
        }

        // Average the samples
        uint16_t sum = std::accumulate(values.begin(), values.end(), 0);
        uint16_t avg_value = sum / values.size();

        // Calculate voltage from the averaged value
        float voltage = (avg_value / 1023.0f) * 3.3f;

        RCLCPP_INFO(rclcpp::get_logger("Kinematics"), "Average Voltage for index %d: %.2f V", startIdx + i, voltage);
    }
}


int Kinematics::setup_serial_port(const std::string& port_name) {
    // Open the serial port
    int serial_port = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_port == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to open port %s: %s", port_name.c_str(), strerror(errno));
        return -1; // Return -1 on failure
    }

    // Configure serial port settings
    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to get attributes for %s: %s", port_name.c_str(), strerror(errno));
        close(serial_port); // Close the port before returning
        return -1;
    }

    // Set baud rates
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    // Configure 8-bit chars, no parity, 1 stop bit
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                     // Disable break processing
    tty.c_lflag = 0;                            // No signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                            // No remapping, no delays
    tty.c_cc[VMIN] = 1;                         // Read at least 1 character
    tty.c_cc[VTIME] = 1;                        // Wait up to 0.1 seconds

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);     // Disable flow control
    tty.c_cflag |= (CLOCAL | CREAD);            // Enable receiver, set local mode

    // Apply the configuration
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to set attributes for %s: %s", port_name.c_str(), strerror(errno));
        close(serial_port); // Close the port before returning
        return -1;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully configured serial port: %s", port_name.c_str());
    return serial_port; // Return the file descriptor if successful
}

std::vector<std::vector<int>> pulses = {
        {460, 1460, 2460}, // L3C, 1
        {500, 1500, 2500}, // L3F, 2
        {560, 1560, 2560}, // L3T, 3
        {500, 1500, 2500}, // L2C, 4
        {450, 1450, 2450}, // L2F, 5
        {600, 1600, 2600}, // L2T, 6
        {400, 1400, 2400}, // L1C, 7
        {500, 1500, 2500}, // L1F, 8
        {440, 1440, 2440}, // L1T, 9
        {580, 1580, 2580}, // R3C, 10
        {460, 1460, 2460}, // R3F, 11
        {545, 1545, 2545}, // R3T, 12
        {520, 1520, 2520}, // R2C, 13
        {460, 1460, 2460}, // R2F, 14
        {400, 1400, 2400}, // R2T, 15
        {440, 1440, 2440}, // R1C, 16
        {520, 1520, 2520}, // R1F, 17
        {540, 1540, 2540}  // R1T, 18
};

uint16_t Kinematics::convert_angle_to_value(int servoNum, double angle) {
    // Map angle to servo range (example: -90° to 90° mapped to 0-4095)
    constexpr double min_angle = -90.0;  // Minimum angle in degrees
    constexpr double max_angle = 90.0;   // Maximum angle in degrees
    uint16_t servo_min = pulses[servoNum][0];    // Servo's minimum value
    uint16_t servo_max = pulses[servoNum][2];   // Servo's maximum value

    // Clamp the angle to the valid range
    angle = std::clamp(angle, min_angle, max_angle);

    // Scale angle to servo value
    return static_cast<uint16_t>(
        ((angle - min_angle) / (max_angle - min_angle)) * (servo_max - servo_min) + servo_min);
}

void Kinematics::toggleRelayOn(int serial_port) {
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

void Kinematics::toggleRelayOff(int serial_port) {
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

void Kinematics::send_command(int serial_port, uint8_t startIdx, uint8_t count, uint16_t* values) {
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

void Kinematics::publish_joint_states() {
    if (params_set_flag) {
        //R1
        double coxa_angle_degR1 = (legs[5].getCoxaAngle() * (180.0 / M_PI) );
        double femur_angle_degR1 = (legs[5].getFemurAngle() * (180.0 / M_PI)) - 35;
        double tibia_angle_degR1 = -(legs[5].getTibiaAngle() * (180.0 / M_PI) + 90);

        //R2
        double coxa_angle_degR2 = (legs[4].getCoxaAngle() * (180.0 / M_PI));
        double femur_angle_degR2 = (legs[4].getFemurAngle() * (180.0 / M_PI)) - 35;
        double tibia_angle_degR2 = -(legs[4].getTibiaAngle() * (180.0 / M_PI) + 90);

        // R3
        double coxa_angle_degR3 = (legs[3].getCoxaAngle() * (180.0 / M_PI));
        double femur_angle_degR3 = (legs[3].getFemurAngle() * (180.0 / M_PI)) - 35;
        double tibia_angle_degR3 = -(legs[3].getTibiaAngle() * (180.0 / M_PI) + 90);

        //L1
        double coxa_angle_degL1 = (legs[0].getCoxaAngle() * (180.0 / M_PI));
        double femur_angle_degL1 = -(legs[0].getFemurAngle() * (180.0 / M_PI) - 35);
        double tibia_angle_degL1 = (legs[0].getTibiaAngle() * (180.0 / M_PI) + 90);

        //L2
        double coxa_angle_degL2 = (legs[1].getCoxaAngle() * (180.0 / M_PI));
        double femur_angle_degL2 = -(legs[1].getFemurAngle() * (180.0 / M_PI) - 35) ;
        double tibia_angle_degL2 = (legs[1].getTibiaAngle() * (180.0 / M_PI) + 90);

        //L3
        double coxa_angle_degL3 = (legs[2].getCoxaAngle() * (180.0 / M_PI));
        double femur_angle_degL3 = -(legs[2].getFemurAngle() * (180.0 / M_PI) - 35) ;
        double tibia_angle_degL3 = (legs[2].getTibiaAngle() * (180.0 / M_PI) + 90);

        uint16_t values[18] = {
            convert_angle_to_value(0, coxa_angle_degL3),
            convert_angle_to_value(1, femur_angle_degL3),
            convert_angle_to_value(2, tibia_angle_degL3),
            convert_angle_to_value(9, coxa_angle_degR3),
            convert_angle_to_value(10, femur_angle_degR3),
            convert_angle_to_value(11, tibia_angle_degR3),
            convert_angle_to_value(3, coxa_angle_degL2),
            convert_angle_to_value(4, femur_angle_degL2),
            convert_angle_to_value(5, tibia_angle_degL2),
            convert_angle_to_value(12, coxa_angle_degR2),
            convert_angle_to_value(13, femur_angle_degR2),
            convert_angle_to_value(14, tibia_angle_degR2),
            convert_angle_to_value(6, coxa_angle_degL1),
            convert_angle_to_value(7, femur_angle_degL1),
            convert_angle_to_value(8, tibia_angle_degL1),
            convert_angle_to_value(15, coxa_angle_degR1),
            convert_angle_to_value(16, femur_angle_degR1),
            convert_angle_to_value(17, tibia_angle_degR1)
        };
        
        send_command(serial_port_fd_, 0, 18, values);

        for (Leg& leg : legs) {
            leg.resetDuration();
        }
    }
}

// Gait
void Kinematics::sit_down() {
    double dt = DEFAULT_TIMESTEP;
    send_get_command(serial_port_fd_, VOLT, 1);

    std::vector<KDL::Path_RoundedComposite*> paths;
    for (int i = 0; i < num_legs; i++) {
        paths.push_back(new KDL::Path_RoundedComposite(0.01, 0.01, new KDL::RotationalInterpolation_SingleAxis()));
        paths[i]->Add(KDL::Frame(KDL::Rotation::Identity(), legs[i].getNullPositionB()));
        paths[i]->Add(KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(legs[i].getNullPositionB().x(), 
                                                                        legs[i].getNullPositionB().y(),
                                                                        legs[i].getSittingPositionB().z())));
        paths[i]->Add(KDL::Frame(KDL::Rotation::Identity(), legs[i].getSittingPositionB()));
        paths[i]->Finish();
    }
    std::vector<KDL::VelocityProfile*> velprofs;
    for (int i = 0; i < num_legs; i++) {
        velprofs.push_back(new KDL::VelocityProfile_Trap(0.2,0.1));
        velprofs[i]->SetProfile(0,paths[i]->PathLength());  
    }

    std::vector<KDL::Trajectory*> trajs;
    for (int i = 0; i < num_legs; i++) {
        trajs.push_back(new KDL::Trajectory_Segment(paths[i], velprofs[i]));
    }

	std::vector<KDL::Trajectory_Composite*> ctrajs;
    for (int i = 0; i < num_legs; i++) {
        ctrajs.push_back(new KDL::Trajectory_Composite());
        ctrajs[i]->Add(trajs[i]);
    }

    for (double t=0.0; t<=trajs[0]->Duration(); t+=dt) {
        for (int i = 0; i < num_legs; i++) {
            legs[i].setPositionB((trajs[i]->Pos(t)).p);
        }
        int64_t dt_ns = static_cast<int64_t>(dt * 1e9);
        publish_joint_states();
        rclcpp::sleep_for(std::chrono::nanoseconds(dt_ns));
    }

    for (auto ctraj : ctrajs) {delete ctraj;}

    is_sitting = true;
    is_in_null_pos = false;
    toggleRelayOff(serial_port_fd_);
}

void Kinematics::stand_up() {
    toggleRelayOn(serial_port_fd_);
    std::vector<KDL::Path_RoundedComposite*> paths;
    for (int i = 0; i < num_legs; i++) {
        paths.push_back(new KDL::Path_RoundedComposite(0.01, 0.01, new KDL::RotationalInterpolation_SingleAxis()));
        paths[i]->Add(KDL::Frame(KDL::Rotation::Identity(), legs[i].getSittingPositionB()));
        paths[i]->Add(KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(legs[i].getNullPositionB().x(), 
                                                                        legs[i].getNullPositionB().y(),
                                                                        legs[i].getSittingPositionB().z())));
        paths[i]->Add(KDL::Frame(KDL::Rotation::Identity(), legs[i].getNullPositionB()));
        paths[i]->Finish();
    }
    std::vector<KDL::VelocityProfile*> velprofs;
    for (int i = 0; i < num_legs; i++) {
        velprofs.push_back(new KDL::VelocityProfile_Trap(0.2,0.1));
        velprofs[i]->SetProfile(0,paths[i]->PathLength());  
    }

    std::vector<KDL::Trajectory*> trajs;
    for (int i = 0; i < num_legs; i++) {
        trajs.push_back(new KDL::Trajectory_Segment(paths[i], velprofs[i]));
    }

	std::vector<KDL::Trajectory_Composite*> ctrajs;
    for (int i = 0; i < num_legs; i++) {
        ctrajs.push_back(new KDL::Trajectory_Composite());
        ctrajs[i]->Add(trajs[i]);
    }

    double dt = 0.01;
    for (double t=0.0; t<=trajs[0]->Duration(); t+=dt) {
        for (int i = 0; i < num_legs; i++) {
            legs[i].setPositionB((trajs[i]->Pos(t)).p);
        }
        int64_t dt_ns = static_cast<int64_t>(dt * 1e9);
        publish_joint_states();
        rclcpp::sleep_for(std::chrono::nanoseconds(dt_ns));
    }

    for (auto ctraj : ctrajs) {delete ctraj;}

    is_sitting = false;
    is_in_null_pos = true;
}

void Kinematics::move_robot() {
    publish_joint_states();
    switch (gait_type) {
        case 0:
            //move_wave();
            break;
        case 1:
            //move_ripple();
            break;
        case 2:
            move_tripod();
            Leg::wavePhase = 0;
            Leg::ripplePhase = (Leg::ripplePhase + 1) % 6;
            Leg::tripodPhase = (Leg::tripodPhase + 1) % 2;
            break;
        default:
            break;
    }
}

void Kinematics::move_tripod() {
    int phase = Leg::tripodPhase;

    this->body_pose = KDL::Frame::Identity();
    move_body();

    double cycle_duration = MAX_DURATION;
    for (int i = phase; i < num_legs; i+=2) {
        legs[i].generateSupportPath();
        cycle_duration = std::min(cycle_duration, legs[i].traj->Duration());
    } 

    double speed_factor = std::sqrt(pow( (motion_twist.vel.Norm()) / (max_lin_velocity), 2) + 
                                    pow( (motion_twist.rot.Norm()) / (max_ang_velocity), 2 ));

    //RCLCPP_INFO(this->get_logger(), "speed_factor: %f", speed_factor);

    cycle_duration *= speed_factor;
    if (cycle_duration <= KDL::epsilon) 
        return;

    bool not_enough_swing_time = false;
    for (int i = (phase+1)%2; i < num_legs; i+=2) {
        legs[i].generateTransferPathCenter(transfer_height * speed_factor, cycle_duration);
        if (legs[i].velprof->Vel(0.5*cycle_duration) > MAX_SWING_VEL) {
            not_enough_swing_time = true;
        }
    }

    if (not_enough_swing_time) center_legs();
    for (int i = (phase+1)%2; i < num_legs; i+=2) {
        legs[i].generateTransferPathCenter(transfer_height * speed_factor, cycle_duration);
    }
    

    double dt = cycle_duration / TRAJ_RES;
    for (double t = 0.0; std::abs(t) < cycle_duration; t+=dt) {
        if(!is_ready || !is_power_on) return;
        for (int i = 0; i < num_legs; i++) {
            legs[i].setPositionB(legs[i].traj->Pos(t).p);
        }
        int64_t dt_ns = static_cast<int64_t>(dt * 1e9);
        publish_joint_states();
        rclcpp::sleep_for(std::chrono::nanoseconds(dt_ns));
    }

    is_in_null_pos = false;
}
void Kinematics::move_body() {

    for (auto& leg : legs) {
        if(!leg.isInTaskspaceB(this->body_pose * Leg::body_pose.Inverse() * leg.getCurrentPositionB()))
            return;
    }
    
    for (auto& leg : legs) {
        leg.setPositionB(this->body_pose * Leg::body_pose.Inverse() * leg.getCurrentPositionB());
    } 
    Leg::body_pose = this->body_pose;
    publish_joint_states();
}

void Kinematics::center_legs() {
    this->body_pose = KDL::Frame::Identity();
    move_body();
    int phase = Leg::tripodPhase;
    double dt = DEFAULT_TIMESTEP;

    double duration = 1.0;
    for (int i = (phase+1)%2; i < num_legs; i+=2) {
        legs[i].generateTransferPathCenter(transfer_height * 0.5, duration);
    }
    for (double t = 0.0; std::abs(t) < duration; t+=dt) {
        for (int i = (phase+1)%2; i < num_legs; i+=2) {
            legs[i].setPositionB(legs[i].traj->Pos(t).p);
        }
        int64_t dt_ns = static_cast<int64_t>(dt * 1e9);
        publish_joint_states();
        rclcpp::sleep_for(std::chrono::nanoseconds(dt_ns));
    }
    is_in_null_pos = true;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Kinematics>());
    rclcpp::shutdown();
    return 0;
}