#include "serial_port/SerialPort.hpp"
#include <iostream>

std::shared_ptr<SerialPort> SerialPort::instance = nullptr;
std::mutex SerialPort::mtx;

SerialPort::SerialPort(const rclcpp::NodeOptions& options ) : Node("serial_port",options), baud_rate(115200) {
    joint_state_publisher = this->create_publisher<JointStateMsg>("joint_states",50);
    goal_joint_state_subscription = this->create_subscription<JointStateMsg>("goal_joint_states",5,[this](const JointStateMsg::SharedPtr msg){
        std::vector<int16_t> data;
        joint_states_for_send js;
        js.header = 0xFF;
        js.pitch_joint_1 = static_cast<int16_t>(msg->position[0] / 2 / M_PI * 65536);
        js.pitch_joint_2 = - static_cast<int16_t>(msg->position[1] / 2 / M_PI * 65536);
        js.pitch_joint_3 = - static_cast<int16_t>(msg->position[2] / 2 / M_PI * 8192);
        js.roll_joint_1 = - static_cast<int16_t>(msg->position[3] / 2 / M_PI * 65536);
        js.roll_joint_2 = static_cast<int16_t>(msg->position[4] / 2 / M_PI * 8192);
        js.yaw_joint_1 = static_cast<int16_t>(msg->position[6] / 2 / M_PI * 65536);
        js.tail = 0xFE;
        data.resize(7);
        memcpy(data.data(),&js,sizeof(js));
        write(data);
        RCLCPP_INFO(this->get_logger(),"Send joint states to serial port");
    });
    this->init();

    std::thread([this](){
        this->read();
        this->io_service.run();
    }).detach();
    using namespace std::chrono_literals;
    //create a timer to make sure the serial port is alive
    timer = this->create_wall_timer(5s,[this](){
        RCLCPP_INFO(this->get_logger(),"Checking serial port");
        if(!is_serial_alive()) {
            RCLCPP_ERROR(this->get_logger(),"Serial port is not alive");
            serial_port = nullptr;
        }
    });

}

SerialPort::SerialPort(const rclcpp::NodeOptions& options, std::string& name) : Node("serial_port",options), baud_rate(115200){}

SerialPort::~SerialPort() {
    if(serial_port != nullptr) {
        serial_port->close();
        serial_port = nullptr;
    }
}

std::shared_ptr<SerialPort> SerialPort::getInstance() {
    if(instance == nullptr) {
        std::lock_guard<std::mutex> lock(mtx);
        if(instance == nullptr) {
            instance = std::make_shared<SerialPort>(rclcpp::NodeOptions());
        }
    }
    return instance;
}

bool SerialPort::init() {
    for(auto& portName : port_names) {
        try {
            if(serial_port == nullptr) {
                serial_port = std::make_shared<boost::asio::serial_port>(io_service, portName);
                serial_port->close();
                serial_port->open(portName);
                if (serial_port->is_open()) {
                    RCLCPP_INFO(this->get_logger(), "Opened port %s", portName.c_str());
                    //serial_port->set_option(boost::asio::serial_port::baud_rate(baud_rate));
                    serial_port->set_option(boost::asio::serial_port::flow_control());
                    serial_port->set_option(boost::asio::serial_port::parity());
                    serial_port->set_option(boost::asio::serial_port::character_size(8));
                    port_name = portName;
                    //serial_port->set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
                    return true;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to open port %s", portName.c_str());
                    serial_port = nullptr;
                }
            }
        } catch(std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }
    }
}

void SerialPort::setBaudRate(uint baud_rate) {
    this->baud_rate = baud_rate;
}

void SerialPort::close() {
    serial_port->close();
}

// void SerialPort::read_handler(boost::system::error_code error_code,size_t bytes_transferred) {
//     if(error_code) {
//         std::cerr << "Error: " << error_code.message() << std::endl;
//     }
//     //RCLCPP_INFO(this->get_logger(),"Read %ld bytes", bytes_transferred);
//     std::vector<uint8_t> data(buffer.begin(),buffer.end());
//     joint_states_for_send* js = reinterpret_cast<joint_states_for_send*>(data.data());
//     JointStateMsg joint_state_msg;
//     joint_state_msg.header.frame_id = js->header;
//     joint_state_msg.header.stamp = this->now();
//     joint_state_msg.name = {"joint1","joint2","joint3","joint4","joint5","joint6","joint7"};
//     for(int i = 0;i < 7;i += 1) {
//         joint_state_msg.position.push_back(js->positions[i]);
//     }
//     joint_state_publisher->publish(joint_state_msg);
//     read();
// }

bool SerialPort::is_serial_alive() {
    if (serial_port && serial_port->is_open()) {
        try {
            char test_byte = 0x00;
            serial_port->write_some(boost::asio::buffer(&test_byte, 1));
            return true;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Serial port write failed: %s", e.what());
            return false;
        }
    }
    return false;
}

void SerialPort::write_handler(boost::system::error_code error_code,size_t bytes_transferred) {
    if(error_code) {
        RCLCPP_ERROR(this->get_logger(),"Error: %s",error_code.message().c_str());
    }
//    std::cout << "Write " << bytes_transferred << " bytes" << std::endl;
}

void SerialPort::read() {
    readHeader();
}
void SerialPort::write(std::vector<int16_t> data) {
    boost::asio::async_write(*serial_port,boost::asio::buffer(data),boost::asio::transfer_exactly(packet_size),boost::bind(&SerialPort::write_handler,this,boost::asio::placeholders::error,boost::asio::placeholders::bytes_transferred));
}

boost::asio::streambuf::const_buffers_type SerialPort::getReadBuf() {
    return read_buf.data();
}

void SerialPort::readHeader() {
    boost::asio::async_read(*serial_port, boost::asio::buffer(header_buffer, 1),
        [this](const boost::system::error_code &error, std::size_t bytes_transferred) {
            //RCLCPP_INFO(this->get_logger(), "Read header: %d", header_buffer[0]);
            if (!error) {
                if (header_buffer[0] == 0xff) {
//                    RCLCPP_INFO(this->get_logger(), "Read header: 0xFF");
                    readPayload();
                } else {
                    // 丢弃错误的数据帧头
//                    RCLCPP_INFO(this->get_logger(), "Error header: 0x%x", header_buffer[0]);
                    readHeader();
                }
            } else {
//                RCLCPP_INFO(this->get_logger(), "Error reading header: %s", error.message().c_str());
                readHeader();
            }
        });
}

void SerialPort::readPayload() {
        boost::asio::async_read(*serial_port, boost::asio::buffer(payload_buffer, sizeof(payload_buffer)), boost::asio::transfer_exactly(sizeof(payload_buffer)),
                   [this](const boost::system::error_code& error, std::size_t bytes_transferred) {
                       if (!error) {
                            // 处理数据负载
//                            RCLCPP_INFO(this->get_logger(), "Read payload: ");
                            joint_states_ received_joint_states{};
                            memcpy(&received_joint_states, payload_buffer.data(), sizeof(received_joint_states));
                            JointStateMsg joint_state_msg;
                            joint_state_msg.header.frame_id = "base_link";
                            joint_state_msg.header.stamp = this->now();
                            joint_state_msg.name = {"yaw_joint_1", "pitch_joint_1", "pitch_joint_2", "roll_joint_1", "pitch_joint_3", "roll_joint_2", "shift_joint"};
                            joint_state_msg.position.resize(7);
                            joint_state_msg.position[0] = static_cast<double>(received_joint_states.yaw_joint_1) / 65536 * 2 * M_PI ;
                            joint_state_msg.position[1] = static_cast<double>(received_joint_states.pitch_joint_1) / 65536 * 2 * M_PI;
                            joint_state_msg.position[2] = - static_cast<double>(received_joint_states.pitch_joint_2) / 65536 * 2 * M_PI;
                            joint_state_msg.position[3] =  - static_cast<double>(received_joint_states.roll_joint_1) / 65536 * 2 * M_PI;
                            joint_state_msg.position[4] = - static_cast<double>(received_joint_states.pitch_joint_3) / 8192 * 2 * M_PI;
                            joint_state_msg.position[5] = (static_cast<double>(received_joint_states.roll_joint_2 % 8192) / 8192 * 2 * M_PI);
                            joint_state_msg.position[6] = 0.0;
//                            RCLCPP_INFO(this->get_logger(), "yaw_joint_1: %f, pitch_joint_1: %f, pitch_joint_2: %f, roll_joint_1: %f, pitch_joint_3: %f, roll_joint_2: %f", joint_state_msg.position[0], joint_state_msg.position[1], joint_state_msg.position[2], joint_state_msg.position[3], joint_state_msg.position[4], joint_state_msg.position[5]);
                            joint_state_publisher->publish(joint_state_msg);
                            readHeader();
                        } else {
//                            RCLCPP_INFO(this->get_logger(), "Error reading payload: %s", error.message().c_str());
                            readHeader();
                       }
                   });
}



