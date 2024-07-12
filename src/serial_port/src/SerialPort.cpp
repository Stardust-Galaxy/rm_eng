#include "serial_port/SerialPort.hpp"
#include <iostream>

std::shared_ptr<SerialPort> SerialPort::instance = nullptr;
std::mutex SerialPort::mtx;

SerialPort::SerialPort(const rclcpp::NodeOptions& options ) : Node("serial_port",options), port_name("/dev/ttyACM0"), baud_rate(115200) {
    joint_state_publisher = this->create_publisher<JointStateMsg>("joint_states",10);
    std::thread([this](){
        this->init();
        this->read();
        this->io_service.run();
    }).detach();
}

SerialPort::SerialPort(const rclcpp::NodeOptions& options, std::string& name) : Node("serial_port",options),port_name(name),baud_rate(115200){}

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
    try {
        if(serial_port == nullptr) 
            serial_port = std::make_shared<boost::asio::serial_port>(io_service, port_name);
        serial_port->open(port_name,error_code);
        serial_port->set_option(boost::asio::serial_port::baud_rate(baud_rate));
        serial_port->set_option(boost::asio::serial_port::flow_control());
        serial_port->set_option(boost::asio::serial_port::parity());
        serial_port->set_option(boost::asio::serial_port::character_size(8));
        serial_port->set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
        return true;
    } catch(std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return false;
    }
}

void SerialPort::setBaudRate(uint baud_rate) {
    this->baud_rate = baud_rate;
}

void SerialPort::setPortName(const std::string& name) {
    this->port_name = name;
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
//     joint_states* js = reinterpret_cast<joint_states*>(data.data());
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

void SerialPort::write_handler(boost::system::error_code error_code,size_t bytes_transferred) {
    if(error_code) {
        std::cerr << "Error: " << error_code.message() << std::endl;
    }
    std::cout << "Write " << bytes_transferred << " bytes" << std::endl;
}

void SerialPort::read() {
    readHeader();
}
void SerialPort::write(std::vector<uint8_t> data) {
    boost::asio::async_write(*serial_port,boost::asio::buffer(data),boost::asio::transfer_exactly(packet_size),boost::bind(&SerialPort::write_handler,this,boost::asio::placeholders::error,boost::asio::placeholders::bytes_transferred));
}

boost::asio::streambuf::const_buffers_type SerialPort::getReadBuf() {
    return read_buf.data();
}

void SerialPort::readHeader() {
    boost::asio::async_read(*serial_port, boost::asio::buffer(header_buffer, 1),
                [this](const boost::system::error_code& error, std::size_t bytes_transferred) {
                    if (!error) {
                        if (header_buffer[0] == 0x08) {
                            RCLCPP_INFO(this->get_logger(), "Read header: 0x08");
                            readPayload();
                        } else {
                            // 丢弃错误的数据帧头
                            RCLCPP_INFO(this->get_logger(), "Error header: 0x%x", header_buffer[0]);
                            readHeader();
                        }
                    } else {
                        RCLCPP_INFO(this->get_logger(), "Error reading header: %s", error.message().c_str());
                        readHeader();
                    }
                });
}

void SerialPort::readPayload() {
        boost::asio::async_read(*serial_port, boost::asio::buffer(payload_buffer, sizeof(payload_buffer)), boost::asio::transfer_exactly(sizeof(payload_buffer)),
                   [this](const boost::system::error_code& error, std::size_t bytes_transferred) {
                       if (!error) {
                            // 处理数据负载
                            RCLCPP_INFO(this->get_logger(), "Read payload: ");
                            std::memcpy(payload.begin(), payload_buffer.data(), sizeof(payload));
                            for (const auto& value : payload) {
                               std::cout << value << " ";
                            }
                            std::cout << std::endl;
                            JointStateMsg joint_state_msg;
                            joint_state_msg.header.frame_id = 0x08;
                            joint_state_msg.header.stamp = this->now();
                            joint_state_msg.name = {"joint1","joint2","joint3","joint4","joint5","joint6","joint7"};
                            for(int i = 0;i < 7;i += 1) {
                                joint_state_msg.position.push_back(payload[i]);
                            }
                            joint_state_publisher->publish(joint_state_msg);
                           // 继续读取下一个数据帧
                            readHeader();
                        } else {
                            RCLCPP_INFO(this->get_logger(), "Error reading payload: %s", error.message().c_str());
                            readHeader();
                       }
                   });
}



