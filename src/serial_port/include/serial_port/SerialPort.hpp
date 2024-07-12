#ifndef SERIAL_PORT_HPP
#define SERIAL_PORT_HPP
#include "serial_port/RMEngJointState.hpp"
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <memory>
#include <thread>
#include <vector>
#include <array>
#include <mutex>

const int packet_size = sizeof(joint_states);

class SerialPort : public rclcpp::Node {
public:
    using JointStateMsg = sensor_msgs::msg::JointState;
        SerialPort(const rclcpp::NodeOptions& options);
    SerialPort(const rclcpp::NodeOptions& options, std::string& name);
    ~SerialPort();
    static std::shared_ptr<SerialPort> getInstance();
    bool init();
    void read();
    void write(std::vector<uint8_t> data);
    void setBaudRate(uint badu_rate);
    void setPortName(const std::string& name);
    void close();
    boost::asio::streambuf::const_buffers_type getReadBuf();
private:
    void readHeader();
    void readPayload();
    void read_handler(boost::system::error_code error_code,size_t bytes_transferred);
    void write_handler(boost::system::error_code error_code,size_t bytes_transferred);
    uint baud_rate;
    std::string port_name;
    boost::asio::io_service io_service;
    std::shared_ptr<boost::asio::serial_port> serial_port;
    boost::asio::streambuf read_buf;
    boost::system::error_code error_code;
    static std::shared_ptr<SerialPort> instance;
    static std::mutex mtx;
    std::array<uint8_t,1> header_buffer;
    std::array<uint8_t, 7 * sizeof(double)> payload_buffer;
    std::array<double,7> payload;
    rclcpp::Publisher<JointStateMsg>::SharedPtr joint_state_publisher;
};
#endif

RCLCPP_COMPONENTS_REGISTER_NODE(SerialPort)