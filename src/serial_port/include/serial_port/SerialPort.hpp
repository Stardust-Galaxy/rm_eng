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
#include <chrono>

const int packet_size = sizeof(joint_states_for_send);

class SerialPort : public rclcpp::Node {
public:
    using JointStateMsg = sensor_msgs::msg::JointState;
    SerialPort(const rclcpp::NodeOptions& options);
    SerialPort(const rclcpp::NodeOptions& options, std::string& name);
    ~SerialPort();
    static std::shared_ptr<SerialPort> getInstance();
    bool init();
    void read();
    void write(std::vector<int16_t> data);
    void setBaudRate(uint baud_rate);
    void close();
    boost::asio::streambuf::const_buffers_type getReadBuf();
private:
    void readHeader();
    void readPayload();
    void read_handler(boost::system::error_code error_code,size_t bytes_transferred);
    void write_handler(boost::system::error_code error_code,size_t bytes_transferred);
    bool is_serial_alive();
    uint baud_rate;
    std::vector<std::string> port_names = {"/dev/ttyACM0", "/dev/ttyACM1"};
    std::string port_name;
    std::future<void> serial_read_task;
    boost::asio::io_service io_service;
    std::unique_ptr<boost::asio::io_service::work> work_guard;
    std::shared_ptr<boost::asio::serial_port> serial_port;
    boost::asio::streambuf read_buf;
    boost::system::error_code error_code;
    static std::shared_ptr<SerialPort> instance;
    static std::mutex mtx;
    std::array<uint8_t,1> header_buffer;
    std::array<int16_t,7> payload_buffer;
    std::array<int16_t,6> payload;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Subscription<JointStateMsg>::SharedPtr goal_joint_state_subscription; // publisher not written yet
    rclcpp::Publisher<JointStateMsg>::SharedPtr joint_state_publisher;
};
#endif

RCLCPP_COMPONENTS_REGISTER_NODE(SerialPort)