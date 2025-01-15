#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
namespace rmcs_core::controller {

class Drag : public rclcpp::Node {
public:
    explicit Drag(const std::string& filename)
        : rclcpp::Node("Drag") {
        std::string package_share_directory =
            ament_index_cpp::get_package_share_directory("/rmcs_core");
        filename_ = package_share_directory +"/"+ filename;
        infile.open(filename, std::ios::binary);
        if (infile) {
            RCLCPP_INFO(this->get_logger(), "open file successfully");
            is_file_open = true;
        } else {
            is_file_open = false;
        }
    }

    void read_data_from_file() {
        if (is_file_open) {
            infile.read(reinterpret_cast<char*>(data.data()), sizeof(data));
        } else {
            for (auto& value : data) {
                value = NAN;
            }
        }
    }
    void write_data_to_file(const std::array<double, 6>& data) {
        std::ofstream outfile(
            filename_, is_first_write_ ? std::ios::binary | std::ios::trunc
                                       : std::ios::binary | std::ios::app);
        if (outfile) {
            outfile.write(reinterpret_cast<const char*>(data.data()), sizeof(data));
            is_first_write_ = false;
        } else {
        }
    }
    void reset_file_pointer() {
        if (infile) {
            infile.clear();
            infile.seekg(0, std::ios::beg);
        }
    }

    const double* get_data() const { return data.data(); }

private:
    void check_and_clear_file() {
        std::ifstream infile(filename_, std::ios::binary);
        bool file_exists = infile.is_open();
        infile.close();

        if (file_exists) {
            std::ofstream outfile(filename_, std::ios::binary | std::ios::trunc); // 清空文件
            if (!outfile) {
                RCLCPP_WARN(rclcpp::get_logger("Drag"), "Failed to clear the existing file");
            } else {
                RCLCPP_INFO(rclcpp::get_logger("Drag"), "Existing file cleared successfully");
            }
        } else {
            RCLCPP_INFO(rclcpp::get_logger("Drag"), "File does not exist, starting fresh");
        }
    }
    bool is_file_open = false;
    std::array<double, 6> data;
    std::ifstream infile;
    std::string filename_;
    bool is_first_write_;
};

} // namespace rmcs_core::controller