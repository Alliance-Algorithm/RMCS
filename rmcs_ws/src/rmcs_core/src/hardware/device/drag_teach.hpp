#pragma once

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
            filename_ = filename;
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
        std::ofstream outfile(filename_, std::ios::binary | std::ios::app);
        if (outfile) {
            outfile.write(reinterpret_cast<const char*>(data.data()), sizeof(data));
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

    bool is_file_open = false;
    std::array<double, 6> data;
    std::ifstream infile;
    std::string filename_;
};

} // namespace rmcs_core::controller