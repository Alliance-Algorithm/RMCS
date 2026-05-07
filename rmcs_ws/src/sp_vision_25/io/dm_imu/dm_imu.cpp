#include "dm_imu.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace io
{
DM_IMU::DM_IMU() : queue_(5000)
{
  init_serial();
  rec_thread_ = std::thread(&DM_IMU::get_imu_data_thread, this);
  queue_.pop(data_ahead_);
  queue_.pop(data_behind_);
  tools::logger()->info("[DM_IMU] initialized");
}

DM_IMU::~DM_IMU()
{
  stop_thread_ = true;
  if (rec_thread_.joinable()) {
    rec_thread_.join();
  }
  if (serial_.isOpen()) {
    serial_.close();
  }
}

void DM_IMU::init_serial()
{
  try {
    serial_.setPort("/dev/ttyACM0");
    serial_.setBaudrate(921600);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);  //default is parity_none
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(20);
    serial_.setTimeout(time_out);
    serial_.open();
    usleep(1000000);  //1s

    tools::logger()->info("[DM_IMU] serial port opened");
  }

  catch (serial::IOException & e) {
    tools::logger()->warn("[DM_IMU] failed to open serial port ");
    exit(0);
  }
}

void DM_IMU::get_imu_data_thread()
{
  while (!stop_thread_) {
    if (!serial_.isOpen()) {
      tools::logger()->warn("In get_imu_data_thread,imu serial port unopen");
    }

    serial_.read((uint8_t *)(&receive_data.FrameHeader1), 4);

    if (
      receive_data.FrameHeader1 == 0x55 && receive_data.flag1 == 0xAA &&
      receive_data.slave_id1 == 0x01 && receive_data.reg_acc == 0x01)

    {
      serial_.read((uint8_t *)(&receive_data.accx_u32), 57 - 4);

      if (tools::get_crc16((uint8_t *)(&receive_data.FrameHeader1), 16) == receive_data.crc1) {
        data.accx = *((float *)(&receive_data.accx_u32));
        data.accy = *((float *)(&receive_data.accy_u32));
        data.accz = *((float *)(&receive_data.accz_u32));
      }
      if (tools::get_crc16((uint8_t *)(&receive_data.FrameHeader2), 16) == receive_data.crc2) {
        data.gyrox = *((float *)(&receive_data.gyrox_u32));
        data.gyroy = *((float *)(&receive_data.gyroy_u32));
        data.gyroz = *((float *)(&receive_data.gyroz_u32));
      }
      if (tools::get_crc16((uint8_t *)(&receive_data.FrameHeader3), 16) == receive_data.crc3) {
        data.roll = *((float *)(&receive_data.roll_u32));
        data.pitch = *((float *)(&receive_data.pitch_u32));
        data.yaw = *((float *)(&receive_data.yaw_u32));
        // tools::logger()->debug(
        //   "yaw: {:.2f}, pitch: {:.2f}, roll: {:.2f}", static_cast<double>(data.yaw),
        //   static_cast<double>(data.pitch), static_cast<double>(data.roll));
      }
      auto timestamp = std::chrono::steady_clock::now();
      Eigen::Quaterniond q = Eigen::AngleAxisd(data.yaw * M_PI / 180, Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(data.pitch * M_PI / 180, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(data.roll * M_PI / 180, Eigen::Vector3d::UnitX());
      q.normalize();
      queue_.push({q, timestamp});
    } else {
      tools::logger()->info("[DM_IMU] failed to get correct data");
    }
  }
}

Eigen::Quaterniond DM_IMU::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

  while (true) {
    queue_.pop(data_behind_);
    if (data_behind_.timestamp > timestamp) break;
    data_ahead_ = data_behind_;
  }

  Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  Eigen::Quaterniond q_b = data_behind_.q.normalized();
  auto t_a = data_ahead_.timestamp;
  auto t_b = data_behind_.timestamp;
  auto t_c = timestamp;
  std::chrono::duration<double> t_ab = t_b - t_a;
  std::chrono::duration<double> t_ac = t_c - t_a;

  // 四元数插值
  auto k = t_ac / t_ab;
  Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

  return q_c;
}

}  // namespace io
