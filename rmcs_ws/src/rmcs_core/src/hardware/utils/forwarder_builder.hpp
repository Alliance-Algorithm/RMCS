#pragma once

#include "controller/chassis/steering_wheel/concexpt_chassis_controller.hpp"
#include "hardware/forwarder/cboard.hpp"

#include <cmath>
#include <functional>
#include <memory>
#include <rclcpp/logger.hpp>
#include <type_traits>
#include <utility>

namespace rmcs_core::hardware
{
class ForwarderConstructDataBase
{
public:
    virtual void can1_receive_callback(uint32_t, uint64_t, bool, bool, uint8_t) const = 0;
    virtual void can2_receive_callback(uint32_t, uint64_t, bool, bool, uint8_t) const = 0;
    virtual void uart1_receive_callback(const std::byte*, uint8_t) const = 0;
    virtual void dbus_receive_callback(const std::byte*, uint8_t) const = 0;
    virtual void accelerometer_receive_callback(int16_t, int16_t, int16_t) const = 0;
    virtual void gyroscope_receive_callback(int16_t, int16_t, int16_t) const = 0;
};

struct Flag
{
    uint32_t reserve    : 8;
    uint32_t board_type : 8;
    uint32_t usb_pid    : 16;
};

template <typename T>
requires DerivedFromBoth<T>
class ForwarderConstructData final : public ForwarderConstructDataBase
{
public:
    ForwarderConstructData(T*                    instance_object,
                           const rclcpp::Logger& logger,
                           const Flag&           forwarder_flag,
                           auto T::*forwarder_can1_receive_callback,
                           auto T::*forwarder_can2_receive_callback,
                           auto T::*forwarder_uart1_receive_callback,
                           auto T::*forwarder_dbus_receive_callback,
                           auto T::*forwarder_accelerometer_receive_callback,
                           auto T::*forwarder_gyroscope_receive_callback)
        : logger(logger)
        , flag { forwarder_flag }
        , can1_receive_callback_ { forwarder_can1_receive_callback }
        , can2_receive_callback_ { forwarder_can2_receive_callback }
        , uart1_receive_callback_ { forwarder_uart1_receive_callback }
        , dbus_receive_callback_ { forwarder_dbus_receive_callback }
        , accelerometer_receive_callback_ { forwarder_accelerometer_receive_callback }
        , gyroscope_receive_callback_ { forwarder_gyroscope_receive_callback }
        , instance_(instance_object) {
        RCLCPP_INFO(logger, "ForwarderConstructData");
    }

    inline void can1_receive_callback(uint32_t a, uint64_t b, bool c, bool d, uint8_t e) const override {
        (instance_->*can1_receive_callback_)(a, b, c, d, e);
    }
    inline void can2_receive_callback(uint32_t a, uint64_t b, bool c, bool d, uint8_t e) const override {
        (instance_->*can2_receive_callback_)(a, b, c, d, e);
    }
    inline void uart1_receive_callback(const std::byte* a, uint8_t b) const override {
        (instance_->*uart1_receive_callback_)(a, b);
    }
    inline void dbus_receive_callback(const std::byte* a, uint8_t b) const override {
        (instance_->*dbus_receive_callback_)(a, b);
    }
    inline void accelerometer_receive_callback(int16_t a, int16_t b, int16_t c) const override {
        (instance_->*accelerometer_receive_callback_)(a, b, c);
    }
    inline void gyroscope_receive_callback(int16_t a, int16_t b, int16_t c) const override {
        (instance_->*gyroscope_receive_callback_)(a, b, c);
    }

    const rclcpp::Logger& logger;
    const Flag            flag;

private:
    void (T::*const can1_receive_callback_)(uint32_t, uint64_t, bool, bool, uint8_t);
    void (T::*const can2_receive_callback_)(uint32_t, uint64_t, bool, bool, uint8_t);
    void (T::*const uart1_receive_callback_)(const std::byte*, uint8_t);
    void (T::*const dbus_receive_callback_)(const std::byte*, uint8_t);
    void (T::*const accelerometer_receive_callback_)(int16_t, int16_t, int16_t);
    void (T::*const gyroscope_receive_callback_)(int16_t, int16_t, int16_t);

    // unsafe
    T* const instance_;
};

class Forwarder
{
public:
    virtual bool
        add_can1_transmission(uint32_t, uint64_t, bool = false, bool = false, uint8_t = 8) const noexcept {
        return true;
    }
    virtual bool
        add_can2_transmission(uint32_t, uint64_t, bool = false, bool = false, uint8_t = 8) const noexcept {
        return true;
    }
    virtual bool add_uart1_transmission(const std::byte*, uint8_t) const { return true; }

    virtual bool add_uart2_transmission(const std::byte*, uint8_t) const { return true; }

    virtual bool add_dbus_transmission(const std::byte*, uint8_t) const { return true; }

    virtual bool trigger_transmission() const noexcept { return true; }
};

template <typename T>
class ForwarderProduct : Forwarder
{ };

class ForwarderOpenedDecorator : public Forwarder
{
public:
    explicit ForwarderOpenedDecorator(ForwarderConstructDataBase& callbacks_in)
        : callbacks(callbacks_in) {};
    inline bool add_can1_transmission(uint32_t can_id,
                                      uint64_t can_data,
                                      bool     is_extended_can_id = false,
                                      bool     is_remote_transmission = false,
                                      uint8_t  can_data_length = 8) const noexcept override {
        return transmit_buffer_->add_can1_transmission(can_id,
                                                       can_data,
                                                       is_extended_can_id,
                                                       is_remote_transmission,
                                                       can_data_length);
    };
    inline bool add_can2_transmission(uint32_t can_id,
                                      uint64_t can_data,
                                      bool     is_extended_can_id = false,
                                      bool     is_remote_transmission = false,
                                      uint8_t  can_data_length = 8) const noexcept override {
        return transmit_buffer_->add_can2_transmission(can_id,
                                                       can_data,
                                                       is_extended_can_id,
                                                       is_remote_transmission,
                                                       can_data_length);
    };
    inline bool add_uart1_transmission(const std::byte* uart_data, uint8_t uart_data_length) const override {
        return transmit_buffer_->add_uart1_transmission(uart_data, uart_data_length);
    }
    inline bool add_uart2_transmission(const std::byte* uart_data, uint8_t uart_data_length) const override {
        return transmit_buffer_->add_uart2_transmission(uart_data, uart_data_length);
    }
    inline bool add_dbus_transmission(const std::byte* uart_data, uint8_t uart_data_length) const override {
        return transmit_buffer_->add_dbus_transmission(uart_data, uart_data_length);
    }
    inline bool trigger_transmission() const noexcept override {
        return transmit_buffer_->trigger_transmission();
    };

protected:
    std::unique_ptr<forwarder::TransmitBufferBase> transmit_buffer_;
    ForwarderConstructDataBase&                    callbacks;
};

template <>
class ForwarderProduct<forwarder::CBoard> final
    : public ForwarderOpenedDecorator
    , public forwarder::CBoard
{
public:
    template <typename T>
    ForwarderProduct(ForwarderConstructData<T>&& data)
        : ForwarderOpenedDecorator(data)
        , forwarder::CBoard(data.flag.usb_pid, data.logger) {
        transmit_buffer_ = (std::make_unique<forwarder::CBoard::TransmitBuffer>(*this, 16));
    }

private:
    inline void can1_receive_callback(uint32_t a, uint64_t b, bool c, bool d, uint8_t e) override {
        callbacks.can1_receive_callback(a, b, c, d, e);
    }
    inline void can2_receive_callback(uint32_t a, uint64_t b, bool c, bool d, uint8_t e) override {
        callbacks.can2_receive_callback(a, b, c, d, e);
    }
    inline void uart1_receive_callback(const std::byte* a, uint8_t b) override {
        callbacks.uart1_receive_callback(a, b);
    }
    inline void dbus_receive_callback(const std::byte* a, uint8_t b) override {
        callbacks.dbus_receive_callback(a, b);
    }
    inline void accelerometer_receive_callback(int16_t a, int16_t b, int16_t c) override {
        callbacks.accelerometer_receive_callback(a, b, c);
    }
    inline void gyroscope_receive_callback(int16_t a, int16_t b, int16_t c) override {
        callbacks.gyroscope_receive_callback(a, b, c);
    }
};

// template <>
// class ForwardProduct<forwarder::Other> : forwarder::Other;

template <typename T, typename N>
concept IsForwarderConstructData = std::is_same_v<N, ForwarderConstructData<T>> || std::is_same_v < N,
        ForwarderConstructData<T>
& > ;

class ForwarderFactory
{
public:
    template <typename T, typename N>
    requires IsForwarderConstructData<T, N>
    static std::unique_ptr<const Forwarder> Create(N&& data) {
        RCLCPP_INFO(data.logger, "create");
        switch (data.flag.board_type) {
        case 1:
            return std::unique_ptr<const Forwarder>(static_cast<const Forwarder*>(
                std::move(std::make_unique<ForwarderProduct<forwarder::CBoard>>(std::move(data)).release())));
            break;
        case 0:
        default:
            return std::make_unique<const Forwarder>();
        }
    }
};
} // namespace rmcs_core::hardware