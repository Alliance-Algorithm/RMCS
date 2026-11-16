#pragma once

#include <cstddef>
#include <memory>
#include <new>
#include <string>
#include <string_view>
#include <utility>

#include <librmcs/board/c_board.hpp>
#include <librmcs/data/datas.hpp>

namespace librmcs::data {
struct AccelerometerDataView : ImuAccelerometerDataView {
    explicit AccelerometerDataView(const ImuAccelerometerDataView& data)
        : ImuAccelerometerDataView{data} {}
};
struct GyroscopeDataView : ImuGyroscopeDataView {
    explicit GyroscopeDataView(const ImuGyroscopeDataView& data)
        : ImuGyroscopeDataView{data} {}
};
} // namespace librmcs::data

namespace librmcs::agent {

// Compatibility adapter for hardware drivers that have not yet been migrated
// to the librmcs v3.3 callback API.
class CBoard : public librmcs::board::CBoard::Callback {
public:
    using Spec = librmcs::board::CBoard::Callback::Spec;
    using View = librmcs::board::CBoard::Callback::View;

    explicit CBoard(std::string_view serial_filter = {})
        : serial_filter_{serial_filter} {}

    CBoard(const CBoard&) = delete;
    CBoard& operator=(const CBoard&) = delete;
    CBoard(CBoard&&) = delete;
    CBoard& operator=(CBoard&&) = delete;

    ~CBoard() override = default;

    class PacketBuilder {
    public:
        PacketBuilder(const PacketBuilder&) = delete;
        PacketBuilder& operator=(const PacketBuilder&) = delete;
        PacketBuilder(PacketBuilder&&) = delete;
        PacketBuilder& operator=(PacketBuilder&&) = delete;

        ~PacketBuilder() { std::destroy_at(inner()); }

        PacketBuilder& can1_transmit(const librmcs::data::CanDataView& data) {
            inner()->can_transmit(Spec::kCans.kCan1, data);
            return *this;
        }

        PacketBuilder& can2_transmit(const librmcs::data::CanDataView& data) {
            inner()->can_transmit(Spec::kCans.kCan2, data);
            return *this;
        }

        PacketBuilder& dbus_transmit(const librmcs::data::UartDataView& data) {
            inner()->uart_transmit(Spec::kUarts.kDbus, data);
            return *this;
        }

        PacketBuilder& uart1_transmit(const librmcs::data::UartDataView& data) {
            inner()->uart_transmit(Spec::kUarts.kUart1, data);
            return *this;
        }

        PacketBuilder& uart2_transmit(const librmcs::data::UartDataView& data) {
            inner()->uart_transmit(Spec::kUarts.kUart2, data);
            return *this;
        }

    private:
        friend class CBoard;
        using Inner = librmcs::board::CBoard::PacketBuilder;

        explicit PacketBuilder(CBoard& board) {
            ::new (static_cast<void*>(storage_))
                Inner(board.ensure_started().start_transmit());
        }

        Inner* inner() {
            return std::launder(reinterpret_cast<Inner*>(storage_));
        }

        alignas(Inner) std::byte storage_[sizeof(Inner)];
    };

    PacketBuilder start_transmit() { return PacketBuilder{*this}; }

protected:
    virtual void can1_receive_callback(const librmcs::data::CanDataView& data) {
        (void)data;
    }
    virtual void can2_receive_callback(const librmcs::data::CanDataView& data) {
        (void)data;
    }

    virtual void dbus_receive_callback(const librmcs::data::UartDataView& data) {
        (void)data;
    }
    virtual void uart1_receive_callback(const librmcs::data::UartDataView& data) {
        (void)data;
    }
    virtual void uart2_receive_callback(const librmcs::data::UartDataView& data) {
        (void)data;
    }

private:
    librmcs::board::CBoard& ensure_started() {
        if (!board_) {
            board_ = std::make_unique<librmcs::board::CBoard>(
                *this, serial_filter_);
        }
        return *board_;
    }

    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (can == Spec::kCans.kCan1)
            can1_receive_callback(data);
        else if (can == Spec::kCans.kCan2)
            can2_receive_callback(data);
    }

    void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
        if (uart == Spec::kUarts.kDbus)
            dbus_receive_callback(data);
        else if (uart == Spec::kUarts.kUart1)
            uart1_receive_callback(data);
        else if (uart == Spec::kUarts.kUart2)
            uart2_receive_callback(data);
    }

    void accelerometer_receive_callback(const View::ImuAccelerometer& data) override {
        const librmcs::data::AccelerometerDataView legacy_data{data};
        accelerometer_receive_callback(legacy_data);
    }

    void gyroscope_receive_callback(const View::ImuGyroscope& data) override {
        const librmcs::data::GyroscopeDataView legacy_data{data};
        gyroscope_receive_callback(legacy_data);
    }

    virtual void accelerometer_receive_callback(
        const librmcs::data::AccelerometerDataView& data) {
        (void)data;
    }
    virtual void gyroscope_receive_callback(
        const librmcs::data::GyroscopeDataView& data) {
        (void)data;
    }

    std::string serial_filter_;
    std::unique_ptr<librmcs::board::CBoard> board_;
};

} // namespace librmcs::agent
