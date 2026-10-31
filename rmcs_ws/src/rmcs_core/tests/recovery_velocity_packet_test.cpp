#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "hardware/device/dm_motor.hpp"

using Motor=rmcs_core::hardware::device::DmMotor;
class Fixture : public rmcs_executor::Component {
public:
    void update() override {}
};

void require(bool condition,const char* message) {
    if (!condition) throw std::runtime_error(message);
}

std::array<double,4> decode(rmcs_core::hardware::device::CanPacket8 packet) {
    const auto data=packet.as_bytes();
    std::array<unsigned,8> b{};
    for (std::size_t i=0;i<8;++i) b[i]=std::to_integer<unsigned>(data[i]);
    return {(double((b[2]<<4)|(b[3]>>4))/4095.*90.)-45.,
        double(((b[3]&15)<<8)|b[4])/4095.*500.,
        double((b[5]<<4)|(b[6]>>4))/4095.*5.,
        (double(((b[6]&15)<<8)|b[7])/4095.*108.)-54.};
}

int main() {
    for (bool reversed:{false,true}) {
        Fixture status,command;
        Motor::Config config{Motor::Type::kDM8009};
        config.id=1;config.reversed=reversed;
        Motor motor(status,command,"/test/motor",config);
        require(!motor.velocity_control_requested(),"An absent velocity request enabled the servo");
        for (double velocity:{-3.14,-2.5,-1.2,0.,1.2,2.5,3.14}) {
            const auto values=decode(motor.generate_velocity_command(velocity,5.,18.));
            const double api_velocity=(reversed ? -1. : 1.)*values[0];
            require(values[1]==0. && std::abs(values[2]-5.)<1e-12,"MIT velocity mode has wrong gains");
            require(std::abs(api_velocity)<=3.14,"Encoded target exceeds the mechanical speed bound");
            require(std::abs(api_velocity-velocity)<.05,"MIT velocity sign/range conversion differs");
        }
        for (double velocity:{-1.5,1.5}) {
            const double ff=velocity>0 ? 40. : -40.;
            const auto values=decode(motor.generate_velocity_command(velocity,5.,ff));
            const double sign=reversed ? -1. : 1.;
            require(std::abs(sign*values[3]+5.*sign*values[0])<=40.15,
                "Measured-state total effort was not limited before packet quantization");
        }
        for (const auto parameters:{std::array{3.1401,5.,0.},std::array{0.,5.001,0.},
                 std::array{0.,0.,0.},std::array{0.,5.,std::numeric_limits<double>::quiet_NaN()}}) {
            const auto values=decode(motor.generate_velocity_command(parameters[0],parameters[1],parameters[2]));
            require(values[1]==0. && values[2]==0. && std::abs(values[3])<.014,
                "Invalid velocity request did not produce a zero-gain zero-torque frame");
        }
        const auto pure=decode(motor.generate_command(3.));
        require(pure[1]==0. && pure[2]==0.,"Pure torque path changed gains");
    }
    std::cout<<"PASS: MIT velocity packet bounds, reversal, gains, effort and invalid-input rejection\n";
}
