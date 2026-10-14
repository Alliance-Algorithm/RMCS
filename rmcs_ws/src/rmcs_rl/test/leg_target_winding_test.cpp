#include "leg_target_winding.hpp"

#include <algorithm>
#include <bit>
#include <cassert>
#include <cstdint>
#include <iomanip>
#include <iostream>

using namespace rmcs::rl;

int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string_view{argv[1]} == "--nominal") {
            std::cout << std::setprecision(17);
            for (int i = 0; i < 4; ++i)
                std::cout << kV6PolicyProfile.nominal[i] << ' ';
            return 0;
        }
        if (argc >= 3 && std::string_view{argv[1]} == "--binary") {
            const auto mode = parse_leg_target_winding(argv[2], argc > 3 ? argv[3] : "v6");
            std::array<double, 8> row;
            while (std::cin.read(reinterpret_cast<char*>(row.data()), sizeof(row))) {
                std::array<double, 4> current, desired;
                std::copy_n(row.begin(), 4, current.begin());
                std::copy_n(row.begin() + 4, 4, desired.begin());
                const auto result = wind_leg_targets(desired, current, mode);
                std::cout.write(reinterpret_cast<const char*>(result.data()), sizeof(result));
            }
            if (!std::cin.eof() || std::cin.gcount() != 0)
                throw std::runtime_error("Incomplete binary input row");
            return 0;
        }
        const auto independent = parse_leg_target_winding("independent_nearest_v1", "v6");
        assert(independent == parse_leg_target_winding("independent_nearest_v1", "v5"));
        const auto paired = parse_leg_target_winding("paired_common_v1", "v6");
        for (const auto& [version, profile] : {
                 std::pair{"unknown", "v6"}, std::pair{"", "v6"},
                 std::pair{"paired_common_v1", "v5"}, std::pair{"paired_common_v1", "unknown"}}) {
            bool rejected = false;
            try {
                parse_leg_target_winding(version, profile);
            } catch (const std::runtime_error&) {
                rejected = true;
            }
            assert(rejected);
        }
        constexpr double tau = 2 * std::numbers::pi;
        const std::array<double, 4> desired{-.41, .127, .41, -.127};
        for (int turns : {-20, -2, -1, 0, 1, 2, 20}) {
            std::array<double, 4> current;
            for (int i = 0; i < 4; ++i)
                current[i] = desired[i] + turns * tau + (i % 2 == 0 ? .02 : -.02);
            const auto old = wind_leg_targets(desired, current, independent);
            const auto common = wind_leg_targets(desired, current, paired);
            for (int i = 0; i < 4; ++i) {
                const double original = current[i] + std::remainder(desired[i] - current[i], tau);
                assert(std::bit_cast<std::uint64_t>(old[i]) == std::bit_cast<std::uint64_t>(original));
                assert(std::abs(common[i] - (desired[i] + turns * tau)) < 1e-12);
            }
            for (int side = 0; side < 2; ++side)
                assert(std::abs((common[2 * side + 1] - common[2 * side])
                                - (desired[2 * side + 1] - desired[2 * side])) < 1e-12);
        }
        // The half-turn rule is floor(x + .5), including its negative tie.
        std::array<double, 4> zero{}, positive{}, negative{};
        positive.fill(std::numbers::pi);
        negative.fill(-std::numbers::pi);
        assert(wind_leg_targets(zero, positive, paired)[0] == tau);
        assert(wind_leg_targets(zero, negative, paired)[0] == 0);
        std::cout << "production winding/parser checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 2;
    }
}
