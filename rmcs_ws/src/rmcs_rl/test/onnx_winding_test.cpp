#include "policy.hpp"

#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
    if (argc < 2 || argc > 3)
        return 2;
    try {
        const auto run = [](rmcs::rl::OnnxPolicy& policy) {
            const auto action = policy.run({});
            if (!action)
                throw std::runtime_error(action.error());
            std::cout << "accepted";
            for (float value : *action)
                std::cout << ' ' << value;
            std::cout << '\n';
        };
        if (argc == 2) {
            rmcs::rl::OnnxPolicy policy{argv[1]};
            run(policy);
        } else {
#ifdef RMCS_WINDING_METADATA
            rmcs::rl::OnnxPolicy policy{argv[1], argv[2]};
            run(policy);
#else
            return 2;
#endif
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
