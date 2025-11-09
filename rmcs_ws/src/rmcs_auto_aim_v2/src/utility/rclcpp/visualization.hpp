#pragma once
#include "utility/pimpl.hpp"

namespace rmcs::util {

namespace details {
    template <class T>
    concept translation_trait = requires(T t) {
        t.x();
        t.y();
        t.z();
    };
    template <class Q>
    concept orientation_trait = requires(Q q) {
        q.x();
        q.y();
        q.z();
        q.w();
    };

    struct Translation {
        double x;
        double y;
        double z;
        constexpr explicit Translation(translation_trait auto t) noexcept
            : x { t.x() }
            , y { t.y() }
            , z { t.z() } { }
        auto operator=(translation_trait auto const& t) noexcept -> Translation& {
            x = t.x();
            y = t.y();
            z = t.z();
            return *this;
        }
    };
    struct Orientation {
        double x;
        double y;
        double z;
        double w;
        constexpr explicit Orientation(orientation_trait auto q) noexcept
            : x { q.x() }
            , y { q.y() }
            , z { q.z() }
            , w { q.z() } { }
        auto operator=(orientation_trait auto const& q) noexcept -> Orientation& {
            x = q.x();
            y = q.y();
            z = q.z();
            w = q.w();
            return *this;
        }
    };
}

namespace item {

    struct Armor {
        details::Translation t;
        details::Orientation q;
    };

}

class Visualization {
    RMCS_PIMPL_DEFINITION(Visualization)

public:
    auto set_topic_prefix(const std::string&) noexcept -> void;

    auto set_scaling_value(double) noexcept -> void;

    auto set_transform_link(const std::string&) noexcept -> void;
};

}
