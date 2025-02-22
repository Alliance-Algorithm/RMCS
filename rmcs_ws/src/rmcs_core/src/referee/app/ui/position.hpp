#include <cstdint>
namespace rmcs_core::referee::app::ui {
static constexpr uint16_t screen_width = 1920, screen_height = 1080;
static constexpr uint16_t x_center = screen_width / 2, y_center = screen_height / 2;

static constexpr uint16_t offset_            = 120;
static constexpr uint16_t red_hero_x         = x_center - 240;
static constexpr uint16_t red_engineer_x     = red_hero_x - offset_;
static constexpr uint16_t red_infantry_III_x = red_hero_x - 2 * offset_;
static constexpr uint16_t red_infantry_IV_x  = red_hero_x - 3 * offset_;
static constexpr uint16_t red_infantry_V_x   = red_hero_x - 4 * offset_;
static constexpr uint16_t red_sentry_x       = red_hero_x - 5 * offset_;

static constexpr uint16_t blue_hero_x         = x_center + 140;
static constexpr uint16_t blue_engineer_x     = red_hero_x + offset_;
static constexpr uint16_t blue_infantry_III_x = blue_hero_x + 2 * offset_;
static constexpr uint16_t blue_infantry_IV_x  = blue_hero_x + 3 * offset_;
static constexpr uint16_t blue_infantry_V_x   = blue_hero_x + 4 * offset_;
static constexpr uint16_t blue_sentry_x       = blue_hero_x + 5 * offset_;
static constexpr uint16_t robot_y             = 900;
} // namespace rmcs_core::referee::app::ui