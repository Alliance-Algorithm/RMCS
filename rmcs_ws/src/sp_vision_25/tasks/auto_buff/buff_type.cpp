#include "buff_type.hpp"

#include <algorithm>

#include "tools/logger.hpp"
namespace auto_buff
{
FanBlade::FanBlade(
  const std::vector<cv::Point2f> & kpt, cv::Point2f keypoints_center, FanBlade_type t)
: center(keypoints_center), type(t)
{
  points.insert(points.end(), kpt.begin(), kpt.end());
}

FanBlade::FanBlade(FanBlade_type t) : type(t)
{
  if (t != _unlight) exit(-1);
}

PowerRune::PowerRune(
  std::vector<FanBlade> & ts, const cv::Point2f center, std::optional<PowerRune> last_powerrune)
: r_center(center), light_num(ts.size())
{
  /// 找出target

  // 只有一个fanblade，就为target
  if (light_num == 1) ts[0].type = _target;
  // 没有新亮起来的fanblade
  else if (last_powerrune.has_value() && ts.size() == last_powerrune.value().light_num) {
    auto last_target_center = last_powerrune.value().fanblades[0].center;
    auto target_fanblade_it = ts.begin();  // 初始化为 fanblades 的第一个元素
    float min_distance = norm(ts[0].center - last_target_center);
    for (auto it = ts.begin(); it != ts.end(); ++it) {
      float distance = norm(it->center - last_target_center);
      if (distance < min_distance) {
        min_distance = distance;
        target_fanblade_it = it;  // 更新最近的 fanblade 的迭代器
      }
    }
    target_fanblade_it->type = _target;  // 设置最近的 fanblade 的 type
    std::iter_swap(ts.begin(), target_fanblade_it);
  }
  // 有新亮起来的fanblade
  else if (last_powerrune.has_value() && light_num == last_powerrune.value().light_num + 1) {
    auto last_fanblades = last_powerrune.value().fanblades;
    float max_min_distance = -1.0f;        // 初始化最大最小距离为-1
    auto target_fanblade_it = ts.begin();  // 用于存储目标 fanblade 的迭代器
    for (auto it = ts.begin(); it != ts.end(); ++it) {
      float min_distance = std::numeric_limits<float>::max();  // 初始化最小距离为最大浮点数
      // 计算当前 fanblade 到 last_fanblades 中每个 fanblade 的最小距离
      for (const auto & last_fanblade : last_fanblades) {
        if (last_fanblade.type == _unlight) continue;
        float distance = norm(it->center - last_fanblade.center);
        if (distance < min_distance) {
          min_distance = distance;
        }
      }
      if (min_distance > max_min_distance) {
        max_min_distance = min_distance;
        target_fanblade_it = it;
      }
    }
    target_fanblade_it->type = _target;
    std::iter_swap(ts.begin(), target_fanblade_it);
  }
  // error
  else {
    tools::logger()->debug("[PowerRune] 识别出错!");
    unsolvable_ = true;
    return;
  }

  /// 填充FanBlade.angle

  double angle = atan_angle(ts[0].center);
  for (auto & t : ts) {
    t.angle = atan_angle(t.center) - angle;
    if (t.angle < -1e-3) t.angle += CV_2PI;
  }

  /// fanblades调整顺序

  std::sort(ts.begin(), ts.end(), [](const FanBlade & a, const FanBlade & b) {
    return a.angle < b.angle;
  });  // 按照 t.angle 从小到大排序 ts
  const std::vector<double> target_angles = {
    0, 2.0 * CV_PI / 5.0, 4.0 * CV_PI / 5.0, 6.0 * CV_PI / 5.0, 8.0 * CV_PI / 5.0};
  for (int i = 0, j = 0; i < 5 && j < ts.size(); i++) {
    if (std::fabs(ts[j].angle - target_angles[i]) < CV_PI / 5.0)
      fanblades.emplace_back(ts[j++]);
    else
      fanblades.emplace_back(FanBlade(_unlight));
  }
};

double PowerRune::atan_angle(cv::Point2f point) const
{
  auto v = point - r_center;
  auto angle = std::atan2(v.y, v.x);
  return angle >= 0 ? angle : angle + CV_2PI;
}
}  // namespace auto_buff
