// Copyright 2022 Chen Jun

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/core/mat.hpp>

// STL
#include <algorithm>
#include <chrono>
#include <iostream>
#include <map>
#include <vector>

#include "armor_detector/number_classifier.hpp"

using hrc = std::chrono::high_resolution_clock;

TEST(test_nc, benchmark)
{
  auto pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
  auto model_path = pkg_path + "/model/mlp.onnx";
  auto label_path = pkg_path + "/model/label.txt";
  rm_auto_aim::NumberClassifier nc(model_path, label_path, 0.5);

  auto dummy_armors = std::vector<rm_auto_aim::Armor>(1);
  auto test_mat = cv::Mat(20, 28, CV_8UC1);
  dummy_armors[0].number_img = test_mat;

  int loop_num = 200;
  int warm_up = 30;

  double time_min = DBL_MAX;
  double time_max = -DBL_MAX;
  double time_avg = 0;

  for (int i = 0; i < warm_up + loop_num; i++) {
    auto start = hrc::now();
    nc.classify(dummy_armors);
    auto end = hrc::now();
    double time = std::chrono::duration<double, std::milli>(end - start).count();
    if (i >= warm_up) {
      time_min = std::min(time_min, time);
      time_max = std::max(time_max, time);
      time_avg += time;
    }
  }
  time_avg /= loop_num;

  std::cout << "time_min: " << time_min << "ms" << std::endl;
  std::cout << "time_max: " << time_max << "ms" << std::endl;
  std::cout << "time_avg: " << time_avg << "ms" << std::endl;
}
