#ifndef BUFF__PREDICT_HPP
#define BUFF__PREDICT_HPP

#include <algorithm>
#include <deque>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "tools/extended_kalman_filter.hpp"
#include "tools/img_tools.hpp"
#include "tools/plotter.hpp"
const double SMALL_W = CV_PI / 3;

// Predictor 基类
class Predictor
{
public:
  Predictor(){};
  virtual void update(double angle, double nowtime) = 0;  // 纯虚函数
  virtual double predict(double delta_time) = 0;          // 纯虚函数
  virtual bool is_unsolve() const = 0;                    // 纯虚函数
  virtual Eigen::VectorXd getX_best() const = 0;          // 纯虚函数
protected:
  Eigen::VectorXd x0;
  Eigen::MatrixXd P0;
  Eigen::MatrixXd A;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd H;
  Eigen::MatrixXd R;
  tools::ExtendedKalmanFilter ekf;
  Eigen::VectorXd X_best;
  double lastangle = 0;
  double lasttime = 0;
  int cw_ccw = 0;  // 逆时针-1 顺时针1
  bool first_in = true;
  bool unsolvable = true;
};

class Small_Predictor : public Predictor
{
public:
  Small_Predictor()
  : Predictor()  //, x0(Eigen::VectorXd(1)), P0(1, 1), A(1, 1), Q(1, 1), H(1, 1), R(1, 1)
  {
    // 初始状态协方差矩阵
    x0.resize(1);
    P0.resize(1, 1);
    A.resize(1, 1);
    Q.resize(1, 1);
    H.resize(1, 1);
    R.resize(1, 1);
    // 初始状态
    x0 << 0.0;  // 初始角度为 0，初始角速度为 1
    // 初始状态协方差矩阵
    P0 << 1.0;
    // 状态转移矩阵
    A << 1.0;
    // 过程噪声协方差矩阵                            //// 调整
    Q << 0.0;
    // 测量方程矩阵
    H << 1.0;
    // 测量噪声协方差矩阵                            //// 调整
    R << 0.05;
    // 创建扩展卡尔曼滤波器对象
    ekf = tools::ExtendedKalmanFilter(x0, P0);
  }

  virtual void update(double angle, double nowtime) override
  {
    // [angle]
    // [w    ]   w=CV_PI/6

    // 初始化angle
    if (first_in) {
      first_in = false;
      lasttime = nowtime;
      lastangle = angle;
      ekf.x[0] = angle;
      unsolvable = true;
    }

    // 处理扇叶跳变
    if (abs(angle - lastangle) > CV_PI / 12) {
      for (int i = -5; i <= 5; i++) {
        double angle_c = lastangle + i * 2 * CV_PI / 5;
        if (std::fabs(angle_c - angle) < CV_PI / 5) {
          ekf.x[0] += i * 2 * CV_PI / 5;
          break;
        }
      }
    }

    // 判断是顺时针还是逆时针旋转
    if (abs(cw_ccw) < 100) {
      if (lastangle > angle)
        cw_ccw -= 1;
      else
        cw_ccw += 1;
    }

    // 预测下一个状态
    double deltatime = nowtime - lasttime;
    A << 1.0;
    Eigen::VectorXd B(1);
    B << SMALL_W * (cw_ccw > 0 ? 1 : -1);
    ekf.predict(A, Q, [&](const Eigen::VectorXd & x) { return A * x + deltatime * B; });

    // 更新状态，假设测量到的角度为当前状态的第一个元素
    Eigen::VectorXd z(1);
    z << angle;
    X_best = ekf.update(z, H, R);

    // 更新lasttime lastangle
    lasttime = nowtime;
    lastangle = angle;

#ifdef PLOTJUGGLER
    nlohmann::json json_obj;
    json_obj["angle"] = X_best[0] * 180 / CV_PI;
    tools::Plotter().plot(json_obj);
#endif
    unsolvable = false;
    return;
  }

  virtual double predict(double delta_time) override
  {
    if (unsolvable)
      return 0;
    else
      return (cw_ccw > 0 ? 1 : -1) * SMALL_W * delta_time;
  }

  virtual bool is_unsolve() const { return unsolvable; }

  virtual Eigen::VectorXd getX_best() const { return X_best; }
};

class Big_Predictor : public Predictor
{
public:
  Big_Predictor()
  : Predictor()  //,  x0(Eigen::VectorXd(5)), P0(5, 5), A(5, 5), Q(5, 5), H(1, 5), R(1, 1)
  {
    // [angle
    //  spd
    //  a       0.78-1.045
    //  w       1.884-2.000
    //  sita ]

    // 初始状态协方差矩阵
    x0.resize(5);
    P0.resize(5, 5);
    A.resize(5, 5);
    Q.resize(5, 5);
    H.resize(1, 5);
    R.resize(1, 1);

    // 初始状态
    x0 << 0.0, 1.1775, 0.9125, 1.942, 0.0;  // 初始角度为 0，初始角速度为 1,a:0,w:0
    // x0 << 0.0, 1.32, 0.78, 1.884, 0.0; // 初始角度为 0，初始角速度为 1,a:0,w:0
    // x0 << 0.0, 1.045, 1.045, 2.0, 0.0; // 初始角度为 0，初始角速度为 1,a:0,w:0
    // 初始状态协方差矩阵
    P0 << 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01;

    // 状态转移矩阵
    // A << 1.0, 1.0,
    //     0.0, 1.0;
    // 过程噪声协方差矩阵                            //// 调整
    Q << 0.03, 0.0, 0.0, 0.0, 0.0,  // 0.01-0.03
      0.0, 0.05, 0.0, 0.0, 0.0,     //
      0.0, 0.0, 0.01, 0.0, 0.0,     //
      0.0, 0.0, 0.0, 0.01, 0.0,     //
      0.0, 0.0, 0.0, 0.0, 0.05;     //
    //
    // Q << 0.01, 0.0, 0.0, 0.0, 0.0,
    //     0.0, 0.01, 0.0, 0.0, 0.0,
    //     0.0, 0.0, 0.01, 0.0, 0.0,
    //     0.0, 0.0, 0.0, 0.01, 0.0,
    //     0.0, 0.0, 0.0, 0.0, 0.0,
    // 测量方程矩阵
    H << 1.0, 0.0, 0.0, 0.0, 0.0;
    // 测量噪声协方差矩阵                            //// 调整
    R << 0.05;
    // 创建扩展卡尔曼滤波器对象
    ekf = tools::ExtendedKalmanFilter(x0, P0);
  }

  virtual void update(double angle, double nowtime) override
  {
    // 初始化angle
    if (first_in) {
      first_in = false;
      lasttime = nowtime;
      lastangle = angle;
      ekf.x[0] = angle;
      unsolvable = true;
      return;
    }

    // 处理扇叶跳变
    if (abs(angle - lastangle) > CV_PI / 12) {
      for (int i = -5; i <= 5; i++) {
        double angle_c = lastangle + i * 2 * CV_PI / 5;
        if (std::fabs(angle_c - angle) < CV_PI / 5) {
          ekf.x[0] += i * 2 * CV_PI / 5;
          lastangle += i * 2 * CV_PI / 5;
          break;
        }
      }
    }

    // 判断是顺时针还是逆时针旋转
    if (abs(cw_ccw) < 100) {
      if (lastangle > angle)
        cw_ccw -= 1;
      else
        cw_ccw += 1;
    }

    // 预测下一个状态
    double deltatime = nowtime - lasttime;
    double a = ekf.x[2];
    double w = ekf.x[3];
    double sita = ekf.x[4];
    A << 1.0, 0.0,
      (cw_ccw > 0 ? 1 : -1) * (-1 / w * cos(sita + w * deltatime) + 1 / w * cos(sita) - deltatime),
      (cw_ccw > 0 ? 1 : -1) * (a / (w * w) * cos(sita + w * deltatime) - a / (w * w) * cos(sita)),
      (cw_ccw > 0 ? 1 : -1) * (a / w * sin(sita + w * deltatime) - a / w * sin(sita)), 0.0, 0.0,
      sin(sita + w * deltatime) - 1, deltatime * a * cos(sita + w * deltatime),
      a * cos(sita + w * deltatime), 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
      0.0, deltatime, 1.0;
    ekf.predict(A, Q, [&](const Eigen::VectorXd & x) {
      Eigen::VectorXd m(5);  //a w sita
      m << x[0] + (cw_ccw > 0 ? 1 : -1) * (-a / w * cos(sita + w * deltatime) + a / w * cos(sita) +
                                           (2.09 - a) * deltatime),
        a * sin(sita + w * deltatime) + 2.09 - a, a, w, sita + w * deltatime;
      return m;
    });

    // 更新状态
    Eigen::VectorXd z(1);
    z << angle;
    X_best = ekf.update(z, H, R);

    // 更新lasttime lastangle
    lasttime = nowtime;
    lastangle = angle;
    unsolvable = false;

#ifndef PLOTJUGGLER
    nlohmann::json json_obj;
    json_obj["angle"] = X_best[0] * 180 / CV_PI;
    json_obj["spd"] = X_best[1] * 180 / CV_PI;
    json_obj["a"] = X_best[2];
    json_obj["w"] = X_best[3];
    json_obj["theta"] = X_best[4];
    tools::Plotter().plot(json_obj);
#endif
  }

  virtual double predict(double delta_time) override
  {
    if (unsolvable) return 0;
    double a = X_best[2];
    double w = X_best[3];
    double sita = X_best[4];
    return (cw_ccw > 0 ? 1 : -1) *
           (-a / w * cos(sita + w * delta_time) + a / w * cos(sita) + (2.09 - a) * delta_time);
  }

  virtual bool is_unsolve() const { return unsolvable; }

  virtual Eigen::VectorXd getX_best() const { return X_best; }
};

class XYZ_predictor
{
public:
  Eigen::VectorXd x0;
  Eigen::MatrixXd P0;
  Eigen::MatrixXd A;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd H;
  Eigen::MatrixXd R;
  tools::ExtendedKalmanFilter ekf;
  Eigen::VectorXd X_best;
  XYZ_predictor()
  : x0(Eigen::VectorXd(3)),
    P0(Eigen::MatrixXd(3, 3)),
    A(Eigen::MatrixXd(3, 3)),
    Q(Eigen::MatrixXd(3, 3)),
    H(Eigen::MatrixXd(3, 3)),
    R(Eigen::MatrixXd(3, 3))
  {
    // 初始状态
    x0 << 0.0, 0.0, 7.0;  // 初始x为 0，初始角y为 0, 初始角z为 7m
    // 初始状态协方差矩阵
    P0 << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
    // 状态转移矩阵
    A << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
    // 过程噪声协方差矩阵                            //// 调整
    Q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    // 测量方程矩阵
    H << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
    // 测量噪声协方差矩阵                            //// 调整
    R << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
    // 创建扩展卡尔曼滤波器对象
    ekf = tools::ExtendedKalmanFilter(x0, P0);
  }

  void kalman(Eigen::Vector3d & XYZ)
  {
    if (first_in) {
      first_in = false;
      ekf.x[0] = XYZ[0];
      ekf.x[1] = XYZ[1];
      ekf.x[2] = XYZ[2];
      return;
    }
    // 预测下一个状态
    ekf.predict(A, Q);  // 预测的角度和角速度

    // 更新状态
    Eigen::VectorXd z(3);
    z << XYZ[0], XYZ[1], XYZ[2];
    X_best = ekf.update(z, H, R);

#ifdef PLOTJUGGLER
    nlohmann::json json_obj;
    json_obj["x"] = X_best[0];
    json_obj["y"] = X_best[1];
    json_obj["z"] = X_best[2];
    tools::Plotter().plot(json_obj);
#endif
    XYZ = X_best;
  }

private:
  bool first_in = true;
};
#endif  // BUFF_PREDICT_HPP
