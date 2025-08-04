#ifndef PATH_TRACKING_CONTROLLER_CONTROLLER__PID_HPP_
#define PATH_TRACKING_CONTROLLER_CONTROLLER__PID_HPP_

class PID
{
public:
  // KP - 比例增益
  // ki - 积分增益
  // KD - 衍生收益
  // dt - 循环间隔时间
  // max -纵变量的最大值
  // min -纵变量的最小值
  PID(double dt, double max, double min, double kp, double kd, double ki);

  // 返回给定 set_point 和当前进程值的纵变量
  double calculate(double set_point, double pv);
  void setSumError(double sum_error);
  ~PID();

private:
  double dt_;           // 控制周期
  double max_;          // 输出最大值
  double min_;          // 输出最小值
  double kp_;           // 比例系数
  double kd_;           // 微分系数
  double ki_;           // 积分系数
  double pre_error_;    // 上一次误差
  double integral_;     // 积分项
  double pre_derivative_;  // 上一次微分值（用于低通滤波）
};

#endif  // PATH_TRACKING_CONTROLLER_CONTROLLER__PID_HPP_
