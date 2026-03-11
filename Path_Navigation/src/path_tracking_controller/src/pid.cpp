#include "path_tracking_controller/pid.hpp"

PID::PID(double dt, double max, double min, double kp, double kd, double ki)
: dt_(dt), max_(max), min_(min), kp_(kp), kd_(kd), ki_(ki),
  pre_error_(0.0), integral_(0.0), pre_derivative_(0.0)
{
}

double PID::calculate(double set_point, double pv)
{
  // 计算当前误差
  double error = set_point - pv;
  
  // 计算微分项（使用一阶低通滤波）
  double derivative = (error - pre_error_) / dt_;
  derivative = 0.85 * derivative + 0.15 * pre_derivative_;  // 低通滤波系数
  
  // 使用积分限幅防止积分饱和
  double new_integral = integral_ + error * dt_;
  if (new_integral > max_) {
    new_integral = max_;
  } else if (new_integral < min_) {
    new_integral = min_;
  }
  integral_ = new_integral;
  
  // 计算PID输出
  double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
  
  // 更新历史值
  pre_error_ = error;
  pre_derivative_ = derivative;
  
  // 输出限幅
  if (output > max_) {
    output = max_;
  } else if (output < min_) {
    output = min_;
  }
  
  return output;
}

void PID::setSumError(double sum_error)
{
  integral_ = sum_error;
}

PID::~PID() = default;
