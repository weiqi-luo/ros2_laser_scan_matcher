#pragma once

#include <deque>
#include <string>
#include <vector>

class FilterBase {
 public:
  explicit FilterBase(double max_value = 1.0);
  double update(double time, double current_value);
  virtual void reset() { is_initialized_ = false; }

 protected:
  virtual double filterImpl(double time, double current_value) = 0;

  double max_value_;
  bool is_initialized_ = false;
};

class LowPassFilter : public FilterBase {
 public:
  explicit LowPassFilter(double alpha = 0.2, double max_value = 1.0);

 protected:
  double filterImpl(double time, double current_value) override;

 private:
  double alpha_;
  double filtered_value_{0.0};
  double last_time_{0.0};
};

class MovingAverageFilter : public FilterBase {
 public:
  explicit MovingAverageFilter(double time_window = 1.0, double max_value = 1.0);
  void reset() override {
    FilterBase::reset();
    buffer_.clear();
  }

 protected:
  double filterImpl(double time, double current_value) override;

 private:
  std::deque<std::pair<double, double>> buffer_;
  double time_window_;
};

class MedianFilter : public FilterBase {
 public:
  explicit MedianFilter(double time_window = 1.0, double max_value = 1.0);
  void reset() override {
    FilterBase::reset();
    buffer_.clear();
  }

 protected:
  double filterImpl(double time, double current_value) override;

 private:
  std::deque<std::pair<double, double>> buffer_;
  double time_window_;
  static double getMedian(std::vector<double>& values);
};