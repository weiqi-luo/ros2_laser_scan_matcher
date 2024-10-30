#pragma once

#include <deque>
#include <memory>
#include <rclcpp/rclcpp.hpp>

class FilterBase {
 public:
  explicit FilterBase(const rclcpp::Node::SharedPtr& node, const std::string& filter_name);
  double update(const rclcpp::Time& time, double current_value);
  virtual void reset() { is_initialized_ = false; }

 protected:
  virtual double filterImpl(const rclcpp::Time& time, double current_value) = 0;

  rclcpp::Node::SharedPtr node_;
  std::string filter_name_;
  double max_value_ = 1.0;
  bool is_initialized_ = false;
};

class LowPassFilter : public FilterBase {
 public:
  explicit LowPassFilter(const rclcpp::Node::SharedPtr& node);

 protected:
  double filterImpl(const rclcpp::Time& time, double current_value) override;

 private:
  double alpha_ = 0.2;
  double filtered_value_{0.0};
  rclcpp::Time last_time_{};
};

class MovingAverageFilter : public FilterBase {
 public:
  explicit MovingAverageFilter(const rclcpp::Node::SharedPtr& node);
  void reset() override {
    FilterBase::reset();
    buffer_.clear();
  }

 protected:
  double filterImpl(const rclcpp::Time& time, double current_value) override;

 private:
  std::deque<std::pair<rclcpp::Time, double>> buffer_;
  double time_window_ = 1.0;
};

class MedianFilter : public FilterBase {
 public:
  explicit MedianFilter(const rclcpp::Node::SharedPtr& node);
  void reset() override {
    FilterBase::reset();
    buffer_.clear();
  }

 protected:
  double filterImpl(const rclcpp::Time& time, double current_value) override;

 private:
  std::deque<std::pair<rclcpp::Time, double>> buffer_;
  double time_window_ = 1.0;
  static double getMedian(std::vector<double>& values);
};