#include "ros2_laser_scan_matcher/filter.hpp"

#include <algorithm>
#include <numeric>

FilterBase::FilterBase(const rclcpp::Node::SharedPtr& node, const std::string& filter_name)
    : node_(node) {
  max_value_ = node_->declare_parameter("filter.max_value", max_value_);
}

double FilterBase::update(const rclcpp::Time& time, double current_value) {
  if (!is_initialized_) {
    is_initialized_ = true;
    return std::clamp(current_value, -max_value_, max_value_);
  }
  return std::clamp(filterImpl(time, current_value), -max_value_, max_value_);
}

// Low-pass filter
LowPassFilter::LowPassFilter(const rclcpp::Node::SharedPtr& node) : FilterBase(node, "low_pass") {
  alpha_ = node_->declare_parameter("filter.low_pass.alpha", alpha_);
}

double LowPassFilter::filterImpl(const rclcpp::Time& time, double current_value) {
  if (last_time_.nanoseconds() != 0) {
    // Calculate time difference in seconds
    double dt = (time - last_time_).seconds();

    // Adjust alpha based on time difference
    // alpha = 1 - e^(-dt/tau), where tau is the time constant
    double effective_alpha = 1.0 - std::exp(-dt / alpha_);

    filtered_value_ = effective_alpha * current_value + (1.0 - effective_alpha) * filtered_value_;
  } else {
    // First measurement
    filtered_value_ = current_value;
  }

  last_time_ = time;
  return filtered_value_;
}

// Moving average filter
MovingAverageFilter::MovingAverageFilter(const rclcpp::Node::SharedPtr& node)
    : FilterBase(node, "moving_average") {
  time_window_ = node_->declare_parameter("filter.moving_average.time_window", time_window_);
}

double MovingAverageFilter::filterImpl(const rclcpp::Time& time, double current_value) {
  buffer_.emplace_back(time, current_value);

  while (!buffer_.empty() && (time - buffer_.front().first).seconds() > time_window_) {
    buffer_.pop_front();
  }

  double sum = 0.0;
  double total_weight = 0.0;

  for (const auto& [sample_time, value] : buffer_) {
    double dt = (time - sample_time).seconds();
    double weight = std::exp(-dt / time_window_);
    sum += value * weight;
    total_weight += weight;
  }

  return total_weight > 0.0 ? sum / total_weight : current_value;
}

// Median filter
MedianFilter::MedianFilter(const rclcpp::Node::SharedPtr& node) : FilterBase(node, "median") {
  time_window_ = node_->declare_parameter("filter.median.time_window", time_window_);
}

double MedianFilter::filterImpl(const rclcpp::Time& time, double current_value) {
  buffer_.emplace_back(time, current_value);

  while (!buffer_.empty() && (time - buffer_.front().first).seconds() > time_window_) {
    buffer_.pop_front();
  }

  std::vector<double> values;
  values.reserve(buffer_.size());
  for (const auto& [_, value] : buffer_) {
    values.push_back(value);
  }

  return getMedian(values);
}

double MedianFilter::getMedian(std::vector<double>& values) {
  if (values.empty()) {
    return 0.0;
  }

  const size_t size = values.size();
  const size_t middle = size / 2;

  // Partially sort the array up to the middle element
  std::nth_element(values.begin(), values.begin() + middle, values.end());

  if (size % 2 == 0) {
    // For even number of elements, we need the (middle-1)th element too
    auto max_it = std::max_element(values.begin(), values.begin() + middle);
    return (*max_it + values[middle]) / 2.0;
  } else {
    // For odd number of elements, middle element is the median
    return values[middle];
  }
}
