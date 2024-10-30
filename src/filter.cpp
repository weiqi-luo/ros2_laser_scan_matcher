#include "ros2_laser_scan_matcher/filter.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

FilterBase::FilterBase(double max_value) : max_value_(max_value) {}

double FilterBase::update(double time, double current_value) {
  if (!is_initialized_) {
    is_initialized_ = true;
    return std::clamp(current_value, -max_value_, max_value_);
  }
  return std::clamp(filterImpl(time, current_value), -max_value_, max_value_);
}

// Low-pass filter
LowPassFilter::LowPassFilter(double alpha, double max_value)
    : FilterBase(max_value), alpha_(alpha) {}

double LowPassFilter::filterImpl(double time, double current_value) {
  if (last_time_ != 0) {
    // Calculate time difference in seconds
    double dt = time - last_time_;

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
MovingAverageFilter::MovingAverageFilter(double time_window, double max_value)
    : FilterBase(max_value), time_window_(time_window) {}

double MovingAverageFilter::filterImpl(double time, double current_value) {
  buffer_.emplace_back(time, current_value);

  while (!buffer_.empty() && (time - buffer_.front().first) > time_window_) {
    buffer_.pop_front();
  }

  double sum = 0.0;
  double total_weight = 0.0;

  for (const auto& [sample_time, value] : buffer_) {
    double dt = time - sample_time;
    double weight = std::exp(-dt / time_window_);
    sum += value * weight;
    total_weight += weight;
  }

  return total_weight > 0.0 ? sum / total_weight : current_value;
}

// Median filter
MedianFilter::MedianFilter(double time_window, double max_value)
    : FilterBase(max_value), time_window_(time_window) {}

double MedianFilter::filterImpl(double time, double current_value) {
  buffer_.emplace_back(time, current_value);

  while (!buffer_.empty() && (time - buffer_.front().first) > time_window_) {
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
