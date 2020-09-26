#pragma once
#include <vector>
#include <opencv2/opencv.hpp>

void ransac(const std::vector<cv::Point3i> &points, double &k, double &b, size_t times, double delta);
