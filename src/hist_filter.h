#include <vector>
#include <opencv2/opencv.hpp>

void blank_filter(const cv::Mat1s &hist_v, std::vector<cv::Point3i> &list, int threshold, int horizon_filter = 0);
