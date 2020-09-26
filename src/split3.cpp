#include "split3.h"

const cv::Vec3b roadColor = {0, 255, 0};
const cv::Vec3b posColor = {0, 0, 255};
const cv::Vec3b negColor = {255, 0, 0};

cv::Mat3b split3(cv::Mat input, double k, double b,double delta)
{
    cv::Mat3b result = cv::Mat3b::zeros(input.size());
    for (int i = 0; i < input.rows; i++)
    {
        for (int j = 0; j < input.cols; j++)
        {
            double d_top = (i + delta - b) / k;
            double d_bottom = (i - delta - b) / k;
            uint8_t color = input.at<uint8_t>(i, j);
            if (color == 0)
            {
                result.at<cv::Vec3b>(i, j) = {color, color, color};
                continue;
            }
            else if (d_top >= color && d_bottom < color)
            {
                result.at<cv::Vec3b>(i, j) = roadColor;
            }
            else if (d_bottom < color)
            {
                result.at<cv::Vec3b>(i, j) = posColor;
            }
            else if (d_top > color)
            {
                result.at<cv::Vec3b>(i, j) = negColor;
            }
        }
    }
    return result;
}
