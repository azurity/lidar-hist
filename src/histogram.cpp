#include "histogram.h"
using namespace cv;

void uv_histogram(const Mat &img, Mat1s &u, Mat1s &v)
{
    u = Mat1s::zeros(256, img.cols);
    v = Mat1s::zeros(img.rows, 256);
    const size_t size = img.rows * img.cols;
    for (int i = 0; i < img.rows; i++)
    {
        for (int j = 0; j < img.cols; j++)
        {
            int p = img.at<uint8_t>(i, j);
            u.at<short>(p, j)++;
            v.at<short>(i, p)++;
        }
    }
    return;
}
