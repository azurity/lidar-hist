#include "hist_filter.h"
using namespace std;
using namespace cv;

void blank_filter(const Mat1s &hist_v, vector<Point3i> &list, int threshold, int horizon_filter)
{
    for (int i = horizon_filter; i < hist_v.rows; i++)
    {
        for (int j = 1; j < hist_v.cols; j++)
        {
            if (hist_v.at<short>(i, j) > threshold)
            {
                list.push_back(Point3i(j, i, hist_v.at<int16_t>(i, j)));
            }
        }
    }
    return;
}
