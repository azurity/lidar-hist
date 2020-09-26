#include <cstdlib>
#include <cstdint>
#include "ransac.h"
using namespace std;
using namespace cv;

void ransac(const vector<Point3i> &points, double &k, double &b, size_t times, double delta)
{
    double min_error = UINT32_MAX;
    for (size_t i = 0; i < times; i++)
    {
        int m = rand() % points.size();
        int n = rand() % points.size();
        double lk = (points[m].y - points[n].y) / (double)(points[m].x - points[n].x);
        double lb = points[m].y - points[m].x * lk;
        if (!isnormal(lk) || !isnormal(lb))
            continue;
        double error = 0;
        for (auto &p : points)
        {
            if (abs(p.x * lk + lb - p.y) > delta)
            {
                error += p.z;
                // error++;
            }
        }
        if (error < min_error)
        {
            min_error = error;
            k = lk;
            b = lb;
        }
    }
    return;
}
