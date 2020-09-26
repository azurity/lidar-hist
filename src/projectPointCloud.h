#include <string>
#include <map>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <functional>

struct LidarArgs
{
    int nScan;
    int horizonScan;
    float angResX;
    float angResY;
    float angBottom;
};

extern const std::map<std::string, LidarArgs> lidarArgsMap;
extern const float sensorMinimumRange;

cv::Mat projectPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn, const LidarArgs &lidarArgs, const std::function<bool(double)> &filter);
