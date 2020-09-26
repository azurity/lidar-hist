#include "projectPointCloud.h"
#include <cmath>

const std::map<std::string, LidarArgs> lidarArgsMap = {
    {"VLP-16", {16, 1800, 0.2, 2.0, 15.0 + 0.1}},
    {"HDL-32E", {32, 1800, 360.0 / 1800, 41.33 / 31, 30.67}},
    {"HDL-64E", {64, 1800, 0.2, 0.4, 24.8}}};

const float sensorMinimumRange = 2.5;

cv::Mat projectPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn, const LidarArgs &lidarArgs, const std::function<bool(double)> &filter)
{
    cv::Mat rangeMat = cv::Mat1f::zeros(lidarArgs.nScan, lidarArgs.horizonScan);
    // range image projection
    float verticalAngle, horizonAngle, range;
    float p, q;
    size_t rowIdn, columnIdn, index, cloudSize;
    pcl::PointXYZI thisPoint;

    cloudSize = laserCloudIn->points.size();

    for (size_t i = 0; i < cloudSize; ++i)
    {

        thisPoint.x = laserCloudIn->points[i].x;
        thisPoint.y = laserCloudIn->points[i].y;
        thisPoint.z = laserCloudIn->points[i].z;
        p = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y));
        verticalAngle = p * 180 / M_PI;
        rowIdn = (verticalAngle + lidarArgs.angBottom) / lidarArgs.angResY;

        if (rowIdn < 0 || rowIdn >= lidarArgs.nScan)
            continue;

        q = atan2(thisPoint.x, thisPoint.y);

        if (!filter(q))
            continue;

        horizonAngle = q * 180 / M_PI;

        columnIdn = -round((horizonAngle - 90.0) / lidarArgs.angResX) + lidarArgs.horizonScan / 2;
        if (columnIdn >= lidarArgs.horizonScan)
            columnIdn -= lidarArgs.horizonScan;

        if (columnIdn < 0 || columnIdn >= lidarArgs.horizonScan)
            continue;

        range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
        if (range < sensorMinimumRange)
            continue;

        rangeMat.at<float>(rowIdn, columnIdn) = 1.0 / (range * cos(p) * std::max(abs(sin(q)), abs(cos(q))));
    }
    rangeMat *= 0.5;
    rangeMat.convertTo(rangeMat, CV_8U, 255);
    cv::flip(rangeMat, rangeMat, 0);
    return rangeMat;
}
