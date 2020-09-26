#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include "projectPointCloud.h"
#include "histogram.h"
#include "hist_filter.h"
#include "ransac.h"
#include "split3.h"

#include <chrono>

#define SHOW

pcl::PointCloud<pcl::PointXYZI>::Ptr readBinPointCloud(const std::string &filename)
{
    std::ifstream ifs(filename, std::ios::in | std::ios::binary);
    // if (!ifs.good())
    // {
    //     // error
    // }
    pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>);
    while (!ifs.eof())
    {
        float data[4];
        pcl::PointXYZI point;
        ifs.read((char *)&data, sizeof(float) * 4);
        point.x = data[0];
        point.y = data[1];
        point.z = data[2];
        result->push_back(point);
    }
    ifs.close();
    return result;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorfulPointCloud(pcl::PointCloud<pcl::PointXYZI>::ConstPtr laserCloudIn, const LidarArgs &lidarArgs, const cv::Mat3b &colorMat)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);
    float verticalAngle, horizonAngle, range;
    size_t rowIdn, columnIdn, index, cloudSize;
    pcl::PointXYZRGB thisPoint;

    cloudSize = laserCloudIn->points.size();

    for (size_t i = 0; i < cloudSize; ++i)
    {

        thisPoint.x = laserCloudIn->points[i].x;
        thisPoint.y = laserCloudIn->points[i].y;
        thisPoint.z = laserCloudIn->points[i].z;
        verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
        rowIdn = (verticalAngle + lidarArgs.angBottom) / lidarArgs.angResY;
        if (rowIdn < 0 || rowIdn >= lidarArgs.nScan)
            continue;

        horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

        columnIdn = -round((horizonAngle - 90.0) / lidarArgs.angResX) + lidarArgs.horizonScan / 2;
        if (columnIdn >= lidarArgs.horizonScan)
            columnIdn -= lidarArgs.horizonScan;

        if (columnIdn < 0 || columnIdn >= lidarArgs.horizonScan)
            continue;

        range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
        if (range < sensorMinimumRange)
            continue;

        auto color = colorMat.at<cv::Vec3b>(lidarArgs.nScan - 1 - rowIdn, columnIdn);
        thisPoint.r = color[2];
        thisPoint.g = color[1];
        thisPoint.b = color[0];
        result->push_back(thisPoint);
    }
    return result;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr readKitti(const std::string &filename)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>);
    auto pc_ori = readBinPointCloud(filename);
    Eigen::Matrix4f Tr;
    Tr << 4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03, 0,
        -7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01, 0,
        9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03, 0,
        0, 0, 0, 1;
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.rotate(Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitY()));
    Tr *= transform_2.matrix();
    pcl::transformPointCloud(*pc_ori, *pc, Tr);
    return pc;
}

int main(int, char **)
{
    auto pc = readKitti("../004000.bin");
    auto lidarArg = lidarArgsMap.at("HDL-64E");
    cv::Mat disp = projectPointCloud(pc, lidarArg, [](double q) { return q < M_PI_4 && q > -M_PI_4; });
    cv::Mat1s uHist, vHist;

    uv_histogram(disp, uHist, vHist);
    std::vector<cv::Point3i> pList;
    auto filter = lidarArg.nScan - lidarArg.angBottom / lidarArg.angResY;
    blank_filter(vHist, pList, 5, filter);
    double k, b;
    ransac(pList, k, b, 1000, 2);
    auto result = split3(disp, k, b, 3);

#ifdef SHOW
    cv::imshow("lidar", disp);

    cv::Mat vShow = vHist.clone();
    vShow.col(0).setTo(0);
    cv::normalize(vShow, vShow, 255, 0, cv::NORM_MINMAX);
    vShow.convertTo(vShow, CV_8U);

    cv::Mat1b lineShow;
    cv::resize(vShow, lineShow, cv::Size(), 4, 4, cv::INTER_NEAREST);
    cv::line(lineShow, cv::Point(0, b) * 4, cv::Point((lineShow.rows - 1 - b) / k, lineShow.rows - 1) * 4, cv::Scalar(255));
    cv::imshow("ransac-result", lineShow);

    cv::imshow("result", result);
    pcl::visualization::CloudViewer viewer("pointcloud of result");

    viewer.showCloud(colorfulPointCloud(pc, lidarArg, result));

    cv::waitKey();
    while (!viewer.wasStopped())
        ;
#endif
}
