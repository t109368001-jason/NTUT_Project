#ifndef BACKGROUNDANDDYNAMIC_H_
#define BACKGROUNDANDDYNAMIC_H_

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "velodyne/pcap_cache.h"
#include "basic_function.h"
#include "function.h"

using namespace std;

namespace myFunction
{
  template <typename PointT>
  typename pcl::PointCloud<PointT>::Ptr getDynamicStatisticalOutlierRemoval(const typename pcl::PointCloud<PointT>::Ptr &background, const typename pcl::PointCloud<PointT>::Ptr &originCloud, const int &meanKNumber, const double &stddevMulThreshNumber)
  {
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    boost::shared_ptr<pcl::StatisticalOutlierRemoval<pcl::PointXYZ>> sor(new pcl::StatisticalOutlierRemoval<pcl::PointXYZ>());

    sor->setInputCloud(myFunction::getChanges<pcl::PointXYZ>(background, originCloud, 50.0));
    sor->setMeanK(meanKNumber);
    sor->setStddevMulThresh(stddevMulThreshNumber);
    sor->filter(*cloudFiltered);

    return cloudFiltered;
  }

  template <typename PointT>
  typename pcl::PointCloud<PointT>::Ptr getBackground(typename velodyne::PcapCache<PointT> &pcapCache, const uint64_t &offsetFrame, const uint64_t &compareFrameNumber)
  {
    std::vector<std::thread *> ts(std::thread::hardware_concurrency() + 1);
    std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> backCloud(ts.size());
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> back(new pcl::PointCloud<pcl::PointXYZ>());

    boost::function<void(const uint64_t nowTsSize)> function =
        [&pcapCache, &compareFrameNumber, &offsetFrame, &ts, &backCloud](const uint64_t nowTsSize) {
          boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> back(new pcl::PointCloud<pcl::PointXYZ>());
          boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> temp(new pcl::PointCloud<pcl::PointXYZ>());
          boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> temp1(new pcl::PointCloud<pcl::PointXYZ>());
          boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> combineTemp(new pcl::PointCloud<pcl::PointXYZ>());
          boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> combineTemp1(new pcl::PointCloud<pcl::PointXYZ>());

          auto begin = nowTsSize * pcapCache.totalFrame / compareFrameNumber;

          auto d = pcapCache.totalFrame / compareFrameNumber * ts.size();

          temp = pcapCache.get(begin);
          temp1 = pcapCache.get(begin);

          for (int i = begin + d; i < pcapCache.totalFrame; i = i + d)
          {
            temp = myFunction::getNoChanges<pcl::PointXYZ>(temp, pcapCache.get(i), 0.1);
            temp1 = myFunction::getNoChanges<pcl::PointXYZ>(pcapCache.get(i), temp1, 0.1);
          }

          combineTemp = myFunction::getNoChanges<pcl::PointXYZ>(temp, temp1, 0.1);
          combineTemp1 = myFunction::getNoChanges<pcl::PointXYZ>(temp1, temp, 0.1);

          *back = *combineTemp + *combineTemp1;

          backCloud[nowTsSize] = back;
        };

    for (int j = 0; j < ts.size(); j++)
    {
      ts[j] = new std::thread(std::bind(function, j));
    }

    for (auto thread : ts)
    {
      while (!(thread->joinable()))
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      thread->join();
    }

    std::cout << backCloud.size() << std::endl;

    back = backCloud[0];

    for (int i = 1; i < backCloud.size(); i++)
    {
      back = myFunction::getNoChanges<pcl::PointXYZ>(back, backCloud[i], 0.1);
    }

    if (back->points.size() == 0)
      back = backCloud[0];

    std::cout << back->points.size() << std::endl;

    return back;
  }
} // namespace myFunction
#endif
