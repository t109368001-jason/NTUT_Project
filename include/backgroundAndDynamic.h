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
  typename pcl::PointCloud<PointT>::Ptr getStatisticalOutlierRemoval(const typename pcl::PointCloud<PointT>::Ptr &cloud, const int &meanKNumber, const double &stddevMulThreshNumber)
  {
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    boost::shared_ptr<pcl::StatisticalOutlierRemoval<pcl::PointXYZ>> sor(new pcl::StatisticalOutlierRemoval<pcl::PointXYZ>());

    sor->setInputCloud(cloud);
    sor->setMeanK(meanKNumber);
    sor->setStddevMulThresh(stddevMulThreshNumber);
    sor->filter(*cloudFiltered);

    return cloudFiltered;
  }

  template <typename PointT>
  typename pcl::PointCloud<PointT>::Ptr getBackground(typename velodyne::PcapCache<PointT> &pcapCache, const uint64_t &compareFrameNumber, const double resolution)
  {
    std::vector<std::thread *> ts(std::thread::hardware_concurrency() + 1);
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> back(new pcl::PointCloud<pcl::PointXYZ>());
    std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> backs(ts.size());
    std::mutex g_pages_mutex;


    int d = pcapCache.totalFrame/compareFrameNumber/2*2;
    int jump = compareFrameNumber/2;

    boost::function<void(int i, int j)> function = 
    [&backs, &jump, &d, &pcapCache, &resolution] ( int i, int j) {
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> temp(new pcl::PointCloud<pcl::PointXYZ>());
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> temp1(new pcl::PointCloud<pcl::PointXYZ>());
      temp = myFunction::getNoChanges<pcl::PointXYZ>(pcapCache.get(i), pcapCache.get(i+(jump*d)), resolution);
      temp1 = myFunction::getNoChanges<pcl::PointXYZ>(pcapCache.get(i+(jump*d)), pcapCache.get(i), resolution);
      temp = myFunction::getChanges<pcl::PointXYZ>(backs[j], temp, 1.0);
      temp1 = myFunction::getChanges<pcl::PointXYZ>(backs[j], temp1, 1.0);
      *backs[j] += *temp;
      *backs[j] += *temp1;
    };
    
    for(auto &cloud : backs) {
      cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    }
    int i = 0;
    while(i < int(pcapCache.totalFrame/2)){
      bool skip = false;
      for(int j = 0; j<ts.size(); j++){
        if(i >= int(pcapCache.totalFrame/2)) { skip = true; break; }
        if(ts[j])
        {
          if( ts[j]->joinable() ){
              ts[j]->join();
              ts[j]->~thread();
              delete ts[j];
              ts[j] = nullptr;

              ts[j] = new std::thread(std::bind(function, i, j));     
              i += d;
          }
        } 
        else{
          ts[j] = new std::thread(std::bind(function, i, j));       
          i += d;
        }
      }
      if(skip) break;
    }

    for(int j = 0; j < ts.size(); j++) {
        if(ts[j]) {
            while( !ts[j]->joinable() );
            ts[j]->join();
            ts[j]->~thread();
            delete ts[j];
            ts[j] = nullptr;
        }
    }
    for(auto cloud : backs) {
      if(back->points.size() > 1500000) { break; }

      cloud = myFunction::getChanges<pcl::PointXYZ>(back, cloud, 1.0);
      *back += *cloud;
      std::cout << " back size= " << back->points.size() << std::endl;
    }

    return back;
  }

  template <typename PointT>
  typename pcl::PointCloud<PointT>::Ptr getBackground6(typename velodyne::PcapCache<PointT> &pcapCache, const uint64_t &compareFrameNumber, const double resolution)
  {
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> back(new pcl::PointCloud<pcl::PointXYZ>());
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> temp(new pcl::PointCloud<pcl::PointXYZ>());
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> temp1(new pcl::PointCloud<pcl::PointXYZ>());
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> combineTemp(new pcl::PointCloud<pcl::PointXYZ>());

    int d = pcapCache.totalFrame/compareFrameNumber/2*2;
    int jump = compareFrameNumber/2;

    int i = 0;
    while(i < int(pcapCache.totalFrame/2)){
      temp = myFunction::getNoChanges<pcl::PointXYZ>(pcapCache.get(i), pcapCache.get(i+(jump*d)), resolution);
      temp1 = myFunction::getNoChanges<pcl::PointXYZ>(pcapCache.get(i+(jump*d)), pcapCache.get(i), resolution);
      temp1 = myFunction::getChanges<pcl::PointXYZ>(temp, temp1, 1.0);
      *combineTemp = *temp + *temp1;
      combineTemp = myFunction::getChanges<pcl::PointXYZ>(back, combineTemp, 1.0);
      *back += *combineTemp;
      std::cout << i << "  " << back->points.size() << std::endl;
      i += d;
      if(back->points.size() > 1500000)break;
    }

    return back;
  }

  template <typename PointT>
  typename pcl::PointCloud<PointT>::Ptr getBackground5(typename velodyne::PcapCache<PointT> &pcapCache, const uint64_t &compareFrameNumber, const double resolution)
  {
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> back(new pcl::PointCloud<pcl::PointXYZ>());
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> temp(new pcl::PointCloud<pcl::PointXYZ>());
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> temp1(new pcl::PointCloud<pcl::PointXYZ>());

    int d = pcapCache.totalFrame/compareFrameNumber/2*2;
    int jump = compareFrameNumber/2;

    back = pcapCache.get(0);

    for(int i = 0; i < (pcapCache.totalFrame/2); i = i + d){
      temp = myFunction::getNoChanges<pcl::PointXYZ>(back, pcapCache.get(i+(jump*d)), resolution);
      temp1 = myFunction::getNoChanges<pcl::PointXYZ>(pcapCache.get(i+(jump*d)), back, resolution);
      *back = *temp + *temp1;
      std::cout << i << "  " << back->points.size() << std::endl;
    }
    return back;
  }

  template <typename PointT>
  typename pcl::PointCloud<PointT>::Ptr getBackground4(typename velodyne::PcapCache<PointT> &pcapCache, const uint64_t &compareFrameNumber, const double resolution)
  {

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> back(new pcl::PointCloud<pcl::PointXYZ>());
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> temp(new pcl::PointCloud<pcl::PointXYZ>());
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> temp1(new pcl::PointCloud<pcl::PointXYZ>());
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> combineTemp(new pcl::PointCloud<pcl::PointXYZ>());
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> combineTemp1(new pcl::PointCloud<pcl::PointXYZ>());


    temp = pcapCache.get(0);
    temp1 = pcapCache.get(0);
    //begin = (begin == cfn - 1 ? cfn : (begin < 5 ? begin + cfn/2 : (begin + cfn/2 + 1)%cfn));
    for (int i = compareFrameNumber/2; i < compareFrameNumber; i = (i == compareFrameNumber - 1 ? compareFrameNumber : (i < compareFrameNumber/2 ? i + compareFrameNumber/2 : (i + compareFrameNumber/2 + 1)%compareFrameNumber))){
      temp = myFunction::getNoChanges<pcl::PointXYZ>(temp, pcapCache.get(i*pcapCache.totalFrame/compareFrameNumber), resolution);
      temp1 = myFunction::getNoChanges<pcl::PointXYZ>(pcapCache.get(i*pcapCache.totalFrame/compareFrameNumber), temp1, resolution);
    }

    combineTemp = myFunction::getNoChanges<pcl::PointXYZ>(temp, temp1, resolution);
    combineTemp1 = myFunction::getNoChanges<pcl::PointXYZ>(temp1, temp, resolution);

    *back = *combineTemp + *combineTemp1;

    return back;

  }

  template <typename PointT>
  typename pcl::PointCloud<PointT>::Ptr getBackground3(typename velodyne::PcapCache<PointT> &pcapCache, const uint64_t &compareFrameNumber, const double resolution)
  {
    std::vector<std::thread *> ts(std::thread::hardware_concurrency() + 1);
    std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> backCloud(ts.size());
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> back(new pcl::PointCloud<pcl::PointXYZ>());

    boost::function<void(const uint64_t nowTsSize)> function =
        [&pcapCache, &compareFrameNumber, &ts, &backCloud, &resolution](const uint64_t nowTsSize) {
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
            temp = myFunction::getNoChanges<pcl::PointXYZ>(temp, pcapCache.get(i), resolution);
            temp1 = myFunction::getNoChanges<pcl::PointXYZ>(pcapCache.get(i), temp1, resolution);
          }

          combineTemp = myFunction::getNoChanges<pcl::PointXYZ>(temp, temp1, resolution);
          combineTemp1 = myFunction::getNoChanges<pcl::PointXYZ>(temp1, temp, resolution);

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
      back = myFunction::getNoChanges<pcl::PointXYZ>(back, backCloud[i], resolution);
    }

    if (back->points.size() == 0)
      back = backCloud[0];

    std::cout << back->points.size() << std::endl;

    return back;
  }

  template<typename PointT>
	typename pcl::PointCloud<PointT>::Ptr getBackground2(typename velodyne::PcapCache<PointT> &pcapCache, const uint64_t &offsetFrame, const uint64_t &compareFrameNumber, const uint64_t &times, const double resolution)
	{
		std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> backCloud;
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> back(new pcl::PointCloud<pcl::PointXYZ>());
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> temp(new pcl::PointCloud<pcl::PointXYZ>());
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> temp1(new pcl::PointCloud<pcl::PointXYZ>());
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> combineTemp(new pcl::PointCloud<pcl::PointXYZ>());
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> combineTemp1(new pcl::PointCloud<pcl::PointXYZ>());

		for(int j = 0; j < times; j++){
            temp = pcapCache.get(offsetFrame + j*pcapCache.totalFrame/compareFrameNumber/times);
            temp1 = pcapCache.get(offsetFrame + j*pcapCache.totalFrame/compareFrameNumber/times);

            for(int i = (j + times); i < (compareFrameNumber*times); i = i + times){
                temp = myFunction::getNoChanges<pcl::PointXYZ>(temp, pcapCache.get(offsetFrame + i*pcapCache.totalFrame/compareFrameNumber/times), resolution);
                temp1 = myFunction::getNoChanges<pcl::PointXYZ>(pcapCache.get(offsetFrame + i*pcapCache.totalFrame/compareFrameNumber/times), temp1, resolution);
            }

            combineTemp = myFunction::getNoChanges<pcl::PointXYZ>(temp, temp1, resolution);
            combineTemp1 = myFunction::getNoChanges<pcl::PointXYZ>(temp1, temp, resolution);

            *back = *combineTemp + *combineTemp1;
            backCloud.push_back(back);
        }

        back = backCloud[0];

        for(int i = 1; i < backCloud.size(); i++){
            back = myFunction::getNoChanges<pcl::PointXYZ>(back, backCloud[i], resolution);
        }

        if(back->points.size() == 0) back = backCloud[0];

        std::cout<< back->points.size() << std::endl;

		return back;
	}

} // namespace myFunction
#endif
