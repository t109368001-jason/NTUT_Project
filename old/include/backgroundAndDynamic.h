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
using namespace velodyne;

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
  std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> nochangeFrameIndex(typename PcapCache<PointT> &pcapCacheNoBack, const int &objectStopFrameRange, int &objectNoChangesFramePoints, int &objectNoChangesFramePoints2, const double &resolution, const int &meanKNumber, const double &stddevMulThreshNumber)
  {
    std::vector<std::thread *> ts(std::thread::hardware_concurrency() + 1);
    std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> backs(pcapCacheNoBack.totalFrame);

    for(auto &cloud : backs) {
      cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    }

    boost::function<void(int i)> function = 
    [&pcapCacheNoBack, &objectStopFrameRange, &objectNoChangesFramePoints, &objectNoChangesFramePoints2, &resolution, &meanKNumber, &stddevMulThreshNumber, &backs] (int i) {
      if((i > objectStopFrameRange)&&(i < (pcapCacheNoBack.totalFrame-objectStopFrameRange))){
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> tempBeforeFiltered(new pcl::PointCloud<pcl::PointXYZ>());
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> tempAFterFiltered(new pcl::PointCloud<pcl::PointXYZ>());
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> tempNowFiltered(new pcl::PointCloud<pcl::PointXYZ>());

        tempBeforeFiltered = myFunction::getStatisticalOutlierRemoval<pcl::PointXYZ>(pcapCacheNoBack.get(i - objectStopFrameRange), meanKNumber, stddevMulThreshNumber);
        tempAFterFiltered = myFunction::getStatisticalOutlierRemoval<pcl::PointXYZ>(pcapCacheNoBack.get(i + objectStopFrameRange), meanKNumber, stddevMulThreshNumber);
        tempNowFiltered = myFunction::getStatisticalOutlierRemoval<pcl::PointXYZ>(pcapCacheNoBack.get(i), meanKNumber, stddevMulThreshNumber);
        tempBeforeFiltered = myFunction::getNoChanges<pcl::PointXYZ>(tempBeforeFiltered, tempNowFiltered, resolution);
        tempAFterFiltered = myFunction::getNoChanges<pcl::PointXYZ>(tempAFterFiltered, tempNowFiltered, resolution);
        *tempAFterFiltered += *myFunction::getChanges<pcl::PointXYZ>(tempAFterFiltered, tempBeforeFiltered, resolution);
        backs[i] = tempAFterFiltered;
        
/*
        if(tempBeforeFiltered->points.size() < objectNoChangesFramePoints){
            objectNoChangesFramePoints = tempBeforeFiltered->points.size();
            std::cout<< i << "  tempBeforeFiltered= " << objectNoChangesFramePoints << std::endl;
        }

        if(tempAFterFiltered->points.size() < objectNoChangesFramePoints2){
            objectNoChangesFramePoints2 = tempAFterFiltered->points.size();
            std::cout<< i << "  tempAFterFiltered= " << objectNoChangesFramePoints2 << std::endl;
        }

        
        if((tempBeforeFiltered->points.size() <= objectNoChangesFramePoints)||(tempAFterFiltered->points.size() <= objectNoChangesFramePoints2)){
          nochangeFrameNumber[i] = true;
          //std::cout << i << " " << nochangeFrameNumber[i] << std::endl;
        }
*/
      }
    };

    int i = 0;
    while(i < pcapCacheNoBack.totalFrame){
      for(auto &thread : ts){
        if(thread)
        {
          if( thread->joinable() ){
            thread->join();
            thread->~thread();
            delete thread;
            thread = nullptr;

            thread = new std::thread(std::bind(function, i));     
            i++;
          }
        } 
        else{
          std::cout<< "Frane = " << i << std::endl;
          thread = new std::thread(std::bind(function, i));       
          i++;
        }
      }
    }

    for(auto &thread : ts){
      if(thread) {
          while( !thread->joinable() );
          thread->join();
          thread->~thread();
          delete thread;
          thread = nullptr;
      }
    }

    return backs;
  }

  template <typename PointT>
  typename pcl::PointCloud<PointT>::Ptr getBackground(typename PcapCache<PointT> &pcapCache, const int &backNumber, const int &compareFrameNumber, const double resolution)
  {
    std::vector<std::thread *> ts(std::thread::hardware_concurrency() + 1);
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> back(new pcl::PointCloud<pcl::PointXYZ>());
    std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> backs(ts.size());

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
      if(back->points.size() > backNumber) break;
      cloud = myFunction::getChanges<pcl::PointXYZ>(back, cloud, 1.0);
      *back += *cloud;
      std::cout << " back size= " << back->points.size() << std::endl;
    }

    return back;
  }

  template <typename PointT>
  typename pcl::PointCloud<PointT>::Ptr getBackground6(typename PcapCache<PointT> &pcapCache, const int &backNumber, const int &compareFrameNumber, const double resolution)
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
      if(back->points.size() > backNumber)break;
    }

    return back;
  }

} // namespace myFunction
#endif
