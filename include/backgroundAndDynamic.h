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
    template<typename PointT>
	typename pcl::PointCloud<PointT>::Ptr getBackground(typename velodyne::PcapCache<PointT> &pcapCache, const uint64_t &offsetFrame, const uint64_t &compareFrameNumber, const uint64_t &times)
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
                temp = myFunction::getNoChanges<pcl::PointXYZ>(temp, pcapCache.get(offsetFrame + i*pcapCache.totalFrame/compareFrameNumber/times), 0.1);
                temp1 = myFunction::getNoChanges<pcl::PointXYZ>(pcapCache.get(offsetFrame + i*pcapCache.totalFrame/compareFrameNumber/times), temp1, 0.1);
            }

            combineTemp = myFunction::getNoChanges<pcl::PointXYZ>(temp, temp1, 0.1);
            combineTemp1 = myFunction::getNoChanges<pcl::PointXYZ>(temp1, temp, 0.1);

            *back = *combineTemp + *combineTemp1;
            backCloud.push_back(back);
        }

        back = backCloud[0];

        for(int i = 1; i < backCloud.size(); i++){
            back = myFunction::getNoChanges<pcl::PointXYZ>(back, backCloud[i], 0.1);
        }
        
        if(back->points.size() == 0) back = backCloud[0];

        std::cout<< back->points.size() << std::endl;

		return back;
	}

    template<typename PointT>
	typename pcl::PointCloud<PointT>::Ptr getDynamicStatisticalOutlierRemoval(const typename pcl::PointCloud<PointT>::Ptr &cloud1, const typename pcl::PointCloud<PointT>::Ptr &cloud2, const int &meanKNumber, const double &stddevMulThreshNumber)
    {
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
        boost::shared_ptr<pcl::StatisticalOutlierRemoval<pcl::PointXYZ>> sor(new pcl::StatisticalOutlierRemoval<pcl::PointXYZ>());
        
        sor->setInputCloud (myFunction::getChanges<pcl::PointXYZ>(cloud1, cloud2, 50.0));
        sor->setMeanK (meanKNumber);
        sor->setStddevMulThresh (stddevMulThreshNumber);
        sor->filter (*cloudFiltered);

        return cloudFiltered;
    }
}
#endif
