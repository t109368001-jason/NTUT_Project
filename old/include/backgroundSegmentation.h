#ifndef BACKGROUND_SEGMENTATION_H_
#define BACKGROUND_SEGMENTATION_H_

#include <iostream>
#include <pcl-1.8/pcl/octree/octree_pointcloud_changedetector.h>

namespace myClass
{
	template<typename PointT>
	class backgroundSegmentation
	{
		private:
			bool isSet;
			bool log_to_file;
			typename pcl::PointCloud<PointT>::Ptr background;
			pcl::octree::OctreePointCloudChangeDetector<PointT> *octree;
		public:
			void setBackground(const typename pcl::PointCloud<PointT>::Ptr &background, const double &resolution = 1, const bool log_to_file = false)
			{
				this->background = background;
				
				this->octree = new pcl::octree::OctreePointCloudChangeDetector<PointT>(resolution);
				this->octree->setInputCloud(this->background);
				this->octree->addPointsFromInputCloud();
				this->octree->switchBuffers();

				this->log_to_file = log_to_file;
				
				if(this->log_to_file)
				{
					std::ofstream ofs("backgroundSegmentation.csv", std::ios::ate);
					ofs << "name" << "," << "total" << "," << "after segmentation" << std::endl;
				}

				this->isSet = true;
			}

			typename pcl::PointCloud<PointT>::Ptr compute(const typename pcl::PointCloud<PointT>::Ptr &cloud, const std::string name = "") const
			{
				if(!isSet)
				{
					std::cerr << "Background is not set" << std::endl;
					return cloud;
				}

				typename pcl::PointCloud<PointT>::Ptr temp(new typename pcl::PointCloud<PointT>);
				pcl::octree::OctreePointCloudChangeDetector<PointT> tree = *this->octree;
				std::vector<int> newPointIdxVector;
				
				tree.setInputCloud(cloud);
				tree.addPointsFromInputCloud();

				tree.getPointIndicesFromNewVoxels(newPointIdxVector);	//get changed points
				
				for(auto it = newPointIdxVector.begin(); it != newPointIdxVector.end(); ++it)
				{
					temp->points.push_back(cloud->points[*it]);
				}

				temp->width = static_cast<uint32_t>(temp->points.size());
				temp->height = 1;

				if(this->log_to_file)
				{
					std::ofstream ofs("backgroundSegmentation.csv", std::ios::app);
					ofs << name << "," << cloud->points.size() << "," << temp->points.size() << std::endl;
					ofs.close();
				}

				return temp;
			}
			
	};
}
#endif