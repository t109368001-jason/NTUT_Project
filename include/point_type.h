#ifndef POINT_TYPE_H_
#define POINT_TYPE_H_

#include <iostream>
#include <pcl/io/pcd_io.h>

using namespace std;

namespace myClass
{
	class PointXYZR
	{
	public:
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
		std::vector<int> ring;
		double min_Distance;
		double max_Distance;
		PointXYZR()
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
			this->cloud = temp;
		}
		void set(std::string filename)
		{
			pcl::PCLPointCloud2 cloud_blob;
			pcl::io::loadPCDFile(filename, cloud_blob);
			pcl::fromPCLPointCloud2(cloud_blob, *(this->cloud));

			for(size_t i = 0; i < cloud_blob.fields.size(); i++)
			{
				if(cloud_blob.fields[i].name == "ring")
				{
					for(size_t j = cloud_blob.fields[i].offset; j < cloud_blob.data.size(); j += cloud_blob.point_step)
					{
						ring.push_back(cloud_blob.data[j]);
					}
					break;
				}
			}
		}

	};
}
#endif