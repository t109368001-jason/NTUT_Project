
#include <mutex>
    
#include <pcl/point_types.h>
#include "point_type.h"

namespace myFunction
{

	template<typename RandomIt>
	int likePart2(double threshold, int division_num2, RandomIt point, RandomIt beg2, RandomIt end2)
	{

		auto len2 = end2 - beg2;
		
		if(len2 < division_num2)
		{
			int out = 0;
			for(auto it = beg2; it != end2; ++it)
			{
				if(((*point).getVector3fMap() - (*it).getVector3fMap()).norm() < threshold)
				{
					out += 1;
				}
			}
			return out;
		}

		auto mid2 = beg2 + len2/2;

		auto handle = std::async(std::launch::async, likePart2<RandomIt>, threshold, division_num2, point, beg2, mid2);

		int out = likePart2<RandomIt>(threshold, division_num2, point, mid2, end2);

		int out1(handle.get());
		return out + out1;
	}
	template<typename RandomIt>
	int likePart1(double threshold, int division_num1, int division_num2, RandomIt beg1, RandomIt end1, RandomIt beg2, RandomIt end2)
	{

		auto len1 = end1 - beg1;
		
		if(len1 < division_num1)
		{
			int out = 0;
			for(auto it = beg1; it != end1; ++it)
			{
				out += likePart2(threshold, division_num2, it, beg2, end2);
			}
			return out;
		}

		auto mid1 = beg1 + len1/2;

		auto handle = std::async(std::launch::async, likePart1<RandomIt>, threshold, division_num1, division_num2, beg1, mid1, beg2, end2);

		int out = likePart1<RandomIt>(threshold, division_num1, division_num2, mid1, end1, beg2, end2);

		int out1(handle.get());
		return out + out1;
	}

	double like(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, double threshold_m)
	{
		double threshold = threshold_m;

		int division_num1 = std::ceil(cloud1->points.size() / std::thread::hardware_concurrency()) + std::thread::hardware_concurrency();
		int division_num2 = std::ceil(cloud2->points.size() / std::thread::hardware_concurrency()) + std::thread::hardware_concurrency();

		return likePart1(threshold, division_num1, division_num2, cloud1->points.begin(), cloud1->points.end(), cloud2->points.begin(), cloud2->points.end()) / (double)(cloud1->points.size());
	}

	//std::thread::hardware_concurrency();
    template<class T=pcl::PointCloud<pcl::PointXYZ>::Ptr>
    pcl::PointCloud<pcl::PointXYZ>::Ptr getMaxPart (pcl::PointCloud<pcl::PointXYZ>::Ptr input)
    {
        pcl::console::TicToc tt;
        std::cerr << "Segmentation...", tt.tic();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        *cloud = *input;

		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;

		double min_distance = getMinDistanceBetweenPoints(cloud);
		
		std::vector<int> result(1);
		std::vector<float> result_sqr_distance(1);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

		min_distance *= 1.1;
		int i=0;
		while(cloud->points.size() > 0)
		{
			if(i >= cloud->points.size())
			{
				i = 0;
				min_distance *= 1.1;
				continue;
			}
			pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
			result.resize(cloud->points.size());
			result_sqr_distance.resize(result.size());
			tree->setInputCloud(cloud);
			tree->nearestKSearch(cloud->points[i], result.size(), result, result_sqr_distance);
			for(int j = 0; j < result_sqr_distance.size(); j++)
			{
				if(result_sqr_distance[j] > min_distance*min_distance)
				{
					result.erase(result.begin() + j);
					result_sqr_distance.erase(result_sqr_distance.begin() + j);
					j -= 1;
				}
			}
			if(result.size() == 0)
			{
				i++;
				continue;
			}
			std::sort(result.begin(), result.end(), [](const int a, const int b) {return a > b; });
			while(result.size() > 0)
			{
				temp->points.push_back(cloud->points[result[0]]);
				cloud->points.erase(cloud->points.begin() + result[0]);
				result.erase(result.begin() + 0);
			}
			clouds.push_back(temp);
		}
		std::sort(clouds.begin(), clouds.end(), [](const pcl::PointCloud<pcl::PointXYZ>::Ptr a, const pcl::PointCloud<pcl::PointXYZ>::Ptr b) {return a->points.size() > b->points.size(); });

        std::cerr << " >> Done: " << tt.toc() << " ms\n\n";
		if(cloud->size() == 2)
		{
			if(clouds[0]->points.size() < clouds[1]->points.size())
			{
				return clouds[1];
			}
		}
        return clouds[0];
    }
	
	PointXYZR pcd_to_XYZR(std::string filename)
	{
		PointXYZR test;
		test.set(filename);

		return test;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr XYZR_to_XYZRGB(PointXYZR xyzr, int fix_Num = 0, int points_of_fix = 9)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr xyz = xyzr.cloud;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
		xyzr.max_Distance = 0.0;
		xyzr.min_Distance = xyz->points[0].getVector3fMap().norm();

		for(size_t i = 0; i < xyz->points.size(); i++)
		{
			xyzr.max_Distance = fmax((double)(xyz->points[i].getVector3fMap().norm()), xyzr.max_Distance);
			xyzr.min_Distance = fmin((double)(xyz->points[i].getVector3fMap().norm()), xyzr.min_Distance);
		}

		for(size_t i = 0; i < xyz->points.size(); i++)
		{
			pcl::PointXYZRGB point;
			point.x = xyz->points[i].x;
			point.y = xyz->points[i].y;
			point.z = xyz->points[i].z;
			uint8_t r;
			r = ((point.getVector3fMap().norm() - xyzr.min_Distance) * 255.0 / (xyzr.max_Distance - xyzr.min_Distance));
			r = 255 - r;
			uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
				static_cast<uint32_t>(r) << 8 | static_cast<uint32_t>(r));
			point.rgb = *reinterpret_cast<float*>(&rgb);
			xyzrgb->points.push_back (point);
		}
		if((fix_Num == 0)||(points_of_fix == 0))
		{
			xyzrgb->width = (int) xyzrgb->points.size();
			xyzrgb->height = 1;

			return xyzrgb;
		}

		points_of_fix += 1;
		std::vector<pcl::PointXYZRGB> y;
		
		for(size_t h = 0; h < xyzrgb->points.size(); h++)
		{
			std::vector<pcl::PointXYZRGB> x;
			Eigen::Vector3f currentPoint = xyzrgb->points[h].getVector3fMap();
			for(size_t i = 0; i < xyzrgb->points.size(); i++)
			{
				if(xyzr.ring.size() > 0)
				{
					if(xyzr.ring[h] == xyzr.ring[i]) continue;
				}
				//if((xyzrgb->points[h].z/xyzrgb->points[h].getVector3fMap().norm()) == (xyzrgb->points[i].z/xyzrgb->points[i].getVector3fMap().norm()))continue;
				//if(fabs((xyzrgb->points[i].getVector3fMap() - currentPoint).norm()) < averageDistance)continue;
				if(i == h) continue;
				pcl::PointXYZRGB point = xyzrgb->points[i];
				if(x.size() == 0)
				{
					x.push_back(point);
					continue;
				}
				for(size_t j = 0; j < x.size(); j++)
				{
					if(fabs((point.getVector3fMap() - currentPoint).norm()) < fabs((x[j].getVector3fMap() - currentPoint).norm()))
					{
						x.push_back(point);
						if(x.size() > fix_Num)
						{
							x.erase(x.begin() + j);
						}
						break;
					}
				}
			}
			if(h%(xyzrgb->points.size() / 100) == 0)cout << "Calculting " << double(h) / double(xyzrgb->points.size()) *100.0 << "\%" << endl;
			for(size_t j = 0; j < (x.size()-1); j++)
			{
				pcl::PointXYZRGB point;
				Eigen::Vector3f temp = (x[j].getVector3fMap() - xyzrgb->points[h].getVector3fMap()) / points_of_fix;
				for(size_t k=1; k < points_of_fix; k++)
				{
					point.getVector3fMap() = xyzrgb->points[h].getVector3fMap() + temp * k;
			uint8_t r;
			r = ((point.getVector3fMap().norm() - xyzr.min_Distance) * 255.0 / (xyzr.max_Distance - xyzr.min_Distance));
			r = 255 - r;
					uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
						static_cast<uint32_t>(r) << 8 | static_cast<uint32_t>(r));
					point.rgb = *reinterpret_cast<float*>(&rgb);
					y.push_back (point);
				}
			}
		}

		for(size_t i = 0; i < y.size(); i++)
		{
			xyzrgb->points.push_back (y[i]);
		}

		xyzrgb->width = (int) xyzrgb->points.size();
		xyzrgb->height = 1;

		return xyzrgb;
	}
}