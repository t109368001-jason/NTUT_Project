#ifndef FUNCTION_H_
#define FUNCTION_H_

#include <iostream>
#include <future>
//#include <librealsense2/rs.hpp>
//#include <librealsense2/rsutil.h>
#include <boost/algorithm/clamp.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <include/basic_function.h>

using namespace std;

namespace myFunction
{

#pragma region basic

	double distance(const double &ax, const double &ay, const double &az, const double bx = 0, const double by = 0, const double bz = 0)
	{
		return std::sqrt((ax-bx)*(ax-bx)+(ay-by)*(ay-by)+(az-bz)*(az-bz));
	}

	template<typename PointT>
	double distance(PointT p1, PointT p2 = PointT(0,0,0))
	{
		return distance(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);
	}

	template<typename PointT>
	double norm(PointT p1)
	{
		return std::sqrt(p1.x*p1.x+p1.y*p1.y+p1.z*p1.z);
	}

	double norm(double x, double y, double z)
	{
		return std::sqrt(x*x+y*y+z*z);
	}

	double getPhi(const double &x, const double &y)
	{
		double phi = atan(y / x);
		
		if (x < 0.0)
		{
			if(y < 0.0)
			{
				phi -= M_PI;
			}
			else
			{
				phi += M_PI;
			}
		}
		return phi;
	}
	
	double getTheta(const double &x, const double &y, const double &z)
	{
		double radius = norm(x,y,z);
		double theta = acos(z / radius);
		
		return theta;
	}

	void XYZ_to_Sphere(const double &x, const double &y, const double &z, double &radius, double &phi, double &theta)
	{
		radius = norm(x,y,z);
		theta = acos(z / radius);
		phi = atan(y / x);
		
		if (x < 0.0)
		{
			if(y < 0.0)
			{
				phi -= M_PI;
			}
			else
			{
				phi += M_PI;
			}
		}
	}

	double rotateX(double &x, double &y, double &z, const double &angle)	//rotate point by x axis
	{
		double x_ = x;
		double y_ = y;
		double z_ = z;

		x = x_;
		y = y_*std::cos(angle) + z_*std::sin(angle);
		z = y_*(-std::sin(angle)) + z_*std::cos(angle);
	}

	template<typename PointT>
	double rotateX(PointT &point, const double angle)
	{
		rotateX(point.x, point.y, point.z, angle);
	}

	double rotateY(double &x, double &y, double &z, const double &angle)	//rotate point by y axis
	{
		double x_ = x;
		double y_ = y;
		double z_ = z;

		x = x_*std::cos(angle) + z_*(-std::sin(angle));
		y = y_;
		z = x_*std::sin(angle) + z_*std::cos(angle);
	}

	template<typename PointT>
	double rotateY(PointT &point, const double &angle)
	{
		rotateY(point.x, point.y, point.z, angle);
	}

	double rotateZ(double &x, double &y, double &z, const double &angle)	//rotate point by z axis
	{
		double x_ = x;
		double y_ = y;
		double z_ = z;

		z = z_;
		x = x_*std::cos(angle) + y_*std::sin(angle);
		y = x_*(-std::sin(angle)) + y_*std::cos(angle);
	}

	template<typename PointT>
	double rotateZ(PointT &point, const double &angle)
	{
		rotateZ(point.x, point.y, point.z, angle);
	}

	pcl::PolygonMesh stl_to_mesh(std::string filename)
	{
		pcl::PolygonMesh mesh;
		pcl::io::loadPolygonFileSTL(filename, mesh);

		return mesh;
	}

#pragma endregion basic
	
#pragma region value_to_RGB

	void value_to_RGB(const double &value, const double &min, const double &max, uint8_t &r, uint8_t &g, uint8_t &b)
	{
		double temp = (value - min)/(max-min);
		if(temp > 1.0) temp = 1.0;
		else if(temp < 0.0) temp = 0.0;

		temp *= 1023;

		if(temp < 128.0)
		{
			r = 0;
			g = 0;
			b = 128 + temp;
		}
		else if(temp < (384))
		{
			r = 0;
			g = temp - 128;
			b = 255;
		}
		else if(temp < (640))
		{
			r = (temp - 384);
			g = 255;
			b = 255 - (temp - 384);
		}
		else if(temp < (896))
		{
			r = 255;
			g = 255 - (temp - 640);
			b = 0;
		}
		else if(temp < (1024))
		{
			r = 255 - (temp - 896);
			g = 0;
			b = 0;
		}
		else
		{
			r = 127;
			g = 0;
			b = 0;
		}
	}

#pragma endregion value_to_RGB

#pragma region getNearestPointsDistance

	//取得點雲中最近的兩個點距離
	template<typename RandomIt, typename PointT>
	double getNearestPointsDistancePart(const int &division_num, typename pcl::search::KdTree<PointT>::Ptr tree, const RandomIt &beg, const RandomIt &end)
	{
		auto len = end - beg;

		if(len < division_num)
		{
			double sqr_out = std::numeric_limits<double>::max();
			for(auto it = beg; it != end; ++it)
			{
				std::vector<int> indices (2);
				std::vector<float> sqr_distances (2);

				tree->nearestKSearch(*it, 2, indices, sqr_distances);

				if ((sqr_distances[1] < sqr_out)&&(sqr_distances[1] != 0.0)) sqr_out = sqr_distances[1];
			}
			return std::sqrt(sqr_out);
		}
		auto mid = beg + len/2;
		auto handle = std::async(std::launch::async, getNearestPointsDistancePart<RandomIt, PointT>, division_num, tree, beg, mid);
		auto out = getNearestPointsDistancePart<RandomIt, PointT>(division_num, tree, mid, end);
		auto out1 = handle.get();

		if(out1 < out) out = out1;

		return out;
	}

	//取得點雲中最近的兩個點距離取得點雲中最近的兩個點距離
	template<typename PointT>
	double getNearestPointsDistance(const typename pcl::PointCloud<PointT>::Ptr &cloud)
	{
        int division_num = getDivNum<size_t, size_t>(cloud->points.size());
		typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
 		
		tree->setInputCloud(cloud);
		
		return getNearestPointsDistancePart<decltype(cloud->points.begin()), PointT>(division_num, tree, cloud->points.begin(), cloud->points.end());
	}

#pragma endregion getNearestPointsDistance

#pragma region getFarthestPointsDistance
	//取得點雲中最遠的兩個點距離
	template<typename RandomIt>
	double getFarthestPointsDistancePart(const int &division_num, const RandomIt &beg1, const RandomIt &end1, const RandomIt &beg2, const RandomIt &end2)
	{
		auto len1 = end1 - beg1;

		if(len1 < division_num)
		{
			double sqr_out = -std::numeric_limits<double>::max();
			for(auto it1 = beg1; it1 != end1; ++it1)
			{
				for(auto it2 = beg2; it2 != end2; ++it2)
				{
					if(it1 == it2) continue;
					double dist = distance(*it1, *it2);
					if(dist > sqr_out) sqr_out = dist;
				}
			}
			return std::sqrt(sqr_out);
		}
		auto mid1 = beg1 + len1/2;
		auto handle = std::async(std::launch::async, getFarthestPointsDistancePart<RandomIt>, division_num, beg1, mid1, beg2, end2);
		auto out = getFarthestPointsDistancePart<RandomIt>(division_num, mid1, end1, beg2, end2);
		auto out1 = handle.get();

		if(out1 < out) out = out1;

		return out;
	}
	
	//取得點雲中最遠的兩個點距離
	template<typename PointT>
	double getFarthestPointsDistance(const typename pcl::PointCloud<PointT>::Ptr &cloud)
	{
        int division_num = getDivNum<size_t, size_t>(cloud->points.size());
		
		return getFarthestPointsDistancePart<decltype(cloud->points.begin())>(division_num, cloud->points.begin(), cloud->points.end(), cloud->points.begin(), cloud->points.end());
	}

#pragma endregion getFarthestPointsDistance

#pragma region getNearOrFarthestPoint

	//取得點雲中距離point最遠的點
	template<typename RandomIt, typename PointT>
	PointT getNearOrFarthestPointPart(const int &division_num, const bool &Nearest, const PointT &point, const RandomIt &beg, const RandomIt &end)
	{
		auto len = end - beg;

		if(len < division_num)
		{
			double current;
			PointT out;
			if(Nearest)
			{
				for(auto it = beg; it != end; ++it)
				{
					double temp = distance(point.x, point.y, point.z, (*it).x, (*it).y, (*it).z);
					if(temp < current)
					{
						current = temp;
						out = *it;
					}
				}
			}
			else
			{
				for(auto it = beg; it != end; ++it)
				{
					double temp = distance(point.x, point.y, point.z, (*it).x, (*it).y, (*it).z);
					if(temp > current)
					{
						current = temp;
						out = *it;
					}
				}
			}
			return out;
		}
		auto mid = beg + len/2;
		auto handle = std::async(std::launch::async, getNearOrFarthestPointPart<RandomIt, PointT>, division_num, Nearest, point, beg, mid);
		auto out = getNearOrFarthestPointPart<RandomIt, PointT>(division_num, Nearest, point, mid, end);
		auto out1 = handle.get();

		if((distance(point.x, point.y, point.z, out1.x, out1.y, out1.z) > distance(point.x, point.y, point.z, out.x, out.y, out.z)) ^ Nearest) return out1;

		return out;
	}

	template<typename PointT>
	PointT getNearOrFarthestPoint(const typename pcl::PointCloud<PointT>::Ptr &cloud, const bool Nearest = true, const PointT point = PointT(0,0,0))
	{
        int division_num = getDivNum<size_t, size_t>(cloud->points.size());
		
		return getNearOrFarthestPointPart<decltype(cloud->points.begin()), PointT>(division_num, Nearest, point, cloud->points.begin(), cloud->points.end());
	}

#pragma endregion getNearOrFarthestPoint

#pragma region pcd_to_poissonMesh

	void pcd_to_poissonMesh(const std::string &filename, pcl::PolygonMesh &poission)
	{
		string ply_filename = filename.substr(0,filename.find_last_of('.'))+"_poission.ply";

		if (pcl::io::loadPLYFile(ply_filename, poission) == -1)
		{
			pcl::PCDReader reader;
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

			reader.read (filename, *cloud);

			pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
			pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
			tree->setInputCloud(cloud);
			n.setInputCloud(cloud);
			n.setSearchMethod(tree);
			n.setKSearch(30);
			n.compute(*normals);

			pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
			pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

			pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
			tree2->setInputCloud(cloud_with_normals);

			pcl::Poisson<pcl::PointNormal> poisson;
			poisson.setDepth(8);
			poisson.setSolverDivide(8);
			poisson.setIsoDivide(8);
			poisson.setPointWeight(4.0f);
			poisson.setInputCloud(cloud_with_normals);

			poisson.reconstruct(poission);
			pcl::io::savePLYFile(ply_filename, poission);
		}
	}

	pcl::PolygonMesh pcd_to_poissonMesh(const std::string &filename)
	{
		string ply_filename = filename.substr(0,filename.find_last_of('.'))+"_poission.ply";
		pcl::PolygonMesh poission;
		if (pcl::io::loadPLYFile(ply_filename, poission) == -1)
		{
			pcl::PCDReader reader;
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

			reader.read (filename, *cloud);

			pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
			pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
			tree->setInputCloud(cloud);
			n.setInputCloud(cloud);
			n.setSearchMethod(tree);
			n.setKSearch(30);
			n.compute(*normals);

			pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
			pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

			pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
			tree2->setInputCloud(cloud_with_normals);

			pcl::Poisson<pcl::PointNormal> poisson;
			poisson.setDepth(8);
			poisson.setSolverDivide(8);
			poisson.setIsoDivide(8);
			poisson.setPointWeight(4.0f);
			poisson.setInputCloud(cloud_with_normals);

			poisson.reconstruct(poission);
			pcl::io::savePLYFile(ply_filename, poission);
		}
		return poission;
	}
	
#pragma endregion pcd_to_poissonMesh

#pragma region loadMultiPCD

	template<typename RandomIt1, typename RandomIt2, typename PointT>
	int loadMultiPCDPart(const int &division_num, const RandomIt1 &beg1, const RandomIt1 &end1, const RandomIt2 &beg2, const RandomIt2 &end2)
	{
		auto len1 = end1 - beg1;
		auto len2 = end2 - beg2;

		if(len1 < division_num)
		{
			int out = 0;
			auto it2 = beg2;
			for(auto it1 = beg1; it1 != end1; ++it1,++it2)
			{
				typename pcl::PointCloud<PointT>::Ptr temp(new typename pcl::PointCloud<PointT>);
				*it2 = temp;
				pcl::io::loadPCDFile(*it1, **it2);
				out++;
			}
			return out;
		}

		auto mid1 = beg1 + len1/2;
		auto mid2 = beg2 + len1/2;
		auto handle = std::async(std::launch::async, loadMultiPCDPart<RandomIt1, RandomIt2, PointT>, division_num, beg1, mid1, beg2, mid2);
		auto out1 = loadMultiPCDPart<RandomIt1, RandomIt2, PointT>(division_num, mid1, end1, mid2, end2);
		auto out = handle.get();

		return out + out1;
	}

	template<typename PointT>
	int loadMultiPCD(const std::string &filename, std::vector<boost::shared_ptr<typename pcl::PointCloud<PointT>>> &clouds)
	{
		std::ifstream fs;
		std::string line;
		std::vector<std::string> lines;

		fs.open(filename);

		while(!fs.eof())
		{
			std::getline(fs, line);

			boost::trim(line);
			if(line == "") continue;
			lines.push_back(line);
		}

		clouds.resize(lines.size());

        int division_num = getDivNum<size_t, size_t>(lines.size());
		int num = loadMultiPCDPart<decltype(lines.begin()), decltype(clouds.begin()), PointT>(division_num, lines.begin(), lines.end(), clouds.begin(), clouds.end());
		
		return num;
	}

#pragma endregion loadMultiPCD

#pragma region XYZ_to_XYZRGB

	template<typename RandomIt>
	std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> XYZ_to_XYZRGBPart(const int &division_num, const double &min_Distance, const double &div, const bool &gray, const RandomIt beg, const RandomIt &end, const double clip_lower = std::numeric_limits<double>::max(), const double clip_upper = std::numeric_limits<double>::min())
	{
		auto len = end - beg;

		if(len < division_num)
		{
			std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> out;
			uint32_t rgb;
			pcl::PointXYZRGB point;
			for(auto it = beg; it != end; ++it)
			{
				point.x = (*it).x;
				point.y = (*it).y;
				point.z = (*it).z;
				if(gray)
				{
					uint8_t r;
					double radius = std::sqrt(point.x*point.x+point.y*point.y+point.z*point.z);
					boost::algorithm::clamp(radius, clip_lower, clip_upper);
					r = ((radius - min_Distance) * 255.0 / div);
					r = 255 - r;
					rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(r) << 8 | static_cast<uint32_t>(r));
				}
				else
				{
					uint8_t r;
					uint8_t g;
					uint8_t b;
					value_to_RGB(std::sqrt(point.x*point.x+point.y*point.y+point.z*point.z), min_Distance, div + min_Distance, r, g, b);
					rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
				}
				
				point.rgb = *reinterpret_cast<float*>(&rgb);
				out.push_back (point);
			}
			return out;
		}
		auto mid = beg + len/2;
		auto handle = std::async(std::launch::async, XYZ_to_XYZRGBPart<RandomIt>, division_num, min_Distance, div, gray, beg, mid, clip_lower, clip_upper);
		auto out = XYZ_to_XYZRGBPart(division_num, min_Distance, div, gray, mid, end, clip_lower, clip_upper);
		auto out1 = handle.get();

		std::copy(out1.begin(), out1.end(), std::back_inserter(out));

		return out;
	}

	template<typename PointT>
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr XYZ_to_XYZRGB(const typename pcl::PointCloud<PointT>::Ptr &cloud_in, const bool gray = false, const double clip_lower = std::numeric_limits<double>::max(), const double clip_upper = std::numeric_limits<double>::min())
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
		double min_Distance = std::max(distance<PointT>(getNearOrFarthestPoint<PointT>(cloud_in)), clip_lower);
		double max_Distance = std::min(distance<PointT>(getNearOrFarthestPoint<PointT>(cloud_in, false)), clip_upper);
		double div = max_Distance - min_Distance;
        int division_num = getDivNum<size_t, size_t>(cloud_in->points.size());

		cloud_out->points = XYZ_to_XYZRGBPart(division_num, min_Distance, div, gray, cloud_in->points.begin(), cloud_in->points.end(), clip_lower, clip_upper);
		cloud_out->width = static_cast<uint32_t>(cloud_out->points.size());
		cloud_out->height = 1;

		return cloud_out;
	}

#pragma endregion XYZ_to_XYZRGB

#pragma region fillColor

	template<typename RandomIt>
	int fillColorPart(const int &division_num, const uint8_t &r, const uint8_t &g, const uint8_t &b, const RandomIt &beg, const RandomIt &end)
	{
		auto len = end - beg;

		if(len < division_num)
		{
			int out = 0;
			uint32_t rgb;
			for(auto it = beg; it != end; ++it)
			{
				rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
				(*it).rgb = *reinterpret_cast<float*>(&rgb);
			}
			return out;
		}
		auto mid = beg + len/2;
		auto handle = std::async(std::launch::async, fillColorPart<RandomIt>, division_num, r, g, b, beg, mid);
		auto out1 = fillColorPart<RandomIt>(division_num, r, g, b, mid, end);
		auto out = handle.get();

		return out + out1;
	}

	int fillColor(const typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const uint8_t &r, const uint8_t &g, const uint8_t &b)
	{
        int division_num = getDivNum<size_t, size_t>(cloud->points.size());
		
		return fillColorPart<decltype(cloud->points.begin())>(division_num, r, g, b, cloud->points.begin(), cloud->points.end());
	}

	template<typename PointT>
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr fillColor(const typename pcl::PointCloud<PointT>::Ptr &cloud, const uint8_t &r, const uint8_t &g, const uint8_t &b)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointXYZRGB point;
		uint32_t rgb;

		for(auto it = cloud->points.begin(); it != cloud->points.end(); ++it)
		{
			point.x = (*it).x;
			point.y = (*it).y;
			point.z = (*it).z;
			rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
			point.rgb = *reinterpret_cast<float*>(&rgb);
			out->points.push_back (point);
		}
		
		out->width = static_cast<uint32_t>(out->points.size());
		out->height = 1;

		return out;
	}

#pragma endregion fillColor

#pragma region getOrigin

	template<typename RandomIt, typename PointT>
	PointT getOriginPart(const int &division_num, const RandomIt &beg, const RandomIt &end)
	{
		auto len = end - beg;

		if(len < division_num)
		{
			PointT out;
			out.x = 0;
			out.y = 0;
			out.z = 0;
			for(auto it = beg; it != end; ++it)
			{
				out.x += (*it).x;
				out.y += (*it).y;
				out.z += (*it).z;
			}
			return out;
		}
		auto mid = beg + len/2;
		auto handle = std::async(std::launch::async, getOriginPart<RandomIt, PointT>, division_num, beg, mid);
		auto out = getOriginPart<RandomIt, PointT>(division_num, mid, end);
		auto out1 = handle.get();

		out.x += out1.x;
		out.y += out1.y;
		out.z += out1.z;

		return out;
	}
	template<typename PointT>
	PointT getOrigin(const typename pcl::PointCloud<PointT>::Ptr &cloud)
	{
        int division_num = getDivNum<size_t, size_t>(cloud->points.size());
		
		PointT out = getOriginPart<decltype(cloud->points.begin()), PointT>(division_num, cloud->points.begin(), cloud->points.end());
		
		out.x = out.x / static_cast<double>(cloud->points.size());
		out.y = out.y / static_cast<double>(cloud->points.size());
		out.z = out.z / static_cast<double>(cloud->points.size());

		return out;
	}

#pragma endregion getOrigin

#pragma region offsetToOrigin

	template<typename RandomIt, typename PointT>
	int offsetToOriginPart(const int &division_num, const PointT &point, const RandomIt &beg, const RandomIt &end)
	{
		auto len = end - beg;

		if(len < division_num)
		{
			int out;
			for(auto it = beg; it != end; ++it)
			{
				(*it).x -= point.x;
				(*it).y -= point.y;
				(*it).z -= point.z;
				out++;
			}
			return out;
		}
		auto mid = beg + len/2;
		auto handle = std::async(std::launch::async, offsetToOriginPart<RandomIt, PointT>, division_num, point, beg, mid);
		auto out = offsetToOriginPart<RandomIt, PointT>(division_num, point, mid, end);
		auto out1 = handle.get();

		return out + out1;
	}
	template<typename PointT>
	int offsetToOrigin(const typename pcl::PointCloud<PointT>::Ptr &cloud)
	{
        int division_num = getDivNum<size_t, size_t>(cloud->points.size());
		
		return offsetToOriginPart(division_num, getOrigin<PointT>(cloud), cloud->points.begin(), cloud->points.end());
	}

#pragma endregion offsetToOrigin

#pragma region combineCloud

	template<typename PointT>
	void mergeCloud(typename pcl::PointCloud<PointT>::Ptr &cloudMain, typename pcl::PointCloud<PointT>::Ptr &cloud)
	{
		std::copy(cloud->points.begin(), cloud->points.end(), std::back_inserter(cloudMain->points));
		cloudMain->width = static_cast<uint32_t>(cloudMain->points.size());
		cloudMain->height = 1;	
	}

	template<typename PointT>
	typename pcl::PointCloud<PointT>::Ptr combineCloud(typename pcl::PointCloud<PointT>::Ptr &cloud_1, typename pcl::PointCloud<PointT>::Ptr &cloud_2)
	{
		typename pcl::PointCloud<PointT>::Ptr cloud_out(new pcl::PointCloud<PointT>);
		std::copy(cloud_1->points.begin(), cloud_1->points.end(), std::back_inserter(cloud_out->points));
		std::copy(cloud_2->points.begin(), cloud_2->points.end(), std::back_inserter(cloud_out->points));
		cloud_out->width = static_cast<uint32_t>(cloud_out->points.size());
		cloud_out->height = 1;	
		return cloud_out;
	}

#pragma endregion combineCloud
	/*
#pragma region points_to_pcl

	template<typename PointT>
	typename pcl::PointCloud<PointT>::Ptr points_to_pcl(const rs2::points &points)
	{
        int division_num = getDivNum<size_t, size_t>(points.size());
		
		typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

		auto sp = points.get_profile().as<rs2::video_stream_profile>();
		
		auto ptr = points.get_vertices();
		for(int i = 0; i < points.size(); i++)
		{
			if(fabs(ptr->x*ptr->y*ptr->z) != 0)
			{
				PointT p;
				p.x = ptr->x;
				p.y = ptr->y;
				p.z = ptr->z;
				cloud->points.push_back(p);
			}
			ptr++;
		}

		cloud->width = static_cast<uint32_t>(cloud->points.size());
		cloud->height = 1;
		return cloud;
	}

#pragma endregion points_to_pcl
	*/
#pragma region getChanges

	template<typename PointT>
	typename pcl::PointCloud<PointT>::Ptr getChanges(const typename pcl::PointCloud<PointT>::Ptr &cloud1, const typename pcl::PointCloud<PointT>::Ptr &cloud2, const double &resolution)
	{
		typename pcl::PointCloud<PointT>::Ptr temp(new typename pcl::PointCloud<PointT>);
		pcl::octree::OctreePointCloudChangeDetector<PointT> octree (resolution);
		std::vector<int> newPointIdxVector;

		octree.setInputCloud(cloud1);
		octree.addPointsFromInputCloud();
		octree.switchBuffers ();
		octree.setInputCloud (cloud2);
		octree.addPointsFromInputCloud ();
		octree.getPointIndicesFromNewVoxels (newPointIdxVector);

		for(auto it = newPointIdxVector.begin(); it != newPointIdxVector.end(); ++it)
		{
			temp->points.push_back(cloud2->points[*it]);
		}

		temp->width = static_cast<uint32_t>(temp->points.size());
		temp->height = 1;

		return temp;
	}

#pragma endregion getChanges

#pragma region getNoChanges

	template<typename PointT>
	typename pcl::PointCloud<PointT>::Ptr getNoChanges(const typename pcl::PointCloud<PointT>::Ptr &cloud1, const typename pcl::PointCloud<PointT>::Ptr &cloud2, const double &resolution)
	{
		typename pcl::PointCloud<PointT>::Ptr temp(new typename pcl::PointCloud<PointT>);
		pcl::octree::OctreePointCloudChangeDetector<PointT> octree (resolution);
		std::vector<int> newPointIdxVector;

		octree.setInputCloud(cloud1);
		octree.addPointsFromInputCloud();
		octree.switchBuffers ();
		octree.setInputCloud (cloud2);
		octree.addPointsFromInputCloud ();
		octree.getPointIndicesFromNewVoxels (newPointIdxVector);

		std::sort(newPointIdxVector.begin(), newPointIdxVector.end());

		int it1;
		auto it2 = newPointIdxVector.begin();
		for(it1 = 0, it2 = newPointIdxVector.begin(); it1 < cloud2->points.size() && it2 != newPointIdxVector.end(); it1++) 
		{
			if(it1 != (*it2))
			{
				temp->points.push_back(cloud2->points[it1]);
			} 
			else 
			{
				it2++;
			}
		}
		if(it2 == newPointIdxVector.end()) {
			for(int i = it1; i < cloud2->points.size(); i++) {
				temp->points.push_back(cloud2->points[i]);
			}
		}
		temp->width = static_cast<uint32_t>(temp->points.size());
		temp->height = 1;

		return temp;
	}

#pragma endregion getChanges

#pragma region removeTooClosedPoint

	template<typename RandomIt, typename PointT>
	std::vector<PointT> removeTooClosedPointPart(RandomIt &beg, RandomIt &end, typename pcl::PointCloud<PointT>::Ptr &cloud)
	{
		std::vector<PointT> r;

		int i = 0;

		for(auto it = beg; it != end; ++it)
		{
			r.push_back(cloud->points[*it]);
		}

		return r;
	}

	template<typename PointT>
	void removeTooClosedPoint(typename pcl::PointCloud<PointT>::Ptr &cloud, const double &resolution = 1.0) 
	{
		typename pcl::PointCloud<PointT>::Ptr cloud_out(new typename pcl::PointCloud<PointT>());
		pcl::octree::OctreePointCloudSearch<PointT> octree(resolution);

		octree.setInputCloud(cloud);
		octree.addPointsFromInputCloud();
		std::vector<int> deletePointIdxVector;
		for(auto it = octree.leaf_begin(); it != octree.leaf_end(); it++) 
		{
			pcl::octree::OctreeContainerPointIndices c = it.getLeafContainer();
			std::vector<int> allPointIdxVector;
			c.getPointIndices(allPointIdxVector);
			allPointIdxVector.erase(allPointIdxVector.begin());
			std::copy(allPointIdxVector.begin(), allPointIdxVector.end(), std::back_inserter(deletePointIdxVector));
		}
		int i = 0;
		std::sort(deletePointIdxVector.begin(), deletePointIdxVector.end());
		for(const int &index : deletePointIdxVector)
		{
			cloud->points.erase(cloud->points.begin() + index - i);
			i++;
		}

		cloud->width = static_cast<uint32_t>(cloud->points.size());
		cloud->height = 1;

		/*
		cloud_out->points = myFunction::multiThread(deletePointIdxVector, 5, removeTooClosedPointPart, cloud);
		
		cloud_out->width = static_cast<uint32_t>(cloud_out->points.size());
		cloud_out->height = 1;
		*/
	}

#pragma endregion

#pragma region printCamera

	void printCamera(const pcl::visualization::Camera &camera)
	{
		std::cout << "Cam: " << endl;
        std::cout << " - pos: (" << camera.pos[0] << ", "    << camera.pos[1] << ", "    << camera.pos[2] << ")" << endl;
        std::cout << " - view: ("    << camera.view[0] << ", "   << camera.view[1] << ", "   << camera.view[2] << ")"    << endl;
        std::cout << " - focal: ("   << camera.focal[0] << ", "  << camera.focal[1] << ", "  << camera.focal[2] << ")"   << endl;
	}

#pragma endregion printCamera

#pragma region showCloud

	void showCloud(const boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const std::string name, const double &size = 1.0)
	{
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
		viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, name);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, name);
	}

	template<typename PointT>
	void showCloud(const boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, const typename pcl::PointCloud<PointT>::Ptr &cloud, const std::string name, const double &size = 1.0, const bool gray = false, const double clip_lower = std::numeric_limits<double>::max(), const double clip_upper = std::numeric_limits<double>::min())
	{
		auto cloud_rgb = XYZ_to_XYZRGB<PointT>(cloud, gray, clip_lower, clip_upper);
		showCloud(viewer, cloud_rgb, name, size);
	}

	void showCloudWithText(const boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const std::string name, const double &size, const std::string text = "")
	{
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
		viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, name);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, name);
		viewer->addText3D(text, cloud->points[0], 0.01, 1.0, 1.0, 1.0, name + "_text");
	}

#pragma endregion showCloud

#pragma region updateCloud

	void updateCloud(const boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const std::string name, const double &size = 1.0)
	{
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);

		if( !viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, name))
		{
			viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, name);
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, name);
		}
	}

	void updateCloud(const boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::string name, const uint8_t r, const uint8_t g, const uint8_t b)
	{
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(cloud, r, g, b);

		if( !viewer->updatePointCloud<pcl::PointXYZ> (cloud, rgb, name))
		{
			viewer->addPointCloud<pcl::PointXYZ> (cloud, rgb, name);
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, name);
		}
	}

	template<typename PointT>
	void updateCloud(const boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, const typename pcl::PointCloud<PointT>::Ptr &cloud, const std::string name, const double &size = 1.0, const bool gray = false, const double clip_lower = std::numeric_limits<double>::min(), const double clip_upper = std::numeric_limits<double>::max())
	{
		auto cloud_rgb = XYZ_to_XYZRGB<PointT>(cloud, gray, clip_lower, clip_upper);
		updateCloud(viewer, cloud_rgb, name, size);
	}

	void updateCloud(const boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const std::string name, const double &size, const std::string text = "")
	{
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);

		if( !viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, name))
		{
			viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, name);
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, name);
		}
		viewer->removeText3D(name + "_text");
		viewer->addText3D(text, cloud->points[0], 1.0, 1.0, 1.0, 1.0, name + "_text");
	}

#pragma endregion updateCloud

#pragma region removeCloud

	void removeCloud(const boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, const std::string name)
	{
		viewer->removePointCloud(name);
	}

	void removeCloudWithText(const boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, const std::string name)
	{
		viewer->removePointCloud(name);
		viewer->removeText3D(name + "_text");
	}

#pragma endregion showCloud

#pragma region getSimilarity

	template<typename PointT>
	double getSimilarity(const typename pcl::PointCloud<PointT>::Ptr &cloud1, const typename pcl::PointCloud<PointT>::Ptr &cloud2, const double &resolution)
	{
        pcl::octree::OctreePointCloudChangeDetector<PointT> octree (resolution);
        std::vector<int> newPointIdxVector;

        octree.setInputCloud(cloud1);
        octree.addPointsFromInputCloud();
        octree.switchBuffers ();
        octree.setInputCloud (cloud2);
        octree.addPointsFromInputCloud ();
        octree.getPointIndicesFromNewVoxels (newPointIdxVector);

        return 1 - (static_cast<double>(newPointIdxVector.size()) / static_cast<double>(cloud2->points.size()));
	}

#pragma endregion getSimilarity

#pragma region bagFileNameToMilliseconds

	std::chrono::milliseconds bagFileNameToMilliseconds(const std::string &bagFileName)
	{
		if(fileExists(bagFileName))
		{
			std::tm tm;
			std::istringstream ss(bagFileName.substr(bagFileName.rfind('/') + 1, bagFileName.rfind('.')-bagFileName.rfind('/')-1));
			ss >> std::get_time(&tm,"%Y%m%d_%H%M%S");
			return std::chrono::milliseconds(std::mktime(&tm)*1000);
		}
		return std::chrono::milliseconds(0);
	}

#pragma endregion bagFileNameToMilliseconds

	template<typename PointT>
	PointT vector_plane_cross_point(const PointT &v, const PointT &n, const PointT &p0)
	{
		PointT result;
		double t = (p0.x*n.x+p0.y*n.y+p0.z*n.z)/(v.x*n.x+v.y*n.y+v.z*n.z);

		result.x = v.x*t;
		result.y = v.y*t;
		result.z = v.z*t;
	}

	void createColor(const int &number, uint8_t &r, uint8_t &g, uint8_t &b)
	{
		switch(number)
		{
			case 0:
				r = 255;
				g = 255;
				b = 255;
				break;
			case 1:
				r = 255;
				g = 0;
				b = 0;
				break;
			case 2:
				r = 0;
				g = 255;
				b = 0;
				break;
			case 3:
				r = 0;
				g = 0;
				b = 255;
				break;
			case 4:
				r = 255;
				g = 255;
				b = 0;
				break;
			case 5:
				r = 255;
				g = 0;
				b = 255;
				break;
			case 6:
				r = 0;
				g = 255;
				b = 255;
				break;
			case 7:
				r = 255;
				g = 128;
				b = 0;
				break;
			case 8:
				r = 128;
				g = 0;
				b = 255;
				break;
			case 9:
				r = 0;
				g = 255;
				b = 128;
				break;
		}
	}

	bool name_to_color(const std::string &name, uint8_t &r, uint8_t &g, uint8_t &b)
	{
		if(name == "person")
		{
			r = 242;
			g = 181;
			b = 102;
		}
		else if(name == "bicycle")
		{
			r = 67;
			g = 188;
			b = 37;
		}
		else if(name == "car")
		{
			r = 0;
			g = 0;
			b = 200;
		}
		else if(name == "motorbike")
		{
			r = 200;
			g = 0;
			b = 0;
		}
		else if(name == "aeroplane")
		{
			r = 255;
			g = 255;
			b = 255;
		}
		else if(name == "bus")
		{
			r = 0;
			g = 200;
			b = 200;
		}
		else if(name == "train")
		{
			r = 200;
			g = 100;
			b = 0;
		}
		else if(name == "truck")
		{
			r = 200;
			g = 200;
			b = 0;
		}
		else if(name == "boat")
		{
			r = 255;
			g = 255;
			b = 255;
		}
		else if(name == "traffic light")
		{
			r = 255;
			g = 255;
			b = 255;
		}
		else if(name == "fire hydrant")
		{
			r = 255;
			g = 0;
			b = 0;
		}
		else if(name == "stop sign")
		{
			r = 255;
			g = 0;
			b = 0;
		}
		else if(name == "parking meter")
		{
			r = 0;
			g = 0;
			b = 255;
		}
		else if(name == "cup")
		{
			r = 0;
			g = 0;
			b = 255;
		}
		else
		{
			return false;
		}
		return true;
	}
	
	template<typename PointT>
	typename pcl::PointCloud<PointT>::Ptr outlierRemoval(const typename pcl::PointCloud<PointT>::Ptr &cloud, const int meanK = 50, const double StddevMulThresh = 1.0)
	{
		typename pcl::PointCloud<PointT>::Ptr temp;
		pcl::StatisticalOutlierRemoval<PointT> sor;
		sor.setInputCloud (cloud);
		sor.setMeanK (meanK);
		sor.setStddevMulThresh (StddevMulThresh);
		sor.filter (*temp);
		return temp;
	}
}
#endif