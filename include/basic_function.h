#ifndef BASIC_FUNCTION_H_
#define BASIC_FUNCTION_H_

#include <iostream>
#include <thread>
#include <future>
#include <cmath>
#include <iomanip>
#include <sys/stat.h>
#include <boost/filesystem.hpp>
#include <pcl/visualization/pcl_visualizer.h>

#include <include/date.h>
#ifndef TIMEZONE
#define TIMEZONE 8
#endif

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef boost::shared_ptr<PointCloudT> PointCloudPtrT;
typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> ViewerPtrT;

namespace boost{namespace filesystem{

    boost::filesystem::path relative(boost::filesystem::path from, boost::filesystem::path to)
    {
    // Start at the root path and while they are the same then do nothing then when they first
    // diverge take the entire from path, swap it with '..' segments, and then append the remainder of the to path.
    boost::filesystem::path::const_iterator fromIter = from.begin();
    boost::filesystem::path::const_iterator toIter = to.begin();

    // Loop through both while they are the same to find nearest common directory
    while (fromIter != from.end() && toIter != to.end() && (*toIter) == (*fromIter))
    {
        ++toIter;
        ++fromIter;
    }

    // Replace from path segments with '..' (from => nearest common directory)
    boost::filesystem::path finalPath;
    while (fromIter != from.end())
    {
        finalPath /= "..";
        ++fromIter;
    }

    // Append the remainder of the to path (nearest common directory => to)
    while (toIter != to.end())
    {
        finalPath /= *toIter;
        ++toIter;
    }

    return finalPath;
    }
}}

namespace myFunction
{
	bool fileExists(const std::string &filename)
	{
		struct stat buffer;
		return (stat(filename.c_str(), &buffer) == 0);
	}

	template<typename RandomIt1, typename RandomIt2, typename RandomIt3> 
    bool check_is_close(RandomIt1 a, RandomIt2 b, RandomIt3 tolerance) {
        return std::fabs(a - b) < tolerance;
    }

	template<typename Type>
	std::string commaFix(const Type &input)		//1000000 -> 1,000,000
	{
        std::stringstream ss;
        ss.imbue(std::locale(""));
        ss << std::fixed << input;
        
		return ss.str();
	}

    template<typename RandomIt>
    std::string durationToString(const RandomIt &duration, const bool isFileName = true)
    {
        std::ostringstream stream;
        std::chrono::time_point<std::chrono::system_clock> tp = std::chrono::time_point<std::chrono::system_clock>(duration);
        tp += std::chrono::hours(TIMEZONE);
        auto dp = date::floor<date::days>(tp);  // dp is a sys_days, which is a
                                        // type alias for a C::time_point
        auto date = date::year_month_day{dp};
        auto time = date::make_time(std::chrono::duration_cast<std::chrono::milliseconds>(tp-dp));
        stream << std::setfill('0') << std::setw(4) << date.year().operator int();
        if(!isFileName) stream << '/';
        stream << std::setfill('0') << std::setw(2) << date.month().operator unsigned int();
        if(!isFileName) stream << '/';
        stream << std::setfill('0') << std::setw(2) << date.day().operator unsigned int();
        if(!isFileName) stream << ' ';
        else stream << '_';
        stream << std::setfill('0') << std::setw(2) << time.hours().count();
        if(!isFileName) stream << ':';
        stream << std::setfill('0') << std::setw(2) << time.minutes().count();
        if(!isFileName) stream << ':';
        stream << std::setfill('0') << std::setw(2) << time.seconds().count();
        if(!isFileName) stream << '.';
        else stream << '_';
        stream << std::setfill('0') << std::setw(3) << time.subseconds().count();
        if(typeid(RandomIt) == typeid(std::chrono::microseconds))
        {
            stream << std::setfill('0') << std::setw(3) << duration.count() % 1000;
        }
        if(typeid(RandomIt) == typeid(std::chrono::nanoseconds))
        {
            stream << std::setfill('0') << std::setw(3) << duration.count() % 1000000;
        }
        return stream.str();
    }

    template<typename IterItemT>
    std::vector<float> getPointCloudRange(const int splitCount, const IterItemT &beg, const IterItemT &end) {
		auto len = end - beg;

		if(splitCount == 0)
		{
            std::vector<float> range(2);
            range[0] = std::numeric_limits<float>::max();
            range[1] = std::numeric_limits<float>::min();
			for(auto it = beg; it != end; ++it)
			{
                range[0] = std::fmin(range[0], std::sqrt((*it).x*(*it).x+(*it).y*(*it).y+(*it).z*(*it).z));
                range[1] = std::fmax(range[1], std::sqrt((*it).x*(*it).x+(*it).y*(*it).y+(*it).z*(*it).z));
			}
			return range;
		}
		auto mid = beg + len/2;
		auto handle = std::async(std::launch::async, getPointCloudRange<IterItemT>, splitCount-1, beg, mid);
		auto range = getPointCloudRange<IterItemT>(splitCount-1, mid, end);
		auto range1 = handle.get();

        range[0] = std::fmin(range[0], range1[0]);
        range[1] = std::fmax(range[1], range1[1]);

		return range;
    }

	void valueToRGB(uint8_t &r, uint8_t &g, uint8_t &b, const float &value)
	{
        float tmp = value;
        tmp *= 1280.0;
        tmp += 128.0;
        if(tmp >= 0 && tmp < 256) {
			r = 0;
			g = 0;
			b = tmp;
        } else if(tmp >= 256 && tmp < 512) {
			r = 0;
			g = tmp - 255;
			b = 255;
        } else if(tmp >= 512 && tmp < 768) {
			r = 0;
			g = 255;
			b = 767-tmp;
        } else if(tmp >= 768 && tmp < 1024) {
			r = tmp - 767;
			g = 255;
			b = 0;
        } else if(tmp >= 1024 && tmp < 1280) {
			r = 255;
			g = 1279-tmp;
			b = 0;
        } else {
			r = 1535-tmp;
			g = 0;
			b = 0;
        }
	}

	void valueToRGB(uint32_t &rgb, const float &value)
	{
        float tmp = value;
        uint8_t r, g, b;
        tmp *= 1280.0;
        tmp += 128.0;
        if(tmp >= 0 && tmp < 256) {
			r = 0;
			g = 0;
			b = tmp;
        } else if(tmp >= 256 && tmp < 512) {
			r = 0;
			g = tmp - 255;
			b = 255;
        } else if(tmp >= 512 && tmp < 768) {
			r = 0;
			g = 255;
			b = 767-tmp;
        } else if(tmp >= 768 && tmp < 1024) {
			r = tmp - 767;
			g = 255;
			b = 0;
        } else if(tmp >= 1024 && tmp < 1280) {
			r = 255;
			g = 1279-tmp;
			b = 0;
        } else {
			r = 1535-tmp;
			g = 0;
			b = 0;
        }
        rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
	}

    template<typename IterItemT>
    std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> XYZ_to_XYZRGBPart(const int splitCount, const IterItemT &beg, const IterItemT &end, const std::vector<float> &range) {
		auto len = end - beg;

		if(splitCount == 0)
		{
            std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> points;
            float div = range[1] - range[0];
			for(auto it = beg; it != end; ++it)
			{
                pcl::PointXYZRGB point;
                uint32_t rgb;
                point.x = (*it).x;
                point.y = (*it).y;
                point.z = (*it).z;
                float d = std::fmin(range[1], std::sqrt(point.x*point.x+point.y*point.y+point.z*point.z));
                float v = float(d-range[0]) / div;
                v = std::fmin(v, 1.0);

                valueToRGB(rgb, v);
                point.rgb = *reinterpret_cast<float*>(&rgb);
                points.push_back(point);
			}
			return points;
		}
		auto mid = beg + len/2;
		auto handle = std::async(std::launch::async, XYZ_to_XYZRGBPart<IterItemT>, splitCount-1, beg, mid, range);
		auto points = XYZ_to_XYZRGBPart<IterItemT>(splitCount-1, mid, end, range);
		auto points1 = handle.get();

        std::copy(points1.begin(), points1.end(), std::back_insert_iterator(points));

		return points;
    }

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> XYZ_to_XYZRGB(const PointCloudPtrT &cloud, const float maxRange) {
        std::vector<float> range = getPointCloudRange<decltype(cloud->points.begin())>(std::log2(std::thread::hardware_concurrency()+1), cloud->points.begin(), cloud->points.end());
        
        range[1] = std::fmin(range[1], maxRange);

        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);

        cloud_rgb->points = XYZ_to_XYZRGBPart<decltype(cloud->points.begin())>(std::log2(std::thread::hardware_concurrency()+1), cloud->points.begin(), cloud->points.end(), range);
        cloud_rgb->width = static_cast<uint32_t>(cloud_rgb->points.size());
		cloud_rgb->height = 1;

		return cloud_rgb;
    }

    void updateCloudPart(const PointT &point, double &) {

    }

    void updateCloud(ViewerPtrT &viewer, const PointCloudPtrT &cloud, const std::string &name, const uint8_t &r = 255, const uint8_t &g = 255, const uint8_t &b = 255) {

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(cloud, r, g, b);

		if( !viewer->updatePointCloud<pcl::PointXYZ> (cloud, rgb, name))
		{
			viewer->addPointCloud<pcl::PointXYZ> (cloud, rgb, name);
		}
    }

    void updateCloud(ViewerPtrT &viewer, const PointCloudPtrT &cloud, const std::string &name, const double maxRange) {

        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_rgb;
        cloud_rgb = XYZ_to_XYZRGB(cloud, maxRange);

		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_rgb);

		if( !viewer->updatePointCloud<pcl::PointXYZRGB> (cloud_rgb, rgb, name))
		{
			viewer->addPointCloud<pcl::PointXYZRGB> (cloud_rgb, rgb, name);
		}
    }
}

#endif