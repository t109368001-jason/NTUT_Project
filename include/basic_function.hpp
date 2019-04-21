#ifndef BASIC_FUNCTION_H_
#define BASIC_FUNCTION_H_

#include <iostream>
#include <future>
#include <sys/stat.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <microStopwatch.hpp>

#include <date.h>
#ifndef TIMEZONE
#define TIMEZONE 8
#endif

typedef boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> PointCloudPtrT;
typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> ViewerPtrT;

namespace boost{namespace filesystem{

    boost::filesystem::path relative(boost::filesystem::path from, boost::filesystem::path to);
}}

namespace myFunction
{
	bool fileExists(const std::string &filename);

	template<typename RandomIt1, typename RandomIt2, typename RandomIt3> 
    bool check_is_close(RandomIt1 a, RandomIt2 b, RandomIt3 tolerance);

	template<typename Type>
	std::string commaFix(const Type &input);

    template<typename RandomIt>
    std::string durationToString(const RandomIt &duration, const bool isFileName);

	void valueToRGB(uint8_t &r, uint8_t &g, uint8_t &b, float value);

	void valueToRGB(uint32_t &rgb, float value);

    std::vector<float> getNormVector(const PointCloudPtrT &cloud);

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> XYZ_to_XYZRGB(const PointCloudPtrT &cloud, const float maxRange);

    void updateCloud(ViewerPtrT &viewer, const PointCloudPtrT &cloud, const std::string &name, const uint8_t &r = 255, const uint8_t &g = 255, const uint8_t &b = 255);

    void updateCloud(ViewerPtrT &viewer, const PointCloudPtrT &cloud, const std::string &name, const double maxRange);
}

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
	bool fileExists(const std::string &filename) {
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
    std::string durationToString(const RandomIt &duration, const bool isFileName)
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

	void valueToRGB(uint8_t &r, uint8_t &g, uint8_t &b, float value) {
        value *= 1280.0;
        value += 128.0;
        if(value >= 0 && value < 256) {
			r = 0;
			g = 0;
			b = value;
        } else if(value >= 256 && value < 512) {
			r = 0;
			g = value - 255;
			b = 255;
        } else if(value >= 512 && value < 768) {
			r = 0;
			g = 255;
			b = 767-value;
        } else if(value >= 768 && value < 1024) {
			r = value - 767;
			g = 255;
			b = 0;
        } else if(value >= 1024 && value < 1280) {
			r = 255;
			g = 1279-value;
			b = 0;
        } else {
			r = 1535-value;
			g = 0;
			b = 0;
        }
	}

	void valueToRGB(uint32_t &rgb, float value) {
        uint8_t r, g, b;
        valueToRGB(r, g, b, value);
        rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
	}

    std::vector<float> getNormVector(const PointCloudPtrT &cloud) {
        std::vector<float> normVector(cloud->points.size());
        std::transform(cloud->points.begin(), cloud->points.end(), normVector.begin(), [](auto &p){return std::sqrt(p.x*p.x+p.y*p.y+p.z*p.z);});
        return normVector;
    }

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> XYZ_to_XYZRGB(const PointCloudPtrT &cloud, const float maxRange) {
        float min, max;
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);

        auto normVector = getNormVector(cloud);
        auto minmax = std::minmax_element(normVector.begin(), normVector.end());
        min = *minmax.first;
        max = std::fmin(*minmax.second, maxRange);

        pcl::copyPointCloud(*cloud, *cloud_rgb);
        
        float div = max - min;
        auto point = cloud_rgb->points.begin();
        auto norm = normVector.begin();
        for(; point != cloud_rgb->points.end(); ++point, ++norm) {
            uint32_t rgb;
            float v = std::fmin(float((*norm)-min) / div, 1.0);
            valueToRGB(rgb, v);
            (*point).rgb = *reinterpret_cast<float*>(&rgb);
        }
        
		return cloud_rgb;
    }

    void updateCloud(ViewerPtrT &viewer, const PointCloudPtrT &cloud, const std::string &name, const uint8_t &r, const uint8_t &g, const uint8_t &b) {

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