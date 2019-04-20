#ifndef BASIC_FUNCTION_H_
#define BASIC_FUNCTION_H_

#include <iostream>
#include <future>
#include <sys/stat.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <date.h>
#ifndef TIMEZONE
#define TIMEZONE 8
#endif

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef boost::shared_ptr<PointCloudT> PointCloudPtrT;
typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> ViewerPtrT;

namespace boost{namespace filesystem{

    boost::filesystem::path relative(boost::filesystem::path from, boost::filesystem::path to);
}}

namespace myFunction
{
	bool fileExists(const std::string &filename);

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

    float distance(const pcl::PointXYZ &point1, const pcl::PointXYZ &point2);

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

	void valueToRGB(uint8_t &r, uint8_t &g, uint8_t &b, const float &value);

	void valueToRGB(uint32_t &rgb, const float &value);

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

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> XYZ_to_XYZRGB(const PointCloudPtrT &cloud, const float maxRange);

    void updateCloud(ViewerPtrT &viewer, const PointCloudPtrT &cloud, const std::string &name, const uint8_t &r = 255, const uint8_t &g = 255, const uint8_t &b = 255);

    void updateCloud(ViewerPtrT &viewer, const PointCloudPtrT &cloud, const std::string &name, const double maxRange);
}

#endif