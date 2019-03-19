#ifndef BASIC_FUNCTION_H_
#define BASIC_FUNCTION_H_

#include <iostream>
#include <thread>
#include <future>
#include "../include/date.h"
#ifndef TIMEZONE
#define TIMEZONE 8
#endif
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
	template<typename RandomIt1, typename RandomIt2, typename RandomIt3> 
    bool check_is_close(RandomIt1 a, RandomIt2 b, RandomIt3 tolerance) {
        return std::fabs(a - b) < tolerance;
    }

	template<typename RandomIt1, typename RandomIt2> 
	size_t getDivNum(const RandomIt1 &total, const RandomIt2 part = (std::thread::hardware_concurrency()+1))
	{
		return std::ceil(static_cast<double>(total)/static_cast<double>(part));
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
/*
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
*/
    template<typename _R, typename _VType, typename _Fn, typename... _Args>
    std::vector<_R>& multiThread(std::vector<_VType> &v, const int64_t multiNum, _Fn&& __fn, _Args&&... __args)
    {
        int64_t d = v.size() / multiNum;    
        
        std::vector<std::vector<_R>> vv;

        for(int64_t i = 0; i < v.size(); i += d)
        {
            auto beg = v.begin() + i;
            auto end = v.begin() + i + d;
            if(end > v.end()) end = v.end();
            auto handle = std::async(std::launch::async, __fn, beg, end, __args...);
            vv.push_back(handle);
		    //std::vector<_R> out = handle.get();
        }
    }

}
#endif