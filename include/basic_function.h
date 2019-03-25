#ifndef BASIC_FUNCTION_H_
#define BASIC_FUNCTION_H_

#include <iostream>
#include <thread>
#include <future>
#include <sys/stat.h>
#include "../include/date.h"
#include "microStopwatch.h"
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
	bool fileExists(const std::string &filename)
	{
		struct stat buffer;
		return (stat(filename.c_str(), &buffer) == 0);
	}

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
    template<typename _Result, typename _Vector, typename _Fn, typename _FnResult, typename Fn_O, typename... _Args>
    _Result multiThread(std::vector<_Vector> &v, const int64_t multiNum, _Fn&& __fn, _Fn_O&& __fn_o, _Args&&... __args)
    {
        int64_t d = std::ceil(v.size() / multiNum);    
        
        std::vector<std::future<void>> threadVector;
        std::vector<_FnResult> resultVector;

        for(int64_t i = 0; i < v.size(); i += d)
        {
            auto beg = v.begin() + i;
            auto end = v.begin() + ((i+d) > v.size() ? v.size() : (i+d));
            if(end > v.end()) end = v.end();
            std::future<_FnResult> handle = std::async(std::launch::async, __fn, beg, end, __args...);
            threadVector.push_back(handle);
        }

        for(auto thread : threadVector) {
            resultVector.push_back(thread.get());
        }

        return __fn_o(resultVector);
    }
    */
}
#endif