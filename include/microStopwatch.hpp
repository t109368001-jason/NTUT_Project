#ifndef MICROSTOPWATCH_H_
#define MICROSTOPWATCH_H_

#include <boost/date_time/posix_time/posix_time.hpp>
#include <basic_function.hpp>

namespace myClass
{
    static int microStopwatchCount = 0;
    class MicroStopwatch
    {
        private:
            boost::posix_time::ptime tictic;
            boost::posix_time::ptime toctoc;
            std::string name;
            bool stoped;
            bool ticPrintName;

        public:
            int64_t elapsedCurrent;
            int64_t elapsedTotal;

            MicroStopwatch();
            MicroStopwatch(std::string name);

            void rename(const std::string &name);

            void tic(bool ticPrintName = false);

            std::string toc_string();

            void toc();

            std::string elapsed_string();

            void elapsed();
    };
}

#endif