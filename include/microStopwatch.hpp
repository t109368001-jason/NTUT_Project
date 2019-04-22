#ifndef MICROSTOPWATCH_H_
#define MICROSTOPWATCH_H_

#include <boost/date_time/posix_time/posix_time.hpp>

namespace myClass
{
    static int microStopwatchCount = 0;
    class MicroStopwatch
    {
    public:
        int64_t elapsed;

        MicroStopwatch();

        MicroStopwatch(std::string name);

        void rename(const std::string &name);

        void start();

        void restart();

        void pause();

        void stop();

        void tic(bool ticPrintName = false);

        void toc();

    private:
        int64_t elapsedCurrent;
        bool stoped_;
        bool ticPrintName_;
        std::string name_;
        boost::posix_time::ptime tictic_;
        boost::posix_time::ptime toctoc_;

        template<typename Type>
        std::string commaFix(const Type &input);
    };
}
using namespace myClass;
MicroStopwatch::MicroStopwatch() : 
    elapsed(0), 
    elapsedCurrent(0), 
    stoped_(true), 
    ticPrintName_(true), 
    name_("MicroStopwatch" + std::to_string(microStopwatchCount++))
    { }

MicroStopwatch::MicroStopwatch(std::string name) : 
    elapsed(0), 
    elapsedCurrent(0), 
    stoped_(true), 
    ticPrintName_(true), 
    name_(name)
    { }

void MicroStopwatch::rename(const std::string &name) {
    name_ = name; 
}

void MicroStopwatch::start() {
    if(stoped_) {
        elapsedCurrent = 0;
        elapsed = 0;
        stoped_ = false;
    }
    tictic_ = boost::posix_time::microsec_clock::local_time ();
}

void MicroStopwatch::restart() {
    stoped_ = true;
    start();
}

void MicroStopwatch::pause() {
    toctoc_ = boost::posix_time::microsec_clock::local_time ();
    elapsedCurrent = (toctoc_ - tictic_).total_microseconds();
    elapsed += elapsedCurrent;
}

void MicroStopwatch::stop() {
    pause();
    stoped_ = true;
    std::cout << name_ + ": " <<  commaFix(elapsedCurrent) + " us" << std::endl;
}

void MicroStopwatch::tic(bool ticPrintName)
{
    ticPrintName_ = ticPrintName;
    start();
    std::cout << (ticPrintName_ ? name_ + "... " : "");
}

void MicroStopwatch::toc()
{
    stop();
    std::cout << (ticPrintName_ ? "" : name_ + ": ") <<  commaFix(elapsedCurrent) + " us" << std::endl;
}

template<typename Type>
std::string MicroStopwatch::commaFix(const Type &input)		//1000000 -> 1,000,000
{
    std::stringstream ss;
    ss.imbue(std::locale(""));
    ss << std::fixed << input;
    
    return ss.str();
}

#endif