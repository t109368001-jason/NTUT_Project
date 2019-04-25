#ifndef MICROSTOPWATCH_H_
#define MICROSTOPWATCH_H_

#include <boost/date_time/posix_time/posix_time.hpp>

namespace myClass {
    static int microStopwatchCount = 0;
    
    class MicroStopwatch {
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
        bool stoped_;
        bool paused_;
        bool ticPrintName_;

        int64_t elapsedCurrent_;

        std::string name_;

        boost::posix_time::ptime tictic_;
        boost::posix_time::ptime toctoc_;

        template<typename Type>
        std::string commaFix(const Type &input);
    };
}
using namespace myClass;
MicroStopwatch::MicroStopwatch() : 
    stoped_(true),
    paused_(true), 
    ticPrintName_(true), 
    elapsed(0), 
    elapsedCurrent_(0), 
    name_("MicroStopwatch" + std::to_string(microStopwatchCount++))
    { }

MicroStopwatch::MicroStopwatch(const std::string name) : 
    stoped_(true),
    paused_(true), 
    ticPrintName_(true), 
    elapsed(0), 
    elapsedCurrent_(0), 
    name_(name)
    { }

void MicroStopwatch::rename(const std::string &name) {
    name_ = name; 
}

void MicroStopwatch::start() {
    if(stoped_) {
        elapsedCurrent_ = 0;
        elapsed = 0;
        stoped_ = false;
    }
    if(paused_) tictic_ = boost::posix_time::microsec_clock::local_time ();
}

void MicroStopwatch::restart() {
    stoped_ = true;
    paused_ = true;
    start();
}

void MicroStopwatch::pause() {
    if(!paused_) {
        toctoc_ = boost::posix_time::microsec_clock::local_time ();
        elapsedCurrent_ = (toctoc_ - tictic_).total_microseconds();
        elapsed += elapsedCurrent_;
        paused_ = true;
    }
}

void MicroStopwatch::stop() {
    pause();
    stoped_ = true;
    std::cout << name_ + ": " <<  commaFix(elapsed) + " us" << std::endl;
}

void MicroStopwatch::tic(bool ticPrintName) {
    ticPrintName_ = ticPrintName;
    restart();
    std::cout << (ticPrintName_ ? name_ + "... " : "");
}

void MicroStopwatch::toc() {
    pause();
    std::cout << name_ + ": " <<  commaFix(elapsed) + " us" << std::endl;
}

template<typename Type>
std::string MicroStopwatch::commaFix(const Type &input) {
    std::stringstream ss;
    ss.imbue(std::locale(""));
    ss << std::fixed << input;
    
    return ss.str();
}

#endif