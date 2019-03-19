#ifndef MICROSTOPWATCH_H_
#define MICROSTOPWATCH_H_

#include "../include/basic_function.h"
#include <boost/algorithm/string.hpp>

namespace myClass
{
    class MicroStopwatch
    {
        private:
            boost::posix_time::ptime tictic;
            boost::posix_time::ptime toctoc;
            std::string name;
            bool stoped;

        public:
            int64_t elapsed;
            MicroStopwatch()
            {
                elapsed = 0;
                stoped = false;
            }
            MicroStopwatch(const std::string &name)
            {
                elapsed = 0;
                this->name = name;
            }
            void rename(const std::string &name)
            {
                this->name = name;
            }
            void tic()
            {
                tictic = boost::posix_time::microsec_clock::local_time ();
            }
            void tic(const std::string &name)
            {
                this->rename(name);
                tictic = boost::posix_time::microsec_clock::local_time ();
            }
            int64_t toc()
            {
                toctoc = boost::posix_time::microsec_clock::local_time ();
                return (toctoc - tictic).total_microseconds();
            }
            std::string toc_string()
            {
                toctoc = boost::posix_time::microsec_clock::local_time ();
                return myFunction::commaFix((toctoc - tictic).total_microseconds());
            }
            void toc_print_string()
            {
                std::cout << this->name << ": "<< this->toc_string() << " us" <<  std::endl;
            }
            std::string elapsed_string()
            {
                return myFunction::commaFix(elapsed);
            }
            void elapsed_print_string()
            {
                std::cout << this->name << ": "<< this->elapsed_string() << " us" <<  std::endl;
            }
            int64_t toc_pre()
            {
                return (toctoc - tictic).total_microseconds();
            }
            std::string toc_pre_string()
            {
                return myFunction::commaFix((toctoc - tictic).total_microseconds());
            }
            void clear()
            {
                this->elapsed = 0;
            }
            void start()
            {
                if(this->stoped) this->clear();
                this->tic();
            }
            void start(const std::string &name)
            {
                this->rename(name);
                if(this->stoped) this->clear();
                this->tic();
            }
            void pause()
            {
                this->elapsed += this->toc();
            }
            void stop()
            {
                this->pause();
                this->stoped = true;
            }
            void restart()
            {
                this->clear();
                this->start();
            }
            void restart(const std::string &name)
            {
                this->rename(name);
                this->clear();
                this->start();
            }
            friend std::ostream& operator<<(std::ostream &out, MicroStopwatch &obj)
            {
                out << obj.name << ": "<< obj.elapsed_string() << " us" <<  std::endl;
                return out;
            }
    };
}
#endif