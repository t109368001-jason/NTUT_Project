#ifndef LASER_LINE_H_
#define LASER_LINE_H_

#include <vector>
#include "../../3rdparty/VelodyneCapture/VelodyneCapture.h"
#include "../basic_function.h"

namespace velodyne {

    class LaserLine {
        public:
            double azimuth;
            std::vector<velodyne::Laser> lasers;

            LaserLine(velodyne::Laser laser) : azimuth(laser.azimuth) {
                lasers.push_back(laser);
            }

            void add(velodyne::Laser laser) {
                lasers.push_back(laser);
            }

            void removeDuplicate() {
                for(int64_t i = 1; i < lasers.size(); i++) {
                    for(int64_t j = 0; j < i-1; j++) {
                        if(myFunction::check_is_close(lasers[i].vertical, lasers[j].vertical, 1e-6)) {
                            if((lasers[i].distance < lasers[j].vertical)&&(!myFunction::check_is_close(lasers[i].distance, 0.0, 1e-6))) {
                                lasers.erase(lasers.begin() + j);
                                i--;
                            } else {
                                lasers.erase(lasers.begin() + i);
                                i--;
                            }
                            break;
                        }
                    }
                }
            }
    };
}

#endif