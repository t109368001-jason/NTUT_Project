#ifndef LASER_FRAME_H_
#define LASER_FRAME_H_

#include <vector>
#include "laser_line.h"
#include "../microStopwatch.h"

namespace velodyne {

    class LineFrame {
        public:
            std::vector<LaserLine> laserLines;

            int64_t find(const velodyne::Laser& laser) {
                for(int64_t i = 0; i < this->laserLines.size(); i++) {
                    if(myFunction::check_is_close(this->laserLines[i].azimuth, laser.azimuth, 1e-6)) return i;
                }
                return -1;
            }

            size_t weight() {
                return this->laserLines.size();
            }

            size_t height() {
                return (this->weight() > 0 ? this->laserLines[0].lasers.size() : 0);
            }

            void add(const velodyne::Laser& laser) {
                int64_t index;
                if((index = find(laser)) < 0) {
                    LaserLine laserLine(laser);
                    this->laserLines.push_back(laserLine);
                } else {
                    
                    this->laserLines[index].add(laser);
                }
            }

            void removeDuplicate() {
                for(LaserLine& laserLine : this->laserLines) {
                    laserLine.removeDuplicate();
                }
            }

            void operator << (std::vector<velodyne::Laser>& lasers) {
                for(const velodyne::Laser& laser : lasers) {
                    this->add(laser);
                }
                this->removeDuplicate();
            }

            friend std::ostream& operator << (std::ostream& out, LineFrame& lineFrame) {
                out << "height: " << lineFrame.height() << std::endl;
                out << "weight: " << lineFrame.weight() << std::endl;
                return out;
            }
    };
}

#endif