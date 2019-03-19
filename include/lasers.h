#ifndef LASERS_H_
#define LASERS_H_
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../include/stb_image_write.h"
#include "../3rdparty/VelodyneCapture/VelodyneCapture.h"
#include <boost/test/floating_point_comparison.hpp>

namespace myLasers {
    class Line {
        public:
            double azimuth;
            std::vector<velodyne::Laser> lasers;

            Line(velodyne::Laser laser) : azimuth(laser.azimuth) {
                lasers.push_back(laser);
            }

            Line() : azimuth(std::numeric_limits<double>::quiet_NaN()) { }

            void add(velodyne::Laser laser) {
                lasers.push_back(laser);
            }

            void remove_duplicate() {
                for(int i = 0; i < lasers.size(); i++) {
                    for(int j = 0; j < i; j++) {
                        if((lasers[i].azimuth == lasers[j].azimuth)&&(lasers[i].vertical == lasers[j].vertical)) {
                            if((lasers[i].distance < lasers[j].distance)) {
                                lasers.erase(lasers.begin() + i);
                                i--;
                                break;
                            } else {
                                lasers.erase(lasers.begin() + j);
                                i--;
                                break;
                            }
                        }
                    }
                }
            }

            void remove_redundant() {
                for(int i = 0; i < lasers.size(); i++) {
                    if(myFunction::check_is_close(lasers[i].distance, 0.0, 1e-5)) {
                        lasers.erase(lasers.begin() + i);
                        i--;
                    }
                }
            }

            void sort() {
                std::sort(lasers.begin(), lasers.end(), [] (auto a, auto b) { return a.vertical > b.vertical; });
            }

            bool exists(double vertical) {
                for(int i = 0; i < lasers.size(); i++) {
                    if(myFunction::check_is_close(lasers[i].vertical, vertical, 1e-5)) return true;
                }
                return false;
            }

            void fill_empty_point(double d) {
                velodyne::Laser laser;
                laser.azimuth = this->azimuth;
                laser.distance = 0;
                for(double deg = -15.0; deg <= 15.0; deg += d) {
                    if(!this->exists(deg)) {
                        laser.vertical = deg;
                        this->lasers.push_back(laser);
                    }
                    
                }
            }

            void print(std::string prefix, bool basic_info = true) {
                std::cout << prefix << "azimuth : " << this->azimuth << '\t' << "points : " << this->lasers.size() << std::endl;;
                if(!basic_info) {
                    for(int i = 0; i < lasers.size(); i++) {
                        std::cout << prefix << '\t' << i << '\t' << "vertical : " << lasers[i].vertical << '\t' << "distance : " << lasers[i].distance << std::endl;
                    }
                }
            }

            friend ostream& operator << (ostream& out, Line line) {
                out << '\t' << "azimuth : " << line.azimuth << '\t' << "points : " << line.lasers.size() << std::endl;
            }
    };

    class Lines {
        public:
            std::vector<Line> lines;

            void add(velodyne::Laser laser) {
                bool create = true;
                for(int i = 0; i < lines.size(); i++) {
                    if(lines[i].azimuth == laser.azimuth) {
                        lines[i].add(laser);
                        create = false;
                    }
                }
                if(create) {
                    Line line(laser);
                    lines.push_back(line);
                }
            }
            void remove_duplicate() {
                for(int i = 0; i < lines.size(); i++) {
                    lines[i].remove_duplicate();
                }
            }

            void remove_redundant() {
                for(int i = 0; i < lines.size(); i++) {
                    lines[i].remove_redundant();
                }
            }

            void sort() {
                std::sort(lines.begin(), lines.end(), [] (auto a, auto b) { return a.azimuth < b.azimuth; });
                for(int i = 0; i < lines.size(); i++) {
                    lines[i].sort();
                }
            }

            size_t size() {
                size_t num = 0;
                for( const myLasers::Line& line : lines){
                    num += line.lasers.size();
                }
                return num;
            }

            void linear_interpolation(int num) {
                this->sort();
                this->remove_duplicate();
                this->remove_redundant();
                for( myLasers::Line& line : lines){
                    std::vector<velodyne::Laser> lasers_new;
                    for( int i = 1; i < line.lasers.size(); i++ ){
                        for(int j = 0; j < num; j++) {
                            velodyne::Laser laser;
                            laser.azimuth = line.azimuth;
                            laser.distance = (line.lasers[i-1].distance - line.lasers[i].distance) * (j+1) / (num+1) + line.lasers[i].distance;
                            laser.vertical = (line.lasers[i-1].vertical - line.lasers[i].vertical) * (j+1) / (num+1) + line.lasers[i].vertical;
                            laser.intensity = (line.lasers[i-1].intensity - line.lasers[i].intensity) * (j+1) / (num+1) + line.lasers[i].intensity;
                            laser.time = (line.lasers[i-1].time - line.lasers[i].time) * (j+1) / (num+1) + line.lasers[i].time;
                            lasers_new.push_back(laser);
                        }
                    }
		            std::copy(lasers_new.begin(), lasers_new.end(), std::back_inserter(line.lasers));
                }
            }

            void fill_empty_line(double d) {
                this->sort();
                int i = 0;
                while(i < lines.size()) {
                    lines[i].fill_empty_point(d);
                    if((lines[i-1].azimuth-lines[i].azimuth) > d*1.8) {
                        Line line;
                        line.azimuth = lines[i].azimuth-d;
                        this->lines.insert(lines.begin() + i, line);
                        i--;
                        continue;
                    }
                    i++;
                }
            }

            void save_as_png(std::string filename) {
                this->fill_empty_line(0.2);
                this->sort();
                
                double max_distance = std::numeric_limits<double>::min();
                double min_distance = std::numeric_limits<double>::max();
                for(Line line : lines) {
                    for(velodyne::Laser laser : line.lasers) {
                        if(myFunction::check_is_close(laser.distance, 0.0, 1e-5)) continue;
                        max_distance = std::fmax(laser.distance, max_distance);
                        min_distance = std::fmin(laser.distance, min_distance);
                    }
                }
                double div = max_distance - min_distance;
                int width = lines.size();
                int height = lines[0].lasers.size();
                int pixel [width][height][3];
                for(int i = 0; i < lines.size(); i++) {
                    for(int j = 0; j < lines[i].lasers.size(); j++) {
                        pixel[i][j][0] = (lines[i].lasers[j].distance - min_distance) * 255.0 / div;
                        pixel[i][j][0] = (pixel[i][j][0] < 0 ? 0 : 255-pixel[i][j][0]);
                        pixel[i][j][1] = pixel[i][j][0];
                        pixel[i][j][2] = pixel[i][j][0];
                    }
                }

                std::ofstream ofs(filename + ".ppm");

                ofs << "P3" << " " << width << " " << height << " 255" << std::endl;
                for(int h = 0; h < height; h++) {
                    for(int w = 0; w < width; w++) {
                        ofs << pixel[w][h][0] << " " << pixel[w][h][1] << " " << pixel[w][h][2] << '\t';
                    }
                    ofs << std::endl;
                }
                ofs.close();
            }

            void print(std::string prefix, bool basic_info = true) {
                std::cout << prefix << "lines : " << this->lines.size();
                std::cout << prefix << "points : " << this->size();
                for(int i = 0; i < this->lines.size(); i++) {
                    std::cout << i;
                    this->lines[i].print("\t", basic_info);
                }
            }

            friend ofstream& operator << (ofstream& ofs, Lines& lines) {
                lines.fill_empty_line(0.2);
                lines.sort();
                
                double max_distance = std::numeric_limits<double>::min();
                double min_distance = std::numeric_limits<double>::max();
                for(Line line : lines.lines) {
                    for(velodyne::Laser laser : line.lasers) {
                        if(myFunction::check_is_close(laser.distance, 0.0, 1e-5)) continue;
                        max_distance = std::fmax(laser.distance, max_distance);
                        min_distance = std::fmin(laser.distance, min_distance);
                    }
                }
                double div = max_distance - min_distance;
                int width = lines.lines.size();
                int height = lines.lines[0].lasers.size();
                int pixel [width][height][3];
                for(int i = 0; i < lines.lines.size(); i++) {
                    for(int j = 0; j < lines.lines[i].lasers.size(); j++) {
                        pixel[i][j][0] = (lines.lines[i].lasers[j].distance - min_distance) * 255.0 / div;
                        pixel[i][j][0] = (pixel[i][j][0] < 0 ? 0 : 255-pixel[i][j][0]);
                        pixel[i][j][1] = pixel[i][j][0];
                        pixel[i][j][2] = pixel[i][j][0];
                    }
                }

                ofs << "P3" << " " << width << " " << height << " 255" << std::endl;
                for(int h = 0; h < height; h++) {
                    for(int w = 0; w < width; w++) {
                        ofs << pixel[w][h][0] << " " << pixel[w][h][1] << " " << pixel[w][h][2] << '\t';
                    }
                    ofs << std::endl;
                }
                ofs.close();
            }

            friend ostream& operator << (ostream& out, Lines lines) {
                out << "lines : " << lines.lines.size();
                out << "points : " << lines.size();
                for(int i = 0; i < lines.lines.size(); i++) {
                    out << i;
                    out << lines.lines[i];
                }
            }

            friend velodyne::VLP16Capture& operator >> (velodyne::VLP16Capture& vlp16, Lines& lines) {
                std::vector<velodyne::Laser> lasers;
                while( lasers.empty() ){
                    vlp16 >> lasers;
                }
                lasers >> lines;
            }



            friend std::vector<velodyne::Laser>& operator >> (std::vector<velodyne::Laser>& lasers, Lines& lines) {
                for( const velodyne::Laser& laser : lasers ){
                    lines.add(laser);
                }
            }

            friend boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& operator << (boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& cloud, Lines& lines) {
                pcl::PointXYZ point;

                lines.remove_redundant();
                lines.remove_duplicate();
                cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
                for( const myLasers::Line& line : lines.lines){
                    for( const velodyne::Laser& laser : line.lasers ){
                        const double distance = static_cast<double>( laser.distance );
                        const double azimuth  = laser.azimuth  * M_PI / 180.0;
                        const double vertical = laser.vertical * M_PI / 180.0;
                    
                        point.x = static_cast<float>( ( distance * std::cos( vertical ) ) * std::sin( azimuth ) );
                        point.y = static_cast<float>( ( distance * std::cos( vertical ) ) * std::cos( azimuth ) );
                        point.z = static_cast<float>( ( distance * std::sin( vertical ) ) );
                    
                        cloud->points.push_back(point);
                    }
                }
                cloud->width = static_cast<uint32_t>(cloud->points.size());
                cloud->height = 1;
            }
    };
}

#endif