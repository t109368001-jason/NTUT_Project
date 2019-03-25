#ifndef PCAP_CACHER_H_
#define PCAP_CACHER_H_


#include <eigen3/Eigen/src/Core/Matrix.h>
#include "velodyne_grabber.h"

namespace velodyne {


    class PcapFileConfig {
        public:
            std::string pcapFilename;
            uint64_t frameOffset;
            Eigen::Matrix4f transformMatrix;
    };

    template<typename PointT>
    class PcapCache {
            typedef pcl::PointCloud<PointT> PointCloudT;
            typedef boost::shared_ptr<PointCloudT> PointCloudPtrT;
        public:
            std::string tmpFolderName;
            uint64_t totalFrame;
            std::vector<PcapFileConfig> files;

            bool converted;

            PcapCache() : tmpFolderName("pcapCache"), totalFrame(0), converted(false) { };
            PcapCache(std::string tmpFolderName) : tmpFolderName(tmpFolderName), totalFrame(0), converted(false) { };

            void add(std::string pcapFilename) {
                PcapFileConfig file;
                file.pcapFilename = pcapFilename;
                file.frameOffset = 0;
                file.transformMatrix = Eigen::Matrix4f::Identity();
                this->files.push_back(file);
            }

            void add(std::string pcapFilename, uint64_t frameOffset) {
                PcapFileConfig file;
                file.pcapFilename = pcapFilename;
                file.frameOffset = frameOffset;
                file.transformMatrix = Eigen::Matrix4f::Identity();
                this->files.push_back(file);
            }

            void add(std::string pcapFilename, uint64_t frameOffset, Eigen::Matrix4f transformMatrix) {
                PcapFileConfig file;
                file.pcapFilename = pcapFilename;
                file.frameOffset = frameOffset;
                file.transformMatrix = transformMatrix;
                this->files.push_back(file);
            }

            bool VLP16IsRun(std::vector<boost::shared_ptr<VLP16>> &vlp16s) {
                bool isRun = true;
                for(auto vlp16 : vlp16s) {
                    isRun &= vlp16->isRun();
                }
                return isRun;
            }

            boost::shared_ptr<PointCloudT> getCloud(std::vector<boost::shared_ptr<VLP16>> &vlp16s) {
                boost::shared_ptr<PointCloudT> cloud;
                if(VLP16IsRun(vlp16s)) {
					cloud.reset(new PointCloudT());
                    for(auto vlp16 : vlp16s) {
                        boost::shared_ptr<PointCloudT> temp;
					    temp.reset(new PointCloudT());
						(*vlp16) >> temp;
                        *cloud += *temp;
                    }
                }
                return cloud;
            }

            void nextFrame(std::vector<boost::shared_ptr<VLP16>> &vlp16s) {
                for(auto vlp16 : vlp16s) {
                    vlp16->moveToNext();
                }
            }

            void nextFrame(std::vector<boost::shared_ptr<VLP16>> &vlp16s, uint64_t n) {
                for(uint64_t i = 0; i < n; i++) {
                    for(auto vlp16 : vlp16s) {
                        vlp16->moveToNext(n);
                    }
                }
            }

            void saveConfig(boost::filesystem::path outputDir) {
                std::ofstream ofs(outputDir.string() + "config.inf");

                ofs << "tmpFolderName="<< this->tmpFolderName << std::endl;
                ofs << "totalFrame="<< this->totalFrame << std::endl;
                ofs << "converted="<< this->converted << std::endl;
                for(auto file : files) {
                    ofs << "pcapFilename=" << file.pcapFilename << std::endl;
                    ofs << "frameOffset=" << file.frameOffset << std::endl;
                    ofs << "transformMatrix=" << file.transformMatrix << std::endl;
                }
            }
            
            bool convert(const uint64_t &beg = 0, const uint64_t &end = std::numeric_limits<uint64_t>::max()) {
                std::vector<boost::shared_ptr<VLP16>> vlp16s;

                PointCloudPtrT cloud;

                boost::filesystem::path outputDir{"/tmp/" + this->tmpFolderName + "/"};

                this->saveConfig(outputDir);

                for(auto &file : this->files) {
                    boost::shared_ptr<VLP16> vlp16(new velodyne::VLP16);
                    
                    if(!vlp16->open(file.pcapFilename))
                    {
                        std::cout << std::endl << "Error : load " << file.pcapFilename << " failed" << std::endl;
                        return false;
                    }

                    vlp16->moveToNext(file.frameOffset);
                    vlp16->setTransformMatrix(file.transformMatrix);
                    vlp16s.push_back(vlp16);
                }
                uint64_t i = 0;
                this->nextFrame(vlp16s, beg+1);
                i += beg;

                if(!myFunction::fileExists(outputDir.string()))
                {
                    mkdir(outputDir.string().c_str(), 0777);
                }

                while(VLP16IsRun(vlp16s)||(i > end)) {
                    cloud = getCloud(vlp16s);
                    pcl::io::savePCDFileBinaryCompressed(outputDir.string() + std::to_string(i) + ".pcd", *cloud);
                    this->nextFrame(vlp16s);
                    i++;
                }
                totalFrame = i;
                converted = true;
            }

            bool isAvailable() {
                return this->converted;
            }
    };
}
#endif