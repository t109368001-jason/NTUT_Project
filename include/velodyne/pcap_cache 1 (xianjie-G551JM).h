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
        private:
        public:
            std::string tmpFolderName;
            uint64_t totalFrame;
            boost::filesystem::path filename;

            std::vector<PcapFileConfig> files;

            PcapCache() : tmpFolderName("pcapCache"), totalFrame(0) { };

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

            bool VLP16IsRun(std::vector<boost::shared_ptr<VLP16>> vlp16s) {
                bool isRun = true;
                for(auto vlp16 : vlp16s) {
                    isRun &= vlp16->isRun();
                }
                return isRun;
            }

            boost::shared_ptr<PointCloudT> getCloud(std::vector<boost::shared_ptr<VLP16>> vlp16s) {
                boost::shared_ptr<PointCloudT> cloud;
                if(VLP16IsRun()) {
					cloud.reset(new PointCloudT());
                    for(auto vlp16 : vlp16s) {
                        boost::shared_ptr<PointCloudT> temp;
					    temp.reset(new PointCloudT());
						(*vlp16) >> temp;
                        //pcl::transformPointCloud(*temp1, *temp2, vlp16s_transformMatrix[i]);
                        *cloud += *temp1;
                    }
                }
                return cloud;
            }

            bool convert() {
                std::vector<boost::shared_ptr<VLP16>> vlp16s;

                PointCloudPtrT cloud;

                for(auto &file : this->files) {
                    boost::shared_ptr<VLP16> vlp16;
                    if(!vlp16->open(file.pcapFilename))
                    {
                        std::cout << std::endl << "Error : load " << file.pcapFilename << " failed" << std::endl;
                        return false;
                    }

                    vlp16->moveToNext(file.frameOffset);
                    vlp16->setTransformMatrix(file.transformMatrix);
                    vlp16s.push_back(vlp16);
                }
                int i = 0;
                while(VLP16IsRun(vlp16s)) {
                    cloud = getCloud(vlp16s);
                    pcl::io::savePCDFileBinaryCompressed("/tmp/" + std::to_string(i) + ".pcd");
                }

            }
    };
}
#endif