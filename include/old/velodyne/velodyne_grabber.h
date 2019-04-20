#ifndef VELODYNE_GRABBER_H_
#define VELODYNE_GRABBER_H_

#include "velodyne.h"
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl/registration/transforms.h>

namespace velodyne {

    class VLP16Grabber {
        typedef pcl::PointXYZ PointT;
        typedef pcl::PointCloud<PointT> PointCloudT;
        typedef boost::shared_ptr<PointCloudT> PointCloudPtrT;

        public:
        
            int64_t frameNumber;
            bool _pause;
            std::vector<boost::shared_ptr<VLP16>> vlp16s;
            std::vector<int> vlp16s_offsetFrameNum;
            std::vector<Eigen::Matrix4f> vlp16s_transformMatrix;
            boost::function<void (const boost::shared_ptr<PointCloudT>& )> callback;

            boost::shared_ptr<std::thread> thread;

            bool isRun_;

            VLP16Grabber() : frameNumber(0), isRun_(false) { 
                _pause = false;
            };

            bool add(const std::string& filename) {
                boost::shared_ptr<VLP16> vlp16;
                if(!vlp16->open(filename))
                {
                    return false;
                }
                this->vlp16s.push_back(vlp16);
            }

            bool add(const boost::filesystem::path& filename) {
                this->add(filename.string());
            }

            bool add(boost::shared_ptr<VLP16> &vlp16, const int &offsetFrameNum, const Eigen::Matrix4f &transformMatrix = Eigen::Matrix4f::Identity()) {
                for(int i = 0; i < offsetFrameNum; i++)
                    vlp16->moveToNext();
                this->vlp16s.push_back(vlp16);
                this->vlp16s_offsetFrameNum.push_back(offsetFrameNum);
                transformMatrix.transpose();
                this->vlp16s_transformMatrix.push_back(transformMatrix);
            }

            void registerCallback (const boost::function<void (const boost::shared_ptr<PointCloudT>& )> & callback) {
                this->callback = callback;
            }

            void restart() {
                this->close();
                this->start();
            }

            void close() {
                for(boost::shared_ptr<VLP16> &vlp16 : this->vlp16s) {
                    vlp16->close();
                }

                if( thread && thread->joinable() ){
                    thread->join();
                    thread->~thread();
                    thread.reset();
                    thread = nullptr;
                }

            }

            bool isRun() {
                return this->isRun_;
            }

            bool VLP16IsRun() {
                bool isRun = true;
                for(auto it : this->vlp16s) {
                    isRun &= it->isRun();
                }
                return isRun;
            }

            boost::shared_ptr<PointCloudT> getCloud() {
                boost::shared_ptr<PointCloudT> cloud;
                if(VLP16IsRun()) {
					cloud.reset(new PointCloudT());
                    for(int i = 0; i < vlp16s.size(); i++) {
                        boost::shared_ptr<PointCloudT> temp1;
                        boost::shared_ptr<PointCloudT> temp2;
					    temp1.reset(new PointCloudT());
					    temp2.reset(new PointCloudT());
						(*vlp16s[i]) >> temp1;
                        //pcl::transformPointCloud(*temp1, *temp2, vlp16s_transformMatrix[i]);
                        *cloud += *temp1;
                    }
                }
                return cloud;
            }

            void nextFrame() {
                for(int i = 0; i < vlp16s.size(); i++) {
                    vlp16s[i]->moveToNext();
                }
            }

            void nextFrame(const int& n) {
                for(int j = 0; j < n; j++) {
                    this->nextFrame();
                }
            }

            void run() {
                if(this->vlp16s.size() == 0) {
                    isRun_ = false;
                    return;
                }
				boost::shared_ptr<PointCloudT> cloud;
                while(VLP16IsRun()) {

                    if(this->_pause) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        continue;
                    }
                    
                    cloud = this->getCloud();

					this->callback(cloud);
                    this->nextFrame();
                    frameNumber = frameNumber + 1;
                }
                isRun_ = false;
            }

            int toFolder(const std::string& tmpPath) {
                if(this->vlp16s.size() == 0) return 1;
				boost::shared_ptr<PointCloudT> cloud;
                while(VLP16IsRun()) {
					cloud.reset(new PointCloudT());

                    cloud = this->getCloud();

                    boost::filesystem::path p{tmpPath + "/" + std::to_string(frameNumber) + ".pcd"};
                    pcl::io::savePCDFileBinaryCompressed(p.string(), *cloud);
					//this->callback(cloud);

                    this->nextFrame();
                    frameNumber++;
                }
                return 0;
            }

            void start() {
                isRun_ = true;
                thread.reset( new std::thread( std::bind( &VLP16Grabber::run, this ) ));
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            void pause(const bool pause) {
                this->_pause = pause;
            }

    };
}

#endif