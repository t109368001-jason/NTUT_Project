#ifndef VELODYNE_H_
#define VELODYNE_H_

#define HAVE_BOOST
#define HAVE_PCAP
#define HAVE_FAST_PCAP
//#define VELODYNE_CAPTURE_USE_CLOUD
#define MAX_QUEUE_SIZE 100

#include <vector>
#include <pcl/point_cloud.h>
#include <Eigen/src/Core/Matrix.h>
#include <basic_function.h>
#if defined(MAX_QUEUE_SIZE) && defined(VELODYNE_CAPTURE_USE_CLOUD)
#include <VelodyneCapture/VelodyneCapture_cloud.h>
#elif defined(MAX_QUEUE_SIZE)
#include <VelodyneCapture/VelodyneCapture_modified.h>
#else
#include <VelodyneCapture/VelodyneCapture.h>
#endif

namespace velodyne {

    template<typename PointT>
    class VLP16 : public VLP16Captureboost::shared_ptr<pcl::PointCloud<PointT>> {
        typedef pcl::PointCloud<PointT> PointCloudT;
        typedef boost::shared_ptr<PointCloudT> PointCloudPtrT;

        public:
            bool convertToCluodInOtherThread;
            int64_t frameNumber;
            boost::shared_ptr<std::thread> toCloudThreadPtr;
            boost::shared_ptr<std::string> filenamePtr;
            boost::shared_ptr<std::vector<Laser>> lasersPtr;
            
            boost::shared_ptr<Eigen::Matrix4f> transformMatrixPtr;

            boost::shared_ptr<std::vector<PointCloudPtrT>> clouds;


            VLP16() : VLP16Capture(), frameNumber(-1), convertToCluodInOtherThread(false) {};

            const bool open( const std::string &filename, const bool &convertToCluodInOtherThread = false ) {
                this->convertToCluodInOtherThread = convertToCluodInOtherThread;
                this->filenamePtr.reset(&filename);
                bool result = VelodyneCapture::open(filename);
                result &= VLP16Capture::isOpen();
                if(result) {
                    this->nextFrame();
                    this->frameNumber = 0;
                }
                if(convertToCluodInOtherThread) {
                    
                }
                return result;
            }

            void nextFrame() {
                if( this->mutex.try_lock() ){
                    if( !this->queue.empty() ){
                        std::vector<Laser> l = std::move(this->queue.front());
                        this->lasersPtr.reset(&l);
                        this->queue.pop();
                    }
                    this->mutex.unlock();
                }
                while(this->isRun())
                {
                    if(this->queue.size() > 0) break;
                }
                this->frameNumber++;
            }

            void nextFrame(const int n) {
                for(int i = 0; i < n; i++) {
                    this->nextFrame();
                }
            }

            void loadTransformMatrix(std::string filename = "") {

                boost::filesystem::path matrixFilename{(filename == "" ? this->filename : filename)};
                std::ifstream ifs(matrixFilename.parent_path().string() + "/" + matrixFilename.stem().string() + "_transform_matrix.txt");

                Eigen::Matrix4f m;
                //ifs >> m;
                ifs >> m(0,0) >> m(0,1) >> m(0,2) >> m(0,3);
                ifs >> m(1,0) >> m(1,1) >> m(1,2) >> m(1,3);
                ifs >> m(2,0) >> m(2,1) >> m(2,2) >> m(2,3);
                ifs >> m(3,0) >> m(3,1) >> m(3,2) >> m(3,3);
                this->setTransformMatrix(m);
            }
            Eigen::Matrix4f getTransformMatrix() {
                return *this->transformMatrixPtr;
            }

            void saveTransformMatrix(std::string filename = "") {
                boost::filesystem::path matrixFilename{(filename == "" ? this->filename : filename)};
                std::ofstream ofs(matrixFilename.parent_path().string() + "/" + matrixFilename.stem().string() + "_transform_matrix.txt");

                ofs << *this->transformMatrixPtr;
            }

            void setTransformMatrix(Eigen::Matrix4f &transformMatrix) {
                *this->transformMatrixPtr = transformMatrix;
            }

            void operator >> (std::vector<Laser>& lasers) {
                lasers = *this->lasersPtr;
            }
            template<typename RandomIt>
            std::vector<PointCloudPtrT> getCloudsPart(uint64_t division_num, RandomIt beg, RandomIt end) {

                
                auto len = end - beg;

                if(len < division_num)
                {
                    std::vector<PointCloudPtrT> out;
                    for(auto it = beg; it != end; ++it)
                    {
                        PointCloudPtrT cloud(new PointCloudT);
                        cloud = lasersToCloud(*it);
                        out.push_back(cloud);
                    }
                    return out;
                }
                auto mid = beg + len/2;
                auto handle = std::async(std::launch::async, getCloudsPart<RandomIt>, division_num, beg, mid);
                auto out1 = getCloudsPart<RandomIt>(division_num, mid, end);
                auto out = handle.get();

                std::copy(out1.begin(), out1.end(), std::back_inserter(out));

                return out + out1;
            }

            static PointCloudPtrT lasersToCloud(const std::vector<Laser> &lasers, const Eigen::Matrix4f &transformMatrix = Eigen::Matrix4f::Identity()) {
                PointCloudPtrT cloud;
                cloud.reset(new PointCloudT());

                for( const velodyne::Laser laser : lasers ){
                    Eigen::Vector4f p1;
                    const double distance = static_cast<double>( laser.distance );
                    const double azimuth  = laser.azimuth  * M_PI / 180.0;
                    const double vertical = laser.vertical * M_PI / 180.0;
                
                    p1(0) = static_cast<float>( ( distance * std::cos( vertical ) ) * std::sin( azimuth ) );
                    p1(1) = static_cast<float>( ( distance * std::cos( vertical ) ) * std::cos( azimuth ) );
                    p1(2) = static_cast<float>( ( distance * std::sin( vertical ) ) );
                    p1(3) = 1;
                
                    if( (p1(0) == 0.0f) && (p1(1) == 0.0f) && (p1(2) == 0.0f) ){
                        continue;
                    }

                    p1 = transformMatrix*p1;
                    if( (p1(0) == std::numeric_limits<float>::quiet_NaN()) || (p1(1) == std::numeric_limits<float>::quiet_NaN()) || (p1(2) == std::numeric_limits<float>::quiet_NaN()) ){
                        continue;
                    }
                    if( (p1(0) == std::numeric_limits<float>::infinity()) || (p1(1) == std::numeric_limits<float>::infinity()) || (p1(2) == std::numeric_limits<float>::infinity()) ){
                        continue;
                    }

                    cloud->points.push_back(PointT{p1(0), p1(1), p1(2)});
                }
                cloud->width = static_cast<uint32_t>(cloud->points.size());
                cloud->height = 1;
            }

            void operator >> (PointCloudPtrT &cloud) {
                //PointT point;

                cloud.reset(new PointCloudT());

                for( const velodyne::Laser laser : *this->lasersPtr ){
                    Eigen::Vector4f p1, p2;
                    const double distance = static_cast<double>( laser.distance );
                    const double azimuth  = laser.azimuth  * M_PI / 180.0;
                    const double vertical = laser.vertical * M_PI / 180.0;
                
                    p1(0) = static_cast<float>( ( distance * std::cos( vertical ) ) * std::sin( azimuth ) );
                    p1(1) = static_cast<float>( ( distance * std::cos( vertical ) ) * std::cos( azimuth ) );
                    p1(2) = static_cast<float>( ( distance * std::sin( vertical ) ) );
                    p1(3) = 1;
                
                    if( (p1(0) == 0.0f) && (p1(1) == 0.0f) && (p1(2) == 0.0f) ){
                        continue;
                    }

                    if(this->transformMatrixPtr) {
                        p2 = (*this->transformMatrixPtr)*p1;
                        p1 = p2;
                    }
                    if( (p1(0) == std::numeric_limits<float>::quiet_NaN()) || (p1(1) == std::numeric_limits<float>::quiet_NaN()) || (p1(2) == std::numeric_limits<float>::quiet_NaN()) ){
                        continue;
                    }
                    if( (p1(0) == std::numeric_limits<float>::infinity()) || (p1(1) == std::numeric_limits<float>::infinity()) || (p1(2) == std::numeric_limits<float>::infinity()) ){
                        continue;
                    }

                    cloud->points.push_back(PointT{p1(0), p1(1), p1(2)});
                }
                cloud->width = static_cast<uint32_t>(cloud->points.size());
                cloud->height = 1;
            }
            
    };
}

#endif