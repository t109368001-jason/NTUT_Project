#ifndef PCAP_CACHER_H_
#define PCAP_CACHER_H_

#define HAVE_BOOST
#define HAVE_PCAP
#define HAVE_FAST_PCAP
#ifndef MAX_QUEUE_SIZE
#define MAX_QUEUE_SIZE 100
#endif

#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/registration/transforms.h>
#include <VelodyneCapture/VelodyneCapture_cloud.h>
#include <basic_function.hpp>


namespace velodyne {

    class VLP16 {
        typedef pcl::PointXYZ PointT;
    public:
        VLP16Capture<PointT> vlp16;
        std::string pcapFilename;
        uint64_t frameOffset;
        boost::shared_ptr<Eigen::Matrix4f> transformMatrixPtr;
    };

    class PcapCache {
        typedef pcl::PointXYZ PointT;
        typedef pcl::PointCloud<PointT> PointCloudT;
        typedef boost::shared_ptr<PointCloudT> PointCloudPtrT;
    public:
        boost::filesystem::path outputPath;
        std::vector<boost::shared_ptr<VLP16>> files;
        uint64_t beg, end, totalFrame;

        bool converted;

        PcapCache();
        PcapCache(std::string outputPath);

        bool add(const std::string &pcapFilename, const int64_t &frameOffset = 0, const boost::shared_ptr<Eigen::Matrix4f> &transformMatrixPtr = nullptr);

        bool add(const std::string &pcapFilename, const std::string &configFilename);

        bool filesIsRun();

        PointCloudPtrT getCloudFromCapture();

        PointCloudPtrT get(int64_t index);

        void saveConfig();

        std::string getConfigString();

        std::string loadConfigString();

        bool exists();

        void setRange(const uint64_t &beg = 0, const uint64_t &end = std::numeric_limits<uint64_t>::max());

        bool convert();
    };
}
#endif