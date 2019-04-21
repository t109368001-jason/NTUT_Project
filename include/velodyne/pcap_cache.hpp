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
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef boost::shared_ptr<PointCloudT> PointCloudPtrT;

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

using namespace velodyne;
PcapCache::PcapCache() : outputPath("/tmp/pcapCache/"), beg(0), end(std::numeric_limits<uint64_t>::max()), totalFrame(0), converted(false) { };
PcapCache::PcapCache(std::string outputPath) : outputPath(outputPath + "/"), beg(0), end(std::numeric_limits<uint64_t>::max()), totalFrame(0), converted(false) { };

bool PcapCache::add(const std::string &pcapFilename, const int64_t &frameOffset, const boost::shared_ptr<Eigen::Matrix4f> &transformMatrixPtr) {
    boost::shared_ptr<VLP16> file;
    file.reset(new VLP16);

    file->pcapFilename = pcapFilename;
    file->frameOffset = frameOffset;

    if(transformMatrixPtr) {
        if(!file->vlp16.open(file->pcapFilename, *transformMatrixPtr))
        {
            std::cout << std::endl << "Error : load " << file->pcapFilename << " failed 1" << std::endl;
            return false;
        }
        file->transformMatrixPtr = transformMatrixPtr;
    } else {
        if(!file->vlp16.open(file->pcapFilename))
        {
            std::cout << std::endl << "Error : load " << file->pcapFilename << " failed 2" << std::endl;
            return false;
        }
    }
    if(!file->vlp16.isOpen())
    {
        std::cout << std::endl << "Error : load " << file->pcapFilename << " failed 3" << std::endl;
        return false;
    }
    std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );

    PointCloudPtrT cloud;
    for(int64_t i = 0; i < file->frameOffset; i++) {
        cloud.reset(new PointCloudT);
        file->vlp16.retrieve_block(cloud);
    }
    
    this->files.push_back(file);
    
    return true;
}

bool PcapCache::add(const std::string &pcapFilename, const std::string &configFilename) {
    boost::filesystem::path configPath{configFilename};
    if(configFilename == "default") {
        configPath = pcapFilename;
        configPath.replace_extension("txt");
    }

    std::ifstream ifs(configPath.string());
    int64_t frameOffset;
    boost::shared_ptr<Eigen::Matrix4f> transformMatrixPtr;
    transformMatrixPtr.reset(new Eigen::Matrix4f);

    if(ifs.is_open()){
        std::string s;
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 4; j++) {
                ifs >> s;
                (*transformMatrixPtr)(i,j) = std::stof(s);
            }
        }
        ifs >> s;
        frameOffset = std::stoll(s);
    }
    return this->add(pcapFilename, frameOffset, transformMatrixPtr);
}

bool PcapCache::filesIsRun() {
    bool isRun = true;
    for(auto &file : this->files) {
        isRun &= file->vlp16.isRun();
    }
    return isRun;
}

PointCloudPtrT PcapCache::getCloudFromCapture() {
    PointCloudPtrT cloud;
    if(filesIsRun()) {
        cloud.reset(new PointCloudT());
        std::cout << "\033[1G";
        for(auto &file : this->files) {
            PointCloudPtrT temp;
            temp.reset(new PointCloudT());
            file->vlp16.retrieve_block(temp);
            *cloud += *temp;
            std:: cout << std::setw(3) << file->vlp16.getQueueSize() << '\t';
        }
    }
    return cloud;
}

PointCloudPtrT PcapCache::get(int64_t index) {
    PointCloudPtrT cloud(new PointCloudT);
    pcl::io::loadPCDFile<PointT>(outputPath.string() + std::to_string(index) + ".pcd", *cloud);
    return cloud;
}

void PcapCache::saveConfig() {
    std::ofstream ofs(outputPath.string() + "config.inf");

    ofs << "totalFrame="<< this->totalFrame << std::endl;
    ofs << "beg="<< this->beg << std::endl;
    ofs << "end="<< this->end << std::endl;
    for(auto file : files) {
        ofs << "pcapFilename=" << file->pcapFilename << std::endl;
        ofs << "frameOffset=" << file->frameOffset << std::endl;
        std::stringstream ss;
        if(file->transformMatrixPtr) {
            ss << (*file->transformMatrixPtr);
        }
        ofs << "transformMatrix=" << ss.str() << std::endl;
    }
}

std::string PcapCache::getConfigString() {
    std::stringstream ss;

    ss << "totalFrame="<< this->totalFrame << std::endl;
    ss << "beg="<< this->beg << std::endl;
    ss << "end="<< this->end << std::endl;
    for(auto file : files) {
        ss << "pcapFilename=" << file->pcapFilename << std::endl;
        ss << "frameOffset=" << file->frameOffset << std::endl;
        std::stringstream sss;
        if(file->transformMatrixPtr) {
            sss << (*file->transformMatrixPtr);
        }
        ss << "transformMatrix=" << sss.str() << std::endl;
    }
    return ss.str();
}

std::string PcapCache::loadConfigString() {
    if(!myFunction::fileExists(outputPath.string() + "config.inf"))
    {
        return "";
    }

    std::ifstream ifs(outputPath.string() + "config.inf");

    std::string s;
    getline (ifs, s, (char) ifs.eof());
    return s;
}

bool PcapCache::exists() {
    std::string s1 = loadConfigString();
    std::string s2 = getConfigString();
    std::vector<std::string> sv1, sv2;

    boost::split(sv1, s1, boost::is_any_of("\n"));
    boost::split(sv2, s2, boost::is_any_of("\n"));
    int i = 1;
    while(i < sv1.size() || i < sv2.size()) {
        if(i < sv1.size() && i < sv2.size()) {
            if(sv1[i] != sv2[i]) {
                return false;
            }
        } else {
            if(i < sv1.size()) {
                if(sv1[i] != "") {
                    return false;
                }
            }
            if(i < sv2.size()) {
                if(sv2[i] != "") {
                    return false;
                }
            }
        }
        i++;
    }
    boost::split(sv1, sv1[0], boost::is_any_of("="));
    totalFrame = std::stoull(sv1[1]);
    return true;
}

void PcapCache::setRange(const uint64_t &beg, const uint64_t &end) {
    this->beg = beg;
    this->end = end;
}

bool PcapCache::convert() {
    if(!myFunction::fileExists(outputPath.string()))
    {
        mkdir(outputPath.string().c_str(), 0777);
    }

    if(exists()) {
        converted = true;
        return true;
    }

    for(int64_t i = 0; i < (beg+1); i++) {
        getCloudFromCapture();
    }
    totalFrame += beg;

    boost::function<void( const std::string &file_name, const PointCloudT &cloud )> function =
        [ ] ( const std::string &file_name, const PointCloudT &cloud){
            pcl::io::savePCDFileBinaryCompressed<PointT>(file_name, cloud);
        };
        
    std::vector<std::thread*> ts(std::thread::hardware_concurrency()+1);
    while(filesIsRun()&&(totalFrame < end)) {
        
        for(auto &thread : ts) {
            
            if((!filesIsRun())||(totalFrame >= end)) {
                break;
            }
            PointCloudPtrT cloud;
            while((cloud ? cloud->points.size() == 0 : true)) {
                cloud = getCloudFromCapture();
            }
            if(thread)
            {
                if( thread->joinable() ){
                    thread->join();
                    thread->~thread();
                    delete thread;
                    thread = nullptr;
                    thread = new std::thread(std::bind(function, outputPath.string() + std::to_string(totalFrame) + ".pcd", *cloud));
                }
            } else {
                thread = new std::thread(std::bind(function, outputPath.string() + std::to_string(totalFrame) + ".pcd", *cloud));
            }
            std::cout << totalFrame << "\033[5G" << std::flush;
            totalFrame++;
        }
    }
    std::cout << std::endl << "Complete! total: " << totalFrame << " frame" << std::endl;
    for(auto &thread : ts) {
        if(thread) {
            while( !thread->joinable() );
            thread->join();
            thread->~thread();
            delete thread;
            thread = nullptr;
        }
    }
    saveConfig();
    converted = true;
    return true;
}

#endif