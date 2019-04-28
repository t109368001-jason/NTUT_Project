#ifndef PCAP_CACHER_H_
#define PCAP_CACHER_H_

#define HAVE_PCAP
#define HAVE_FAST_PCAP
#ifndef MAX_QUEUE_SIZE
#define MAX_QUEUE_SIZE 100
#endif

#include <pcl/io/pcd_io.h>
#include <pcl/registration/transforms.h>
#include <VelodyneCapture/VelodyneCapture_cloud.h>
#include <basic_function.hpp>
#include <backgroundSegmentation.hpp>
#include <nlohmann/json.hpp>

namespace velodyne {
    
    class VLP16 {
        typedef pcl::PointXYZ PointT;
        public:
            VLP16Capture<PointT> vlp16;
            std::string pcapFilename;
            int frameOffset;
            boost::shared_ptr<Eigen::Matrix4f> transformMatrixPtr;
    };


    class PcapCache {
        typedef pcl::PointXYZ PointT;
        typedef pcl::PointCloud<PointT> PointCloudT;
        typedef boost::shared_ptr<PointCloudT> PointCloudPtrT;

        public:
            PcapCache();
            PcapCache(std::string outputPath);
            
            void addPcap(const std::string &pcapFilename, const int &frameOffset = 0, const boost::shared_ptr<Eigen::Matrix4f> &transformMatrixPtr = nullptr);
            void addPcap(const std::string &pcapFilename, const std::string &configFilename);
            void close();
            void convert();

            void setRange(const int &beg = 0, const int &end = std::numeric_limits<int>::max());
            void setBackgroundParameters(int backNumber, int compareFrameNumber,  double resolution);

            PointCloudPtrT getCloudById(int index);

            void nextMode();

            int beg();
            int end();
            int frameSize();
            int currentFrameId();

            ~PcapCache();

        private:
            boost::filesystem::path outputPath_;
            std::vector<boost::shared_ptr<VLP16>> files_;
            int beg_, end_, currentFrameId_;
            int backNumber_;
            int compareFrameNumber_;
            int mode_;
            double resolution_;
            bool converted_;
            bool backChange_;

            bool capturesIsRun();

            PointCloudPtrT getCloudFromCapture();

            int getMaxFrameSize();

            void saveConfig();

            nlohmann::json getConfigJson();
            nlohmann::json loadConfigJson();

            bool exists();

            PointCloudPtrT getBackgroundCloud();

            PointCloudPtrT computeBackground();

    };
}
using namespace velodyne;
PcapCache::PcapCache() : outputPath_("/tmp/pcapCache/"), mode_(0), backChange_(false), backNumber_(0), beg_(0), end_(std::numeric_limits<int>::max()), converted_(false) { };
PcapCache::PcapCache(std::string outputPath_) : outputPath_(outputPath_ + "/"), mode_(0), backChange_(false), backNumber_(0), beg_(0), end_(std::numeric_limits<int>::max()), converted_(false) { };

void PcapCache::setBackgroundParameters(int backNumber, int compareFrameNumber,  double resolution) {
    backNumber_ = backNumber;
    compareFrameNumber_ = compareFrameNumber;
    resolution_ = resolution;
}

inline int PcapCache::beg() {
    return beg_;
}

inline int PcapCache::end() {
    return end_;
}

inline int PcapCache::currentFrameId() {
    return currentFrameId_;
}

inline int PcapCache::frameSize() {
    return end_ - beg_ + 1;
}

void PcapCache::addPcap(const std::string &pcapFilename, const int &frameOffset, const boost::shared_ptr<Eigen::Matrix4f> &transformMatrixPtr) {
    boost::shared_ptr<VLP16> file;
    file.reset(new VLP16);

    file->pcapFilename = pcapFilename;
    file->frameOffset = frameOffset;

    if(transformMatrixPtr) {
        if(!file->vlp16.open(file->pcapFilename, *transformMatrixPtr))
        {
            throw std::runtime_error("Error : load " + file->pcapFilename + " failed");
        }
        file->transformMatrixPtr = transformMatrixPtr;
    } else {
        if(!file->vlp16.open(file->pcapFilename))
        {
            throw std::runtime_error("Error : load " + file->pcapFilename + " failed");
        }
    }
    if(!file->vlp16.isOpen())
    {
        throw std::runtime_error("Error : load " + file->pcapFilename + " failed");
    }
    std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );

    PointCloudPtrT cloud;
    for(int i = 0; i < file->frameOffset; i++) {
        cloud.reset(new PointCloudT);
        file->vlp16.retrieve_block(cloud);
    }
    
    files_.push_back(file);
}

void PcapCache::addPcap(const std::string &pcapFilename, const std::string &configFilename) {
    boost::filesystem::path configPath{configFilename};
    if(configFilename == "default") {
        configPath = pcapFilename;
        configPath.replace_extension("txt");
    }

    std::ifstream ifs(configPath.string());
    int frameOffset;
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
    addPcap(pcapFilename, frameOffset, transformMatrixPtr);
}

bool PcapCache::capturesIsRun() {
    bool isRun = true;
    for(auto &file : files_) {
        isRun &= file->vlp16.isRun();
    }
    return isRun;
}

PointCloudPtrT PcapCache::getCloudFromCapture() {
    PointCloudPtrT cloud(new PointCloudT());
    std::cout << "\033[0G";
    for(auto &file : files_) {
        PointCloudPtrT temp = nullptr;
        file->vlp16.retrieve_block(temp);
        if(temp) {
            *cloud += *temp;

        } else {
            close();
            return nullptr;
        }
        std:: cout << std::setw(3) << std::left << file->vlp16.getQueueSize() << '\t';
    }
    return cloud;
}

PointCloudPtrT PcapCache::getCloudById(int index) {
    PointCloudPtrT cloud(new PointCloudT);
    currentFrameId_ = index;
    switch(mode_){
        case 0:
            pcl::io::loadPCDFile<PointT>(outputPath_.string() + std::to_string(currentFrameId_) + ".pcd", *cloud);
            break;
        
        case 1:
            pcl::io::loadPCDFile<PointT>(outputPath_.string() + "/noBack/" + std::to_string(currentFrameId_) + ".pcd", *cloud);
            break;

        case 2:
            pcl::io::loadPCDFile<PointT>(outputPath_.string() + "/noBack/" + std::to_string(currentFrameId_) + ".pcd", *cloud);
            *cloud+=*getBackgroundCloud();
            break;
    }
    return cloud;
}

int PcapCache::getMaxFrameSize() {
    int frameSize = std::numeric_limits<int>::max();
    for(auto &file : files_) {
        frameSize = std::min(frameSize, getFrameSize(file->pcapFilename)-file->frameOffset);
    }
    return frameSize;
}

nlohmann::json PcapCache::getConfigJson() {
    nlohmann::json config;
    config["beg"] = beg_;
    config["end"] = std::min(end_, getMaxFrameSize()-1);
    config["back"] = (backNumber_ > 0 ? "back_" + std::to_string(backNumber_) + ".pcd" : "");
    for(int i = 0; i < files_.size(); i++) {
        config["pcapFilename"+std::to_string(i)] = files_[i]->pcapFilename;
        config["frameOffset"+std::to_string(i)] = files_[i]->frameOffset;
        if(files_[i]->transformMatrixPtr) {
            std::stringstream ss;
            ss << *(files_[i]->transformMatrixPtr);
            config["transformMatrix"+std::to_string(i)] = ss.str();
        }
    }
    return config;
}

void PcapCache::saveConfig() {
    std::ofstream ofs(outputPath_.string() + "config.json");
    ofs << getConfigJson();
}

nlohmann::json PcapCache::loadConfigJson() {
    if(!myFunction::fileExists(outputPath_.string() + "config.json"))
    {
        return "";
    }

    std::ifstream ifs(outputPath_.string() + "config.json");
/*
    std::string s;
    getline (ifs, s, (char) ifs.eof());

*/
    nlohmann::json config;
    ifs >> config;
    return config;
}

bool PcapCache::exists() {
    nlohmann::json oldConfig = loadConfigJson();
    nlohmann::json currentConfig = getConfigJson();

    for(auto& [key, value] : oldConfig.items()) {
        if(value != currentConfig[key]) {
            if(key == "back") {
                backChange_ = true;
            } else {
                std::cout << key << " : " << value << " != " << currentConfig[key] << std::endl;
                return false;
            }
        }
    }

    beg_ = oldConfig["beg"].get<int>();
    end_ = oldConfig["end"].get<int>();

    return true;
}

void PcapCache::setRange(const int &beg, const int &end) {
    beg_ = beg;
    end_ = end;
}

void PcapCache::nextMode() {
    mode_ = backNumber_ == 0 ? 0 : (mode_>=2 ?  0 : mode_ + 1);
}

void PcapCache::convert() {
    if(!myFunction::fileExists(outputPath_.string()))
    {
        mkdir(outputPath_.string().c_str(), 0777);
    }

    if(exists()) return;
    
    int index = beg_;
    for(int i = 0; i < (beg_+1); i++) {
        getCloudFromCapture();
    }

    boost::function<void( const std::string &file_name, PointCloudPtrT &cloud )> function =
        [] ( const std::string &file_name, PointCloudPtrT &cloud){
            pcl::io::savePCDFileBinaryCompressed<PointT>(file_name, *cloud);
        };
    
    std::vector<std::thread*> ts(std::thread::hardware_concurrency()+1);

    PointCloudPtrT cloud = nullptr;
    while(capturesIsRun()&&(index <= end_)) {
        
        for(auto &thread : ts) {
            if(((cloud = getCloudFromCapture())==nullptr)||(index > end_)) {
                break;
            }
            if(thread)
            {
                if( thread->joinable() ){
                    thread->join();
                    thread->~thread();
                    delete thread;
                    thread = nullptr;
                    thread = new std::thread(std::bind(function, outputPath_.string() + std::to_string(index) + ".pcd", cloud));
                }
            } else {
                thread = new std::thread(std::bind(function, outputPath_.string() + std::to_string(index) + ".pcd", cloud));
            }
            std::cout << index++ << "\033[0G" << std::flush;
        }
    }
    end_ = std::min(end_, index-1);
    
    std::cout << std::endl << "Complete! " << frameSize() << " frames" << std::endl;
    for(auto &thread : ts) {
        if(thread) {
            while( !thread->joinable() );
            thread->join();
            thread->~thread();
            delete thread;
            thread = nullptr;
        }
    }

    if(backChange_ && (backNumber_ > 0))
    {
        PointCloudPtrT back = nullptr;
        if(!myFunction::fileExists(outputPath_.string() + "/noBack/"))
        {
            mkdir((outputPath_.string() + "/noBack/").c_str(), 0777);
        }

        back = computeBackground();
        pcl::io::savePCDFileBinaryCompressed<PointT>(outputPath_.string() + "/back_" + std::to_string(backNumber_) + ".pcd", *back);
        std::vector<std::thread*> ts(std::thread::hardware_concurrency()+1);

        boost::function<void(const std::string file_name, const int begin, const int ended)> function =
        [this, &back] (const std::string file_name, const int begin, const int ended){
                myClass::backgroundSegmentation<PointT> b;                            
                b.setBackground(back, 20.0);

            for(int k = begin; k <= ended; k++){
                if(k >= frameSize())break;

                PointCloudPtrT temp(new PointCloudT);
                pcl::io::loadPCDFile<PointT>( file_name + std::to_string(k) + ".pcd", *temp);

                temp = b.compute(temp);
                pcl::io::savePCDFileBinaryCompressed<PointT>(file_name + "/noBack/" + std::to_string(k) + ".pcd", *temp);
                std::cout << k << "\033[0G" << std::flush;
            }
        };

        for(int j = 0; j<ts.size(); j++){
            int begin = j*frameSize()/ts.size();
            int ended = begin+frameSize()/ts.size();

            ts[j] = new std::thread(std::bind(function, outputPath_.string(), begin, ended));
        }

        for(int j = 0; j < ts.size(); j++) {
            if(ts[j]) {
                while( !ts[j]->joinable() );
                ts[j]->join();
                ts[j]->~thread();
                delete ts[j];
                ts[j] = nullptr;
            }
        }
    }

    saveConfig();
    converted_ = true;
    system(("chmod 777 " + outputPath_.string() + " -R").c_str());
    return;
}

PointCloudPtrT PcapCache::getBackgroundCloud() {
    PointCloudPtrT back(new PointCloudT);
    pcl::io::loadPCDFile<PointT>(outputPath_.string() + "/back_" + std::to_string(backNumber_) + ".pcd", *back);
    return back;
}

PointCloudPtrT PcapCache::computeBackground ()
{
    std::vector<std::thread *> ts(std::thread::hardware_concurrency() + 1);
    PointCloudPtrT back(new PointCloudT());
    std::vector<PointCloudPtrT> backs(ts.size());

    int d = frameSize()/compareFrameNumber_/2*2;
    int jump = compareFrameNumber_/2;

    boost::function<void(int i, int j)> function = 
    [&backs, &jump, &d, this] ( int i, int j) {
        PointCloudPtrT temp(new PointCloudT);
        PointCloudPtrT temp1(new PointCloudT);
        temp = myFunction::getNoChanges<PointT>(getCloudById(i), getCloudById(i+(jump*d)), resolution_);
        temp1 = myFunction::getNoChanges<PointT>(getCloudById(i+(jump*d)), getCloudById(i), resolution_);
        temp = myFunction::getChanges<PointT>(backs[j], temp, 10.0);
        temp1 = myFunction::getChanges<PointT>(backs[j], temp1, 10.0);
        *backs[j] += *temp;
        *backs[j] += *temp1;
    };
    
    for(auto &cloud : backs) {
        cloud.reset(new PointCloudT());
    }

    int i = 0;
    while(i < int(frameSize()/2)){
        bool skip = false;

        for(int j = 0; j<ts.size(); j++){
            if(i >= int(frameSize()/2)) { skip = true; break; }

            if(ts[j])
            {
                if( ts[j]->joinable() ){
                    ts[j]->join();
                    ts[j]->~thread();
                    delete ts[j];
                    ts[j] = nullptr;

                    ts[j] = new std::thread(std::bind(function, i, j));     
                    i += d;
                }
            } 
            else{
                ts[j] = new std::thread(std::bind(function, i, j));       
                i += d;
            }
        }
        if(skip) break;
    }

    for(int j = 0; j < ts.size(); j++) {
        if(ts[j]) {
            while( !ts[j]->joinable() );
            ts[j]->join();
            ts[j]->~thread();
            delete ts[j];
            ts[j] = nullptr;
        }
    }

    for(auto cloud : backs) {
        if(back->points.size() > backNumber_) break;
        cloud = myFunction::getChanges<PointT>(back, cloud, 1.0);
        *back += *cloud;
        std::cout << " back size= " << back->points.size() << std::endl;
    }

    return back;
}

void PcapCache::close() {
    for(auto &file : files_) {
        file->vlp16.close();
    }
}
PcapCache::~PcapCache() {
    close();
}
#endif