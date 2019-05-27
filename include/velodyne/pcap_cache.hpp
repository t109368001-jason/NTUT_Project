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
#include <pcl/octree/octreenbuf_base.hpp>
#include <pcl/octree/my_octree_pointcloud.hpp>
#include <pcl/filters/extract_indices.h>

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
            void setBackgroundResolution(double resolution, bool use_nbuffer);

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
            int mode_;
            double back_resolution_;
            bool use_nbuffer_;
            bool converted_;
            bool backChange_;
            PointCloudPtrT back_;

            bool capturesIsRun();

            PointCloudPtrT getCloudFromCapture();

            int getMaxFrameSize();

            void saveConfig(nlohmann::json config_new);

            nlohmann::json getConfigJson();
            nlohmann::json loadConfigJson();

            PointCloudPtrT getBackgroundCloud();

            PointCloudPtrT computeBackground();
            PointCloudPtrT computeBackground2();

    };
}
using namespace velodyne;
PcapCache::PcapCache() : outputPath_("/tmp/pcapCache/"), use_nbuffer_(false), back_resolution_(0), back_(nullptr), mode_(0), beg_(0), end_(std::numeric_limits<int>::max()), converted_(false) { };
PcapCache::PcapCache(std::string outputPath_) : outputPath_(outputPath_ + "/"), use_nbuffer_(false), back_resolution_(0), back_(nullptr), mode_(0), beg_(0), end_(std::numeric_limits<int>::max()), converted_(false) { };

void PcapCache::setBackgroundResolution(double resolution, bool use_nbuffer) {
    back_resolution_ = resolution;
    use_nbuffer_ = use_nbuffer;
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
            cloud = getBackgroundCloud();
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
    config["end"] = std::min(end_, getMaxFrameSize()-2);
    config["back"] = (back_resolution_ > 0 ? "back_" + std::to_string(back_resolution_) + ".pcd" : "");
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

void PcapCache::saveConfig(nlohmann::json config_new) {
    std::ofstream ofs(outputPath_.string() + "config.json");
    ofs << config_new;
    system(("chmod 777 " + outputPath_.string() + " -R").c_str());
}

nlohmann::json PcapCache::loadConfigJson() {
    nlohmann::json config;
    if(!myFunction::fileExists(outputPath_.string() + "config.json"))
    {
        return config;
    }

    std::ifstream ifs(outputPath_.string() + "config.json");
/*
    std::string s;
    getline (ifs, s, (char) ifs.eof());

*/
    ifs >> config;
    return config;
}

void PcapCache::setRange(const int &beg, const int &end) {
    beg_ = beg;
    end_ = std::min(end, getMaxFrameSize()-2);
}

void PcapCache::nextMode() {
    mode_ = back_resolution_ > 0 ? (mode_>=2 ?  0 : mode_ + 1) : 0;
}

void PcapCache::convert() {

    myClass::MicroStopwatch tt_convert("convert");
    tt_convert.tic();
    assert(end_ >= beg_);
    PointCloudPtrT cloud = nullptr;
    nlohmann::json oldConfig = loadConfigJson();
    nlohmann::json config_new;
    config_new["beg"] = 0;
    config_new["end"] = 0;
    config_new["back"] = "";
    for(int i = 0; i < files_.size(); i++) {
        config_new["pcapFilename"+std::to_string(i)] = files_[i]->pcapFilename;
        config_new["frameOffset"+std::to_string(i)] = files_[i]->frameOffset;
        if(files_[i]->transformMatrixPtr) {
            std::stringstream ss;
            ss << *(files_[i]->transformMatrixPtr);
            config_new["transformMatrix"+std::to_string(i)] = ss.str();
        }
    }

    if(!myFunction::fileExists(outputPath_.string())) {
        mkdir(outputPath_.string().c_str(), 0777);
    }
    saveConfig(config_new);
    int beg_pre = (oldConfig["beg"] != NULL ? (oldConfig["beg"].is_number() ? oldConfig["beg"].get<int>() : 0) : 0);
    int end_pre = (oldConfig["end"] != NULL ? (oldConfig["end"].is_number() ? oldConfig["end"].get<int>() : 0) : 0);
    std::string back_string_old = (oldConfig["back"] != NULL ? (oldConfig["back"].is_string() ? oldConfig["back"].get<std::string>() : "") : "");
    
    getCloudFromCapture(); //skip first frame

    if(!((beg_ >= beg_pre)&&(end_ <= end_pre))) {
        for(int index = 0; index < beg_; index++) {
            getCloudFromCapture();
            std::cout << index << "\033[0G" << std::flush;
        }

        if((end_pre < beg_)||(end_ < beg_pre)) {
            for(int index = beg_; index <= end_; index++) {
                if(((cloud = getCloudFromCapture())==nullptr)||(index > end_)) {
                    break;
                }
                pcl::io::savePCDFileBinaryCompressed<PointT>(outputPath_.string() + std::to_string(index) + ".pcd", *cloud);
                std::cout << index << "\033[0G" << std::flush;
            }
        } else {
            for(int index = beg_; index < beg_pre; index++) {
                if(((cloud = getCloudFromCapture())==nullptr)||(index > end_)) {
                    break;
                }
                pcl::io::savePCDFileBinaryCompressed<PointT>(outputPath_.string() + std::to_string(index) + ".pcd", *cloud);
                std::cout << index << "\033[0G" << std::flush;
            }

            for(int index = std::max(beg_pre, beg_); index <= std::min(end_pre, end_); index++) {
                getCloudFromCapture();
                std::cout << index << "\033[0G" << std::flush;
            }

            for(int index = end_pre+1; index <= end_; index++) {
                if((cloud = getCloudFromCapture())==nullptr) {
                    break;
                }
                pcl::io::savePCDFileBinaryCompressed<PointT>(outputPath_.string() + std::to_string(index) + ".pcd", *cloud);
                std::cout << index << "\033[0G" << std::flush;
            }
        }
    }

    config_new["beg"] = beg_;
    config_new["end"] = end_;
    saveConfig(config_new);

    if(back_resolution_ > 0) {
        bool do_all = false;
        if(!myFunction::fileExists(outputPath_.string() + "/noBack/"))
        {
            mkdir((outputPath_.string() + "/noBack/").c_str(), 0777);
        }
        if(back_string_old == ("back_" + std::to_string(back_resolution_) + ".pcd"))
        {
            back_.reset(new PointCloudT);
            pcl::io::loadPCDFile<PointT>(back_string_old, *back_);
        } else {
            myClass::MicroStopwatch tt1("Create back");
            tt1.tic();
            if(use_nbuffer_) {
                back_ = computeBackground();
            } else {
                back_ = computeBackground2();
            }
            pcl::io::savePCDFileBinaryCompressed(outputPath_.string() + "back_" + std::to_string(back_resolution_) + ".pcd", *back_);
            tt1.toc();
            do_all = true;
            saveConfig(config_new);
        }
        std::vector<int> indices;
        if(do_all) {
            for(int index = beg_; index <= end_; index++) {
                indices.push_back(index);
            }
        } else {
            if((end_pre < beg_)||(end_ < beg_pre)) {
                for(int index = beg_; index < end_; index++) {
                    indices.push_back(index);
                }
            }
            else {
                for(int index = beg_; index < beg_pre; index++) {
                    indices.push_back(index);
                }
                for(int index = end_pre+1; index <= end_; index++) {
                    indices.push_back(index);
                }
            }
        }
        myClass::backgroundSegmentation<PointT> bgs;
        bgs.setBackground(back_, back_resolution_);
        for(int index : indices) {
            PointCloudPtrT cloud_in(new PointCloudT);
            PointCloudPtrT cloud_out(new PointCloudT);
            pcl::io::loadPCDFile<PointT>(outputPath_.string() + std::to_string(index) + ".pcd", *cloud_in);
            cloud_out = bgs.compute(cloud_in);
            pcl::io::savePCDFileBinaryCompressed<PointT>(outputPath_.string()+"/noBack/" + std::to_string(index) + ".pcd", *cloud_out);
            std::cout << index << " "<< cloud_in->points.size() << " " << cloud_out->points.size()<< "\033[0G" << std::flush;
        }
        close();
    }
    
    std::cout << std::endl << "Complete! " << frameSize() << " frames" << std::endl;

    config_new["back"] = "";
    saveConfig(config_new);
    converted_ = true;
    tt_convert.toc();
    return;
}

PointCloudPtrT PcapCache::getBackgroundCloud() {
    return back_;
}

PointCloudPtrT PcapCache::computeBackground ()
{
    std::vector<int> indicesVector;
    std::vector<pcl::octree::OctreeContainerPointIndices*> leaf_containers;
    PointCloudPtrT cloud1(new PointCloudT);
    PointCloudPtrT cloud2(new PointCloudT);

    int N = 100;
    
    {
        pcl::octree::MyOctreePointCloud<pcl::PointXYZ,
                                      pcl::octree::OctreeContainerPointIndices,
                                      pcl::octree::OctreeContainerEmpty,
                                      pcl::octree::OctreeNBufBase<pcl::octree::OctreeContainerPointIndices,
                                                                  pcl::octree::OctreeContainerEmpty> > octree(back_resolution_);
        for (int i =0;i<N;i++) {
            PointCloudPtrT cloud(new PointCloudT);
            cloud = this->getCloudById(i*(this->frameSize()/N)+this->beg());
            octree.switchBuffers(i);
            octree.setInputCloud(cloud);
            octree.addPointsFromInputCloud();
            std::cout << i << " " << cloud->points.size() << " " << i*(this->frameSize()/N)+this->beg() << std::endl;
        }
        boost::function<bool ( std::vector<bool> bit_patterns )> function =
            [&N] ( std::vector<bool> bit_patterns){
                int true_count = 0;
                for(auto bit_pattern : bit_patterns) {
                    if(bit_pattern) true_count++;
                }
                return true_count >= (N*0.5);
            };
        std::vector<std::pair<unsigned char, pcl::octree::OctreeContainerPointIndices*>> other_leaf_containers;
        octree.serializeLeafs (leaf_containers, N/2, &function, &other_leaf_containers, true);

        int i = 0;
        for(const auto &other_leaf_container : other_leaf_containers) {
            PointCloudPtrT cloud(new PointCloudT);
            std::vector<int> indicesVector2;
            cloud = cloud = this->getCloudById(other_leaf_container.first*(this->frameSize()/N)+this->beg());
            if (static_cast<int> (other_leaf_container.second->getSize ()) >= 0)
            other_leaf_container.second->getPointIndices(indicesVector2);

            for(auto &idx : indicesVector2) {
                cloud1->points.push_back(cloud->points[idx]);
            }

            cloud1->width = static_cast<uint32_t>( cloud1->points.size() );
            cloud1->height = 1;
        }
        cloud1->width = static_cast<uint32_t>(cloud1->points.size());
        cloud1->height = 1;
        std::cout << cloud1->points.size() << std::endl;
    }
    std::cout << "ABCD" << std::endl;
    {
        pcl::octree::MyOctreePointCloud<pcl::PointXYZ, pcl::octree::OctreeContainerPointIndices, pcl::octree::OctreeContainerEmpty, pcl::octree::OctreeNBufBase<pcl::octree::OctreeContainerPointIndices, pcl::octree::OctreeContainerEmpty> > octree2(10);

        octree2.switchBuffers(0);
        octree2.setInputCloud(cloud1);
        octree2.addPointsFromInputCloud();

        leaf_containers.clear();
        octree2.serializeLeafs (leaf_containers, 0, nullptr, nullptr, false);

        for (const auto &leaf_container : leaf_containers)
        {
            if (static_cast<int> (leaf_container->getSize ()) >= 0)
            indicesVector.push_back(leaf_container->getPointIndex());
        }

        for(auto &idx : indicesVector) {
            cloud2->points.push_back(cloud1->points[idx]);
        }
        cloud2->width = static_cast<uint32_t>( cloud2->points.size() );
        cloud2->height = 1;
        std::cout << cloud2->points.size() << std::endl;
    }

    return cloud2;
}

PointCloudPtrT PcapCache::computeBackground2 ()
{
    std::vector<std::thread *> ts(std::thread::hardware_concurrency() + 1);
    PointCloudPtrT back(new PointCloudT());
    std::vector<PointCloudPtrT> backs(ts.size());

    int N = 100;

    int d = frameSize()/N;
    int jump = N/2;

    boost::function<void(int i, int j)> function = 
    [&backs, &jump, &d, this] ( int i, int j) {
        PointCloudPtrT temp(new PointCloudT);
        PointCloudPtrT temp1(new PointCloudT);
        temp = myFunction::getNoChanges<PointT>(getCloudById(i), getCloudById(i+(jump*d)), back_resolution_);
        temp1 = myFunction::getNoChanges<PointT>(getCloudById(i+(jump*d)), getCloudById(i), back_resolution_);
        temp = myFunction::getChanges<PointT>(backs[j], temp, 10.0);
        *backs[j] += *temp;
        temp1 = myFunction::getChanges<PointT>(backs[j], temp1, 10.0);
        *backs[j] += *temp1;
    };
    
    for(auto &cloud : backs) {
        cloud.reset(new PointCloudT());
    }

    int i = beg_;
    while(i < int(beg_ + frameSize()/2)){
        bool skip = false;

        for(int j = 0; j<ts.size(); j++){
            if(i >= int(beg_ + frameSize()/2)) { skip = true; break; }

            if(ts[j])
            {
                if( ts[j]->joinable() ){
                    ts[j]->join();
                    ts[j]->~thread();
                    delete ts[j];
                    ts[j] = nullptr;

                    ts[j] = new std::thread(std::bind(function, i, j));   
                    std::cout << i << "\033[0G" << std::flush;
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

    for(auto &cloud : backs) {
        if(back->points.size() > 1000000) break;
        cloud = myFunction::getChanges<PointT>(back, cloud, 10.0);
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