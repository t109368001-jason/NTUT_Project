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
#include <backgroundSegmentation.hpp>

namespace velodyne {
    typedef pcl::PointXYZ PointT;
    
    class VLP16 {
        public:
            VLP16Capture<PointT> vlp16;
            std::string pcapFilename;
            uint64_t frameOffset;
            boost::shared_ptr<Eigen::Matrix4f> transformMatrixPtr;
    };


    class PcapCache {
        typedef pcl::PointCloud<PointT> PointCloudT;
        typedef boost::shared_ptr<PointCloudT> PointCloudPtrT;

        public:
            boost::filesystem::path outputPath;
            std::vector<boost::shared_ptr<VLP16>> files;
            uint64_t beg, end, totalFrame;
            PointCloudPtrT back;
            int backNumber;
            int compareFrameNumber;
            int mode;
            double resolution;
            bool converted;
            bool backChange;
            int currentFrameId;

            PcapCache();
            PcapCache(std::string outputPath);
            
            void addBack(int backNumber, int compareFrameNumber,  double resolution);

            bool add(const std::string &pcapFilename, const int64_t &frameOffset = 0, const boost::shared_ptr<Eigen::Matrix4f> &transformMatrixPtr = nullptr);

            bool add(const std::string &pcapFilename, const std::string &configFilename);

            bool filesIsRun();

            PointCloudPtrT getCloudFromCapture();

            PointCloudPtrT get(int index);

            void saveConfig();

            std::string getConfigString();

            std::string loadConfigString();

            bool exists();

            void setRange(const uint64_t &beg = 0, const uint64_t &end = std::numeric_limits<uint64_t>::max());

            bool convert();

            boost::shared_ptr<pcl::PointCloud<PointT>> getBack();

            void getBackground ();

            void nextMode();

            ~PcapCache();
    };
}
using namespace velodyne;
PcapCache::PcapCache() : outputPath("/tmp/pcapCache/"), mode(0), backChange(false), currentFrameId(0), backNumber(0), beg(0), end(std::numeric_limits<uint64_t>::max()), totalFrame(0), converted(false) { };
PcapCache::PcapCache(std::string outputPath) : outputPath(outputPath + "/"), mode(0), backChange(false), currentFrameId(0), backNumber(0), beg(0), end(std::numeric_limits<uint64_t>::max()), totalFrame(0), converted(false) { };

void PcapCache::addBack(int backNumber, int compareFrameNumber,  double resolution) {
    this->backNumber = backNumber;
    this->compareFrameNumber = compareFrameNumber;
    this->resolution = resolution;
}

bool PcapCache::add(const std::string &pcapFilename, const int64_t &frameOffset, const boost::shared_ptr<Eigen::Matrix4f> &transformMatrixPtr) {
    boost::shared_ptr<VLP16> file;
    file.reset(new VLP16);

    file->pcapFilename = pcapFilename;
    file->frameOffset = frameOffset;

    if(transformMatrixPtr) {
        if(!file->vlp16.open(file->pcapFilename, *transformMatrixPtr))
        {
            std::cout << std::endl << "Error : load " << file->pcapFilename << " failed" << std::endl;
            return false;
        }
        file->transformMatrixPtr = transformMatrixPtr;
    } else {
        if(!file->vlp16.open(file->pcapFilename))
        {
            std::cout << std::endl << "Error : load " << file->pcapFilename << " failed" << std::endl;
            return false;
        }
    }
    if(!file->vlp16.isOpen())
    {
        std::cout << std::endl << "Error : load " << file->pcapFilename << " failed" << std::endl;
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

PointCloudPtrT PcapCache::get(int index) {
    PointCloudPtrT cloud(new PointCloudT);
    
    switch(this->mode){
        case 0:
            pcl::io::loadPCDFile<PointT>(outputPath.string() + std::to_string(index) + ".pcd", *cloud);
            break;
        
        case 1:
            pcl::io::loadPCDFile<PointT>(outputPath.string() + "/noBack/" + std::to_string(index) + ".pcd", *cloud);
            break;

        case 2:
            pcl::io::loadPCDFile<PointT>(outputPath.string() + "/noBack/" + std::to_string(index) + ".pcd", *cloud);
            *cloud+=*getBack();
            break;
    }
    return cloud;
}

void PcapCache::saveConfig() {
    std::ofstream ofs(outputPath.string() + "config.inf");

    ofs << "totalFrame="<< this->totalFrame << std::endl;
    ofs << "beg="<< this->beg << std::endl;
    ofs << "end="<< this->end << std::endl;
    ofs << "back=" << (back ? "back_" + std::to_string(this->backNumber) + ".pcd" : "") << std::endl;
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
    ss << "back=" << (this->backNumber != 0 ? "back_" + std::to_string(this->backNumber) + ".pcd" : "") << std::endl;

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

    /*std::stringstream ss;

    while(!ifs.eof()) {
        std::string s;
        ifs >> s;
        ss << s;
        ss << std::endl;
    }*/
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
        if(i == 3) {
            i++;
            continue;
        }
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
    
    this->backChange = (sv1[3] != sv2[3]) ? true : false;

    boost::split(sv1, sv1[0], boost::is_any_of("="));
    totalFrame = std::stoull(sv1[1]);
    return true;
}

void PcapCache::setRange(const uint64_t &beg, const uint64_t &end) {
    this->beg = beg;
    this->end = end;
}

void PcapCache::nextMode() {
    mode = (mode>=2) ?  0 : (mode + 1);
}

bool PcapCache::convert() {
    if(!myFunction::fileExists(outputPath.string()))
    {
        mkdir(outputPath.string().c_str(), 0777);
    }

    if(!exists()) {
        for(int64_t i = 0; i < (beg+1); i++) {
            getCloudFromCapture();
        }
        totalFrame += beg;

        boost::function<void( const std::string &file_name, PointCloudPtrT &cloud )> function =
            [] ( const std::string &file_name, PointCloudPtrT &cloud){
                pcl::io::savePCDFileBinaryCompressed<PointT>(file_name, *cloud);
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
                        thread = new std::thread(std::bind(function, outputPath.string() + std::to_string(totalFrame) + ".pcd", cloud));
                    }
                } else {
                    thread = new std::thread(std::bind(function, outputPath.string() + std::to_string(totalFrame) + ".pcd", cloud));
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
    }

    if((!exists())||(this->backChange)){
        if(this->backNumber > 0)
        {
            if(!myFunction::fileExists(outputPath.string() + "/noBack/"))
            {
                mkdir((outputPath.string() + "/noBack/").c_str(), 0777);
            }

            getBackground();
            pcl::io::savePCDFileBinaryCompressed<PointT>(outputPath.string() + "/back" + std::to_string(this->backNumber) + ".pcd", *back);
            std::vector<std::thread*> ts(std::thread::hardware_concurrency()+1);

            boost::function<void(const std::string file_name, const int begin, const int ended)> function =
            [this] (const std::string file_name, const int begin, const int ended){
                    myClass::backgroundSegmentation<PointT> b;                            
                    b.setBackground(this->back, 20.0);

                for(int k = begin; k <= ended; k++){
                    if(k >= this->totalFrame)break;

                    boost::shared_ptr<pcl::PointCloud<PointT>> temp(new pcl::PointCloud<PointT>);
                    pcl::io::loadPCDFile<PointT>( file_name + std::to_string(k) + ".pcd", *temp);

                    temp = b.compute(temp);
                    pcl::io::savePCDFileBinaryCompressed<PointT>(file_name + "/noBack/" + std::to_string(k) + ".pcd", *temp);
                    std::cout << k << "\033[5G" << std::flush;
                }
            };

            for(int j = 0; j<ts.size(); j++){
                int begin = j*this->totalFrame/ts.size();
                int ended = begin+this->totalFrame/ts.size();

                ts[j] = new std::thread(std::bind(function, outputPath.string(), begin, ended));
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
    } else {
        return true;
    }

    saveConfig();
    converted = true;
    system(("chmod 777 " + outputPath.string() + " -R").c_str());
    return true;
}

boost::shared_ptr<pcl::PointCloud<PointT>> PcapCache::getBack() {
    if(!back)back.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile<PointT>(outputPath.string() + "/back" + std::to_string(this->backNumber) + ".pcd", *back);
    return back;
}

void PcapCache::getBackground ()
{
    std::vector<std::thread *> ts(std::thread::hardware_concurrency() + 1);
    boost::shared_ptr<pcl::PointCloud<PointT>> back(new pcl::PointCloud<PointT>());
    std::vector<boost::shared_ptr<pcl::PointCloud<PointT>>> backs(ts.size());

    int d = this->totalFrame/this->compareFrameNumber/2*2;
    int jump = this->compareFrameNumber/2;

    boost::function<void(int i, int j)> function = 
    [&backs, &jump, &d, this] ( int i, int j) {
        boost::shared_ptr<pcl::PointCloud<PointT>> temp(new pcl::PointCloud<PointT>());
        boost::shared_ptr<pcl::PointCloud<PointT>> temp1(new pcl::PointCloud<PointT>());
        temp = myFunction::getNoChanges<PointT>(this->get(i), this->get(i+(jump*d)), this->resolution);
        temp1 = myFunction::getNoChanges<PointT>(this->get(i+(jump*d)), this->get(i), this->resolution);
        temp = myFunction::getChanges<PointT>(backs[j], temp, 1.0);
        temp1 = myFunction::getChanges<PointT>(backs[j], temp1, 1.0);
        *backs[j] += *temp;
        *backs[j] += *temp1;
    };
    
    for(auto &cloud : backs) {
        cloud.reset(new pcl::PointCloud<PointT>());
    }

    int i = 0;
    while(i < int(this->totalFrame/2)){
        bool skip = false;

        for(int j = 0; j<ts.size(); j++){
            if(i >= int(this->totalFrame/2)) { skip = true; break; }

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
        if(back->points.size() > this->backNumber) break;
        cloud = myFunction::getChanges<PointT>(back, cloud, 1.0);
        *back += *cloud;
        std::cout << " back size= " << back->points.size() << std::endl;
    }

    this->back = back;
}

PcapCache::~PcapCache() {
    for(auto &file : files) {
        file->vlp16.close();
    }
}
#endif