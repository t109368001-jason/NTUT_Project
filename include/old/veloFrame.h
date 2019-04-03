#ifndef VELOFRAME_H_
#define VELOFRAME_H_

#ifndef TIMEZONE
    #define TIMEZONE 0
#endif

#include <iostream>
#include <chrono>
#include <typeinfo>
#include <future>
#include <boost/shared_ptr.hpp>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>
#include "../3rdparty/VelodyneCapture/VelodyneCapture.h"
#include "../include/basic_function.h"
#include "../include/microStopwatch.h"
#include "../include/backgroundSegmentation.h"

namespace VeloFrame
{
    class VeloFrameException : public std::exception
    {
        private:
            std::string exceptionMessage;

        public:
            VeloFrameException() : exceptionMessage ("No message.") {}
            explicit VeloFrameException(std::string message) : exceptionMessage (std::move(message)) {}
            const char *what() const throw()
            {
                std::stringstream s;
                s << "VeloFrameException : " << this->exceptionMessage << std::endl;
                std::string wharString = s.str();
                return wharString.c_str();
            }
            std::string message() const
            {
                std::stringstream s;
                s << "\033[0;31m";      //red
                s << "VeloFrameException: ";
                s << "\033[0m";         //default color
                s << this->exceptionMessage;
                s << std::endl;
                return s.str();
            }
    };

    class VeloFrame
    {
        public:
            std::chrono::microseconds minTimestamp;
            std::chrono::microseconds midTimestamp;
            std::chrono::microseconds maxTimestamp;
            boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud;

            VeloFrame()
            {
                this->minTimestamp = std::chrono::microseconds(0);
                this->midTimestamp = std::chrono::microseconds(0);
                this->maxTimestamp = std::chrono::microseconds(0);
                this->cloud = NULL;
            }
            VeloFrame(const long long &minTimestamp, const long long &midTimestamp, const long long &maxTimestamp, const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud)
            {
                this->minTimestamp = std::chrono::microseconds(minTimestamp);
                this->midTimestamp = std::chrono::microseconds(midTimestamp);
                this->maxTimestamp = std::chrono::microseconds(maxTimestamp);
                this->cloud = cloud;
            }

            void setMinTimestamp(const long long &minTimestamp)
            {
                this->minTimestamp = std::chrono::microseconds(minTimestamp);
            }
            
            void setMidTimestamp(const long long &midTimestamp)
            {
                this->midTimestamp = std::chrono::microseconds(midTimestamp);
            }
            
            void setMaxTimestamp(const long long &maxTimestamp)
            {
                this->maxTimestamp = std::chrono::microseconds(maxTimestamp);
            }

            void setCloud(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud)
            {
                this->cloud = cloud;
            }

            void print(const std::string &prefixString = "")
            {
                std::cout << prefixString << "minTimestamp : " << this->minTimestamp.count() << " (" << myFunction::durationToString(this->minTimestamp, false) << ")" << std::endl;
                std::cout << prefixString << "midTimestamp : " << this->midTimestamp.count() << " (" << myFunction::durationToString(this->midTimestamp, false) << ")" << std::endl;
                std::cout << prefixString << "maxTimestamp : " << this->maxTimestamp.count() << " (" << myFunction::durationToString(this->maxTimestamp, false) << ")" << std::endl;
                std::cout << prefixString << "cloud point size : " << this->cloud->points.size() << std::endl;
            }
    };

    class VeloFrames
    {
        friend class VeloFrameViewer;
        protected:
            boost::filesystem::path pcapFilePath;
            boost::filesystem::path outputPath;
            boost::filesystem::path outputPathWithParameter;
            std::string parameterString;
            std::vector<boost::shared_ptr<VeloFrame>> frames;
            boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> backgroundCloud;
            bool isChanged;
            bool isLoaded;
            bool isSaved;
            bool isBackgroundSegmented;
            bool isNoiseRemoved;
            bool isNoiseRemovedOverOnce;
            bool isOffset;
            bool isBackgroundSegmentationResolutionSet;
            bool isNoiseRemovalParameterSet;
            bool isOffsetPointSet;
            bool loadSecondCloudAsBackground;
            bool useZip;
            double backgroundSegmentationResolution;
            double noiseRemovalPercentP;
            double noiseRemovalStddevMulThresh;
            std::vector<double> noiseRemovalParameter;
            std::chrono::microseconds begTime;
            std::chrono::microseconds endTime;
            pcl::PointXYZ offsetPoint;

        public:

            VeloFrames()
            {
                this->pcapFilePath = "";
                this->outputPath = "";
                this->parameterString = "";
                this->isChanged = false;
                this->isLoaded = false;
                this->isSaved = false;
                this->isBackgroundSegmented = false;
                this->isNoiseRemoved = false;
                this->isNoiseRemovedOverOnce = false;
                this->isOffset = false;
                this->isBackgroundSegmentationResolutionSet = false;
                this->isNoiseRemovalParameterSet = false;
                this->isOffsetPointSet = false;
                this->loadSecondCloudAsBackground = false;
                this->backgroundSegmentationResolution = 0.0;
                this->begTime = std::chrono::microseconds(int64_t(0));
                this->endTime = std::chrono::microseconds(std::numeric_limits<int64_t>::max());
                this->offsetPoint.x = std::numeric_limits<float>::quiet_NaN();
                this->offsetPoint.y = std::numeric_limits<float>::quiet_NaN();
                this->offsetPoint.z = std::numeric_limits<float>::quiet_NaN();
            }

            bool setPcapFile(const boost::filesystem::path &pcapFilePath)
            {
                if(!boost::filesystem::exists(pcapFilePath))
                {
                    this->throwException("setPcapFile", boost::filesystem::absolute(pcapFilePath).string() + " not found.");
                    return false;
                }
                this->pcapFilePath = boost::filesystem::canonical(pcapFilePath);
                return true;
            }

            bool setBackgroundSegmentationResolution(const double &backgroundSegmentationResolutionCM)
            {
                if(backgroundSegmentationResolutionCM <= 0.0)
                {
                    std::stringstream ss;
                    ss << "VeloFrame::VeloFrames::setBackgroundSegmentationResolution, ";
                    ss << "background segmentation resolution should > 0";
                    throw VeloFrameException(ss.str());
                    return false;
                }
                this->backgroundSegmentationResolution = backgroundSegmentationResolutionCM;
                this->isBackgroundSegmented = false;
                this->isBackgroundSegmentationResolutionSet = true;
                return true;
            }

            bool setNoiseRemovalParameter(const std::vector<double> noiseRemovalParameter)
            {
                if(noiseRemovalParameter.size() == 3)
                {
                    if((noiseRemovalParameter[0] <= 0.0)&&(noiseRemovalParameter[0] > 1.0))
                    {
                        std::stringstream ss;
                        ss << "VeloFrame::VeloFrames::setNoiseRemovalPercentP, ";
                        ss << "noise removal percentP should > 0 and <= 1.0";
                        throw VeloFrameException(ss.str());
                        return false;
                    }
                    if((noiseRemovalParameter[1] <= 0.0)&&(noiseRemovalParameter[1] > 1.0))
                    {
                        std::stringstream ss;
                        ss << "VeloFrame::VeloFrames::setNoiseRemovalStddevMulThresh, ";
                        ss << "noise removal stddevMulThresh should > 0 and <= 1.0";
                        throw VeloFrameException(ss.str());
                        return false;
                    }
                    if(noiseRemovalParameter[2] <= 0.0)
                    {
                        std::stringstream ss;
                        ss << "VeloFrame::VeloFrames::setNoiseRemovalStddevMulThresh, ";
                        ss << "noise removal times should > 0";
                        throw VeloFrameException(ss.str());
                        return false;
                    }
                }
                else if(((noiseRemovalParameter.size() % 2) == 0)&&(noiseRemovalParameter.size()!= 0))
                {
                    for(int i = 0; i < (noiseRemovalParameter.size()-1); i+=2)
                    {
                        if((noiseRemovalParameter[i] <= 0.0)&&(noiseRemovalParameter[i] > 1.0))
                        {
                            std::stringstream ss;
                            ss << "VeloFrame::VeloFrames::setNoiseRemovalPercentP, ";
                            ss << "noise removal percentP should > 0 and <= 1.0";
                            throw VeloFrameException(ss.str());
                            return false;
                        }
                        if((noiseRemovalParameter[i+1] <= 0.0)&&(noiseRemovalParameter[i+1] > 1.0))
                        {
                            std::stringstream ss;
                            ss << "VeloFrame::VeloFrames::setNoiseRemovalStddevMulThresh, ";
                            ss << "noise removal stddevMulThresh should > 0 and <= 1.0";
                            throw VeloFrameException(ss.str());
                            return false;
                        }
                    }
                }
                else
                {
                    std::stringstream ss;
                    ss << "VeloFrame::VeloFrames::setNoiseRemovalPercentP, ";
                    ss << "noise removal parameter number should be 3 or 2*N";
                    throw VeloFrameException(ss.str());
                    return false;
                }
                this->noiseRemovalParameter = noiseRemovalParameter;
                this->isNoiseRemoved = false;
                this->isNoiseRemovalParameterSet = true;
                return true;
            }

            bool setBackgroundCloud(const boost::filesystem::path &backgroundPath)
            {
                if(backgroundPath.string() == "default")
                {
                    this->loadSecondCloudAsBackground = true;
                    return true;
                }
                if(!boost::filesystem::exists(backgroundPath))
                {
                    std::stringstream ss;
                    ss << "VeloFrame::VeloFrames::setBackgroundCloud, ";
                    ss << boost::filesystem::absolute(backgroundPath).string();
                    ss << " not found.";
                    throw VeloFrameException(ss.str());
                    return false;
                }
                this->backgroundCloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::io::loadPCDFile(backgroundPath.string(), *(this->backgroundCloud));
                return true;
            }

            bool setBegTime(const std::chrono::microseconds &begTime)
            {
                if(begTime < std::chrono::microseconds(int64_t(0)))
                {
                    std::stringstream ss;
                    ss << "VeloFrame::VeloFrames::setBegTime, ";
                    ss << "begin time can't < 0";
                    throw VeloFrameException(ss.str());
                    return false;
                }
                this->begTime = begTime;
                return true;
            }

            bool setEndTime(const std::chrono::microseconds &endTime)
            {
                if(endTime <= this->begTime)
                {
                    std::stringstream ss;
                    ss << "VeloFrame::VeloFrames::setBegTime, ";
                    ss << "end time should > begin time";
                    throw VeloFrameException(ss.str());
                    return false;
                }
                this->endTime = endTime;
                return true;
            }

            bool setOffsetPoint(const float &x, const float &y, const float &z)
            {
                if((x == std::numeric_limits<float>::quiet_NaN())||(y == std::numeric_limits<float>::quiet_NaN())||(z == std::numeric_limits<float>::quiet_NaN()))
                {
                    std::stringstream ss;
                    ss << "VeloFrame::VeloFrames::setOffsetPoint, ";
                    ss << "offset point is NaN";
                    throw VeloFrameException(ss.str());
                    return false;
                }
                if((x == std::numeric_limits<float>::infinity())||(y == std::numeric_limits<float>::infinity())||(z == std::numeric_limits<float>::quiet_NaN()))
                {
                    std::stringstream ss;
                    ss << "VeloFrame::VeloFrames::setOffsetPoint, ";
                    ss << "offset point is infinity";
                    throw VeloFrameException(ss.str());
                    return false;
                }
                this->offsetPoint.x = x;
                this->offsetPoint.y = y;
                this->offsetPoint.z = z;
                this->isOffset = false;
                this->isOffsetPointSet = true;
                return true;
            }

            void setUseZip(bool useZip)
            {
                this->useZip = useZip;
            }

            std::string getOnlyBackgroundSegmentationParameterString(bool isLoad)
            {
                std::stringstream ps;
                if((this->isBackgroundSegmented)||(isLoad))
                {
                    ps << "bs_" << this->backgroundSegmentationResolution;
                }

                return ps.str();
            }

            std::string getOnlyNoiseRemovalParameterString(bool isLoad)
            {
                std::stringstream ps;
                if((this->isNoiseRemoved)||(isLoad))
                {
                    if(noiseRemovalParameter.size() == 3)
                    {
                        for(int i = 0; i < noiseRemovalParameter[2]; ++i)
                        {
                            ps << '/';
                            ps << "nr_" << noiseRemovalParameter[0] << "_" << noiseRemovalParameter[1];
                        }
                    }
                    else if(((noiseRemovalParameter.size() % 2) == 0)&&(noiseRemovalParameter.size()!= 0))
                    {
                        for(int i = 0; i < (noiseRemovalParameter.size()-1); i+=2)
                        {
                            ps << '/';
                            ps << "nr_" << noiseRemovalParameter[i] << "_" << noiseRemovalParameter[i+1];
                        }
                    }
                }
                return ps.str();
            }

            std::string getParameterString(bool isLoad)
            {
                std::stringstream ps;
                if((this->isBackgroundSegmented)||(isLoad))
                {
                    ps << this->getOnlyBackgroundSegmentationParameterString(isLoad);
                }
                ps << '/';
                if((this->isNoiseRemoved)||(isLoad))
                {
                    ps << this->getOnlyNoiseRemovalParameterString(isLoad);
                }
                return ps.str();
            }

            bool load(const std::string &prefixPath)
            {
                bool result;

                if(this->frames.size() != 0)
                {
                    std::cout << "File was loded" << std::endl;
                    return false;
                }

                boost::filesystem::path backgroundSegmentationPath;
                boost::filesystem::path backgroundSegmentationAndNoiseRemovalPath;

                this->outputPath = prefixPath;
                this->outputPath.append(this->pcapFilePath.stem().string());

                backgroundSegmentationPath = this->outputPath;
                backgroundSegmentationPath.append(this->getOnlyBackgroundSegmentationParameterString(true));
                backgroundSegmentationAndNoiseRemovalPath = this->outputPath;
                backgroundSegmentationAndNoiseRemovalPath.append(this->getParameterString(true));

                bool fromPcap = true;
                if((boost::filesystem::exists(backgroundSegmentationAndNoiseRemovalPath))&&(this->isBackgroundSegmentationResolutionSet)&&(this->isNoiseRemovalParameterSet))
                {
                    this->outputPathWithParameter = backgroundSegmentationAndNoiseRemovalPath;
                    this->isBackgroundSegmented = true;
                    this->isNoiseRemoved = true;
                    fromPcap = false;
                }
                else if((boost::filesystem::exists(backgroundSegmentationPath))&&(this->isBackgroundSegmentationResolutionSet))
                {
                    this->outputPathWithParameter = backgroundSegmentationPath;
                    this->isBackgroundSegmented = true;
                    fromPcap = false;
                }
                else if(boost::filesystem::exists(this->outputPath))
                {
                    this->outputPathWithParameter = this->outputPath;
                    fromPcap = false;
                }

                if(!fromPcap)
                {
                    this->outputPathWithParameter = boost::filesystem::canonical(this->outputPathWithParameter);
                    std::cout << "VeloFrame load from " << this->outputPathWithParameter.string() << std::endl;
                    result = this->loadFromFolder();
                }
                else
                {
                    std::cout << "VeloFrame load from " << this->pcapFilePath.string() << std::endl;
                    result = this->loadFromPcap();
                }

                if(this->frames.size() == 0)
                {
                    std::cout << "VeloFrame was loaded, but frame number is zero" << std::endl;
                    return false;
                }

                if(this->loadSecondCloudAsBackground)
                {
                    if(this->frames.size() >= 2)
                    {
                        this->backgroundCloud = this->frames[1]->cloud;
                        std::cout << "Use second second cloud as background" << std::endl;
                    }
                }

                this->isLoaded = result;
                return result;
            }

            bool save(const std::string &prefixPath)
            {
                if(!this->isChanged) 
                {
                    std::cout << std::endl << "No changes to save" << std::endl;
                    return false;
                }
                if(this->isSaved)
                {
                    std::cout << std::endl << "No changes to save" << std::endl;
                    return false;
                }

                this->outputPath = prefixPath;
                this->outputPath.append(this->pcapFilePath.stem().string());
                this->outputPathWithParameter = this->outputPath;
                this->outputPathWithParameter.append(this->getParameterString(false));

                if(!boost::filesystem::exists(this->outputPath))
                {
                    boost::filesystem::create_directories(this->outputPath);
                }
                this->outputPath = boost::filesystem::canonical(this->outputPath);

                if(!boost::filesystem::exists(this->outputPathWithParameter))
                {
                    boost::filesystem::create_directories(this->outputPathWithParameter);
                }
                this->outputPathWithParameter = boost::filesystem::canonical(this->outputPathWithParameter);

                boost::filesystem::path configPath = this->outputPathWithParameter;
                configPath.append("config.txt");

                std::ofstream ofs(configPath.string());

                ofs << "pcap=" << this->pcapFilePath.string() << std::endl;
                if(this->backgroundCloud != NULL)
                {
                    boost::filesystem::path backgroundPath = this->outputPath;
                    backgroundPath.append("background.pcd");
                    pcl::io::savePCDFileBinaryCompressed(backgroundPath.string(), *(this->backgroundCloud));
                    ofs << "background=" << boost::filesystem::relative(this->outputPath, backgroundPath).string() << std::endl;
                }

                #ifdef VELOFRAME_USE_MULTITHREAD
                for(int i = 0; i < this->frames.size(); i++)
                {
                    boost::filesystem::path cloudPath = this->outputPathWithParameter;
                    cloudPath.append(myFunction::durationToString(this->frames[i]->minTimestamp) + ".pcd");
                    ofs << "cloud_" << i << "=" 
                        << this->frames[i]->minTimestamp.count() << "&"
                        << this->frames[i]->midTimestamp.count() << "&"
                        << this->frames[i]->maxTimestamp.count() << "&"
                        << boost::filesystem::relative(this->outputPath, cloudPath).string() << std::endl;
                }

                size_t divisionNumber = myFunction::getDivNum(this->frames.size());

                bool result = this->savePart(divisionNumber, this->frames.begin(), this->frames.end());
                #else
                for(int i = 0; i < this->frames.size(); i++)
                {
                    boost::filesystem::path cloudPath = this->outputPath;
                    cloudPath.append(myFunction::durationToString(this->frames[i]->minTimestamp) + ".pcd");
                    pcl::io::savePCDFileBinaryCompressed(cloudPath.string(), *(this->frames[i]->cloud));
                    ofs << "cloud_" << i << "=" 
                        << this->frames[i]->minTimestamp.count() << "&"
                        << this->frames[i]->midTimestamp.count() << "&"
                        << this->frames[i]->maxTimestamp.count() << "&"
                        << boost::filesystem::relative(this->outputPath, cloudPath).string() << std::endl;
                }
                #endif
                
                std::cout << "VeloFrame save to " << this->outputPathWithParameter.string() << std::endl;

                this->isSaved = true;
                return true;
            }

            bool backgroundSegmentation()
            {
                if(this->isBackgroundSegmented)
                {
                    std::cout << "VeloFrame already Background segmented" << std::endl;
                    return false;
                }
                if(this->backgroundCloud == NULL)
                {
                    std::stringstream ss;
                    ss << "VeloFrame::VeloFrames::backgroundSegmentation, ";
                    ss << "background cloud not set";
                    throw VeloFrameException(ss.str());
                    return false;
                }
                if(this->frames.size() == 0)
                {
                    std::stringstream ss;
                    ss << "VeloFrame::VeloFrames::backgroundSegmentation, ";
                    ss << "veloFrame is empty";
                    throw VeloFrameException(ss.str());
                    return false;
                }
                if(this->backgroundSegmentationResolution <= 0)
                {
                    std::stringstream ss;
                    ss << "VeloFrame::VeloFrames::backgroundSegmentation, ";
                    ss << "background segmentation resolution not set";
                    throw VeloFrameException(ss.str());
                    return false;
                }

                #ifdef VELOFRAME_USE_MULTITHREAD
                myClass::backgroundSegmentation<pcl::PointXYZI> backgroundSegmentation;
                backgroundSegmentation.setBackground(this->backgroundCloud, this->backgroundSegmentationResolution);

                size_t divisionNumber = myFunction::getDivNum(this->frames.size());

                bool result = this->backgroundSegmentationPart(divisionNumber, backgroundSegmentation, this->frames.begin(), this->frames.end());
                #else
                myClass::backgroundSegmentation<pcl::PointXYZI> backgroundSegmentation;
                backgroundSegmentation.setBackground(this->backgroundCloud, this->backgroundSegmentationResolution);
                for(auto it : this->frames)
                {
                    it->cloud = backgroundSegmentation.compute(it->cloud);
                }
                #endif

                for(int i = 0; i < this->frames.size(); ++i)
                {
                    if(this->frames[i]->cloud->points.size() == 0)
                    {
                        this->frames.erase(this->frames.begin() + i);
                        --i;
                        continue;
                    }
                }

                this->isBackgroundSegmented = true;
                this->isChanged = true;
                this->isSaved = false;
            }

            bool noiseRemoval(const bool first = true)
            {
                bool result;
                if((this->isNoiseRemoved)&&(first))
                {
                    std::cout << "VeloFrame already noise removed" << std::endl;
                    return false;
                }
                if(this->frames.size() == 0)
                {
                    std::stringstream ss;
                    ss << "VeloFrame::VeloFrames::noiseRemoval, ";
                    ss << "veloFrame is empty";
                    throw VeloFrameException(ss.str());
                    return false;
                }
                if(this->noiseRemovalParameter.size() == 3)
                {
                    this->noiseRemovalPercentP = this->noiseRemovalParameter[0];
                    this->noiseRemovalStddevMulThresh = this->noiseRemovalParameter[1];
                    for(int i = 0; i < this->noiseRemovalParameter[2]; ++i)
                    {
                        #ifdef VELOFRAME_USE_MULTITHREAD
                        size_t divisionNumber = myFunction::getDivNum(this->frames.size());

                        result = result & this->noiseRemovalPart(divisionNumber, this->frames.begin(), this->frames.end());
                        #else
                        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
                        for(auto it : this->frames)
                        {
                            sor.setInputCloud (it->cloud);
                            sor.setMeanK (it->cloud->points.size() * this->noiseRemovalPercentP);
                            sor.setStddevMulThresh (this->noiseRemovalStddevMulThresh);
                            sor.filter (*(it->cloud));
                        }
                        #endif
                    }
                }
                else if(((noiseRemovalParameter.size() % 2) == 0)&&(noiseRemovalParameter.size()!= 0))
                {
                    for(int i = 0; i < (this->noiseRemovalParameter.size()-1); i+=2)
                    {
                        this->noiseRemovalPercentP = this->noiseRemovalParameter[i];
                        this->noiseRemovalStddevMulThresh = this->noiseRemovalParameter[i+1];
                        #ifdef VELOFRAME_USE_MULTITHREAD
                        size_t divisionNumber = myFunction::getDivNum(this->frames.size());

                        result = result & this->noiseRemovalPart(divisionNumber, this->frames.begin(), this->frames.end());
                        #else
                        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
                        for(auto it : this->frames)
                        {
                            sor.setInputCloud (it->cloud);
                            sor.setMeanK (it->cloud->points.size() * this->noiseRemovalPercentP);
                            sor.setStddevMulThresh (this->noiseRemovalStddevMulThresh);
                            sor.filter (*(it->cloud));
                        }
                        #endif
                    }
                }
                for(int i = 0; i < this->frames.size(); ++i)
                {
                    if(this->frames[i]->cloud->points.size() == 0)
                    {
                        this->frames.erase(this->frames.begin() + i);
                        --i;
                        continue;
                    }
                }

                this->isNoiseRemoved = true;
                this->isChanged = true;
                this->isSaved = false;
            }

	        template<typename RandomIt1>
            bool offsetPart(const size_t &divisionNumber, const RandomIt1 &beg, const RandomIt1 &end)
            {
                auto len = end - beg;

                if(len < divisionNumber)
                {
                    bool out = true;;
                    for(auto it = beg; it != end; ++it)
                    {
                        for(auto it2 : (*it)->cloud->points)
                        {
                            it2.x -= this->offsetPoint.x;
                            it2.y -= this->offsetPoint.y;
                            it2.z -= this->offsetPoint.z;
                        }
                    }
                    return out;
                }

                auto mid = beg + len/2;
                auto handle = std::async(std::launch::async, &VeloFrames::offsetPart<RandomIt1>, this, divisionNumber, beg, mid);
                auto out1 = VeloFrames::offsetPart<RandomIt1>(divisionNumber, mid, end);
                auto out = handle.get();

                return out & out1;
            }

            bool offset()
            {
                bool result;

                if(this->isOffset)
                {
                    std::cout << "VeloFrame already offset" << std::endl;
                    return false;
                }
                if(this->frames.size() == 0)
                {
                    std::stringstream ss;
                    ss << "VeloFrame::VeloFrames::offset, ";
                    ss << "veloFrame is empty";
                    throw VeloFrameException(ss.str());
                    return false;
                }
                #ifdef VELOFRAME_USE_MULTITHREAD
                size_t divisionNumber = myFunction::getDivNum(this->frames.size());

                result = result & this->offsetPart(divisionNumber, this->frames.begin(), this->frames.end());
                #else
                for(auto it : this->frames)
                {
                    for(auto it2 : it->cloud->points)
                    {
                        it2.x -= this->offsetPoint.x;
                        it2.y -= this->offsetPoint.y;
                        it2.z -= this->offsetPoint.z;
                    }
                }
                #endif
                this->isOffset = true;
                return result;
            }

            void print(const bool showBasicInfo = true, const std::string &prefixString = "") const
            {
                std::cout << prefixString << "\033[1;33m";  //yellow
                std::cout << prefixString << "VeloFrames info : " << std::endl;
                std::cout << prefixString << "\033[0m";     //default color
                std::cout << prefixString << "-------" << std::endl;
                std::cout << prefixString << "pcap : " << this->pcapFilePath.string() << std::endl;
                std::cout << prefixString << "size : " << this->frames.size() << std::endl;
                if(this->isSaved)
                {
                    std::cout << prefixString << "output : " << this->outputPathWithParameter.string() << std::endl;
                }
                if(!showBasicInfo)
                {
                    for(int i = 0; i < this->frames.size(); i++)
                    {
                        std::cout << prefixString << "cloud : " << i << std::endl;
                        this->frames[i]->print(prefixString + "\t");
                    }
                }
            }
        private:

            void throwException(std::string functionName, std::string message)
            {
                std::stringstream ss;
                ss << "VeloFrame::VeloFrames::" << functionName << ", ";
                ss << message;
                throw VeloFrameException(ss.str());
            }

	        template<typename RandomIt1, typename RandomIt2>
            bool loadFromPcapPart(const size_t &divisionNumber, const RandomIt1 &beg1, const RandomIt1 &end1, const RandomIt2 &beg2, const RandomIt2 &end2)
            {
                auto len1 = end1 - beg1;
                auto len2 = end2 - beg2;

                if(len1 < divisionNumber)
                {
                    bool out = true;
                    long long minTimestamp;
                    long long midTimestamp;
                    long long maxTimestamp;
                    pcl::PointXYZI point;
                    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud;
                    auto it2 = beg2;
                    for(auto it1 = beg1; it1 != end1; ++it1,++it2)
                    {
                        minTimestamp = std::numeric_limits<long long>::max();
                        midTimestamp = 0;
                        maxTimestamp = std::numeric_limits<long long>::min();

                        cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

                        for( const velodyne::Laser& laser : (**it1) ){
                            const double distance = static_cast<double>( laser.distance );
                            const double azimuth  = laser.azimuth  * M_PI / 180.0;
                            const double vertical = laser.vertical * M_PI / 180.0;
                        
                            minTimestamp = std::min(laser.time, minTimestamp);
                            maxTimestamp = std::max(laser.time, maxTimestamp);
                            midTimestamp = (maxTimestamp - minTimestamp)/2 + minTimestamp;


                            point.x = static_cast<float>( ( distance * std::cos( vertical ) ) * std::sin( azimuth ) );
                            point.y = static_cast<float>( ( distance * std::cos( vertical ) ) * std::cos( azimuth ) );
                            point.z = static_cast<float>( ( distance * std::sin( vertical ) ) );
                        
                            point.intensity = laser.intensity;

                            if( point.x == 0.0f && point.y == 0.0f && point.z == 0.0f ){
                                point.x = std::numeric_limits<float>::quiet_NaN();
                                point.y = std::numeric_limits<float>::quiet_NaN();
                                point.z = std::numeric_limits<float>::quiet_NaN();
                            }
                        
                            cloud->points.push_back(point);
                        }

                        if((minTimestamp < this->begTime.count())&&(minTimestamp > this->endTime.count())) continue;

                        cloud->width = static_cast<uint32_t>(cloud->points.size());
                        cloud->height = 1;

                        (*it2).reset(new VeloFrame());
                        (*it2)->cloud = cloud;
                        (*it2)->minTimestamp = std::chrono::microseconds(minTimestamp);
                        (*it2)->midTimestamp = std::chrono::microseconds(midTimestamp);
                        (*it2)->maxTimestamp = std::chrono::microseconds(maxTimestamp);
                    }
                    return out;
                }

                auto mid1 = beg1 + len1/2;
                auto mid2 = beg2 + len1/2;
                auto handle = std::async(std::launch::async, &VeloFrames::loadFromPcapPart<RandomIt1, RandomIt2>, this, divisionNumber, beg1, mid1, beg2, mid2);
                auto out1 = VeloFrames::loadFromPcapPart<RandomIt1, RandomIt2>(divisionNumber, mid1, end1, mid2, end2);
                auto out = handle.get();

                return out & out1;
            }

            bool loadFromPcap()
            {
                velodyne::VLP16Capture vlp16;

                if(!vlp16.open(this->pcapFilePath.string()))
                {
                    std::cout << std::endl << "Error : load " << this->pcapFilePath << " failed" << std::endl;
                    return false;
                }

                if(!vlp16.isOpen())
                {
                    std::cout << std::endl << "Error : load " << this->pcapFilePath << " failed" << std::endl;
                    return false;
                }

                #ifdef VELOFRAME_USE_MULTITHREAD
                std::vector<boost::shared_ptr<std::vector<velodyne::Laser>>> f;

                while(vlp16.isRun())
                {
                    boost::shared_ptr<std::vector<velodyne::Laser>> lasers(new std::vector<velodyne::Laser>);
                    vlp16 >> *lasers;
                    if( (*lasers).empty() ){
                        continue;
                    }
                    f.push_back(lasers);
                }
                this->frames.resize(f.size());

                size_t divisionNumber = myFunction::getDivNum(f.size());

                bool result = this->loadFromPcapPart(divisionNumber, f.begin(), f.end(), this->frames.begin(), this->frames.end());
                
                for(int i = 0; i < this->frames.size(); ++i)
                {
                    if(this->frames[i] == NULL)
                    {
                        this->frames.erase(this->frames.begin() + i);
                        --i;
                        continue;
                    }
                }
                
                #else
                boost::shared_ptr<VeloFrame> frame;
                long long minTimestamp;
                long long midTimestamp;
                long long maxTimestamp;
                pcl::PointXYZI point;
                boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud;
                while(vlp16.isRun())
                {
                    std::vector<velodyne::Laser> lasers;
                    vlp16 >> lasers;
                    if( lasers.empty() ){
                        continue;
                    }

                    minTimestamp = std::numeric_limits<long long>::max();
                    midTimestamp = 0;
                    maxTimestamp = std::numeric_limits<long long>::min();

                    cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

                    for( const velodyne::Laser& laser : lasers ){
                        const double distance = static_cast<double>( laser.distance );
                        const double azimuth  = laser.azimuth  * M_PI / 180.0;
                        const double vertical = laser.vertical * M_PI / 180.0;
                    
                        minTimestamp = std::min(laser.time, minTimestamp);
                        maxTimestamp = std::max(laser.time, maxTimestamp);
                        midTimestamp = (maxTimestamp - minTimestamp)/2 + minTimestamp;

                        point.x = static_cast<float>( ( distance * std::cos( vertical ) ) * std::sin( azimuth ) );
                        point.y = static_cast<float>( ( distance * std::cos( vertical ) ) * std::cos( azimuth ) );
                        point.z = static_cast<float>( ( distance * std::sin( vertical ) ) );
                    
                        point.intensity = laser.intensity;

                        if( point.x == 0.0f && point.y == 0.0f && point.z == 0.0f ){
                            point.x = std::numeric_limits<float>::quiet_NaN();
                            point.y = std::numeric_limits<float>::quiet_NaN();
                            point.z = std::numeric_limits<float>::quiet_NaN();
                        }
                    
                        cloud->points.push_back(point);
                    }
                    if((minTimestamp < this->begTime.count())&&(minTimestamp > this->endTime.count())) continue;
                    
                    cloud->width = static_cast<uint32_t>(cloud->points.size());
                    cloud->height = 1;

                    frame.reset(new VeloFrame());
                    frame->cloud = cloud;
                    
                    frame->minTimestamp = std::chrono::microseconds(minTimestamp);
                    frame->midTimestamp = std::chrono::microseconds(midTimestamp);
                    frame->maxTimestamp = std::chrono::microseconds(maxTimestamp);
                    this->frames.push_back(frame);
                }
                #endif
                this->isChanged = true;
                return true;
            }

	        template<typename RandomIt1, typename RandomIt2>
            bool loadFromFolderPart(const size_t &divisionNumber, const RandomIt1 &beg1, const RandomIt1 &end1, const RandomIt2 &beg2, const RandomIt2 &end2)
            {
                auto len1 = end1 - beg1;
                auto len2 = end2 - beg2;

                if(len1 < divisionNumber)
                {
                    bool out = true;
                    auto it2 = beg2;
                    for(auto it1 = beg1; it1 != end1; ++it1,++it2)
                    {
                        std::vector<std::string> ss;
                        boost::filesystem::path cloudPath = this->outputPath;
                        boost::split(ss, *it1, boost::is_any_of("&"));

                        if((std::stoll(ss[0]) < this->begTime.count())&&(std::stoll(ss[0]) > this->endTime.count())) continue;

                        (*it2).reset(new VeloFrame());
                        (*it2)->cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
                        (*it2)->setMinTimestamp(std::stoll(ss[0]));
                        (*it2)->setMidTimestamp(std::stoll(ss[1]));
                        (*it2)->setMaxTimestamp(std::stoll(ss[2]));
                        cloudPath.append(ss[3]);
                        pcl::io::loadPCDFile(cloudPath.string(), *((*it2)->cloud));
                    }
                    return out;
                }

                auto mid1 = beg1 + len1/2;
                auto mid2 = beg2 + len1/2;
                auto handle = std::async(std::launch::async, &VeloFrames::loadFromFolderPart<RandomIt1, RandomIt2>, this, divisionNumber, beg1, mid1, beg2, mid2);
                auto out1 = VeloFrames::loadFromFolderPart<RandomIt1, RandomIt2>(divisionNumber, mid1, end1, mid2, end2);
                auto out = handle.get();

                return out & out1;
            }

            bool loadFromFolder()
            {
                boost::filesystem::path configPath = this->outputPathWithParameter;
                configPath.append("config.txt");

                if(!boost::filesystem::exists(configPath))
                {
                    std::stringstream ss;
                    ss << "VeloFrame::VeloFrames::loadFromFolder, ";
                    ss << boost::filesystem::absolute(configPath).string();
                    ss << " not found.";
                    throw VeloFrameException(ss.str());
                    return false;
                }

                std::ifstream ifs(configPath.string());

                if(!ifs.is_open()) return false;

                std::string line;
                std::vector<std::string> ss1s;
                std::vector<std::string> f;

                while(!ifs.eof())
                {
                    std::getline(ifs, line);
                    boost::trim(line);
                    if(line == "") continue;
                    ss1s.push_back(line);
                }

                for(auto ss1 : ss1s)
                {
                    std::vector<std::string> ss2s;
                    boost::split(ss2s, ss1, boost::is_any_of("="));

                    if(ss2s[0] == "pcap") this->pcapFilePath = boost::filesystem::path{ss2s[1]};
                    else if(ss2s[0] == "background")
                    {
                        boost::filesystem::path backgroundPath = this->outputPath;
                        this->backgroundCloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
                        backgroundPath.append(ss2s[1]);
                        pcl::io::loadPCDFile(backgroundPath.string(), *(this->backgroundCloud));
                    }
                    else
                    {
                        f.push_back(ss2s[1]);
                    }
                }
                this->frames.resize(f.size());

                #ifdef VELOFRAME_USE_MULTITHREAD

                size_t divisionNumber = myFunction::getDivNum(f.size());

                bool result = this->loadFromFolderPart(divisionNumber, f.begin(), f.end(), this->frames.begin(), this->frames.end());
                
                for(int i = 0; i < this->frames.size(); ++i)
                {
                    if(this->frames[i] == NULL)
                    {
                        this->frames.erase(this->frames.begin() + i);
                        --i;
                        continue;
                    }
                }
                
                #else
                auto it2 = this->frames.begin();
                for(auto it1 : f)
                {
                    std::vector<std::string> ss;
                    boost::filesystem::path cloudPath = this->outputPath;
                    boost::split(ss, it1, boost::is_any_of("&"));

                    if((std::stoll(ss[0]) < this->begTime.count())&&(std::stoll(ss[0]) > this->endTime.count())) continue;

                    (*it2).reset(new VeloFrame());
                    (*it2)->cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
                    (*it2)->setMinTimestamp(std::stoll(ss[0]));
                    (*it2)->setMidTimestamp(std::stoll(ss[1]));
                    (*it2)->setMaxTimestamp(std::stoll(ss[2]));
                    cloudPath.append(ss[3]);
                    pcl::io::loadPCDFile(cloudPath.string(), *((*it2)->cloud));
                    it2++;
                }
                #endif

                std::sort(this->frames.begin(), this->frames.end(), [](const auto &a, const auto &b){ return a->minTimestamp < b->minTimestamp; });

                return true;
            }

	        template<typename RandomIt1>
            bool savePart(const size_t &divisionNumber, const RandomIt1 &beg, const RandomIt1 &end)
            {
                auto len = end - beg;

                if(len < divisionNumber)
                {
                    bool out = true;;
                    for(auto it = beg; it != end; ++it)
                    {
                        boost::filesystem::path cloudPath = this->outputPathWithParameter;
                        cloudPath.append(myFunction::durationToString((*it)->minTimestamp) + ".pcd");
                        pcl::io::savePCDFileBinaryCompressed(cloudPath.string(), *((*it)->cloud));
                    }
                    return out;
                }

                auto mid = beg + len/2;
                auto handle = std::async(std::launch::async, &VeloFrames::savePart<RandomIt1>, this, divisionNumber, beg, mid);
                auto out1 = VeloFrames::savePart<RandomIt1>(divisionNumber, mid, end);
                auto out = handle.get();

                return out & out1;
            }

	        template<typename RandomIt1>
            bool backgroundSegmentationPart(const size_t &divisionNumber, const myClass::backgroundSegmentation<pcl::PointXYZI> backgroundSegmentation, const RandomIt1 &beg, const RandomIt1 &end)
            {
                auto len = end - beg;

                if(len < divisionNumber)
                {
                    bool out = true;;
                    for(auto it = beg; it != end; ++it)
                    {
                        (*it)->cloud = backgroundSegmentation.compute((*it)->cloud);
                    }
                    return out;
                }

                auto mid = beg + len/2;
                auto handle = std::async(std::launch::async, &VeloFrames::backgroundSegmentationPart<RandomIt1>, this, divisionNumber, backgroundSegmentation, beg, mid);
                auto out1 = VeloFrames::backgroundSegmentationPart<RandomIt1>(divisionNumber, backgroundSegmentation, mid, end);
                auto out = handle.get();

                return out & out1;
            }

	        template<typename RandomIt1>
            bool noiseRemovalPart(const size_t &divisionNumber, const RandomIt1 &beg, const RandomIt1 &end)
            {
                auto len = end - beg;

                if(len < divisionNumber)
                {
                    bool out = true;;
                    for(auto it = beg; it != end; ++it)
                    {
                        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
                        sor.setInputCloud ((*it)->cloud);
                        sor.setMeanK ((*it)->cloud->points.size() * this->noiseRemovalPercentP);
                        sor.setStddevMulThresh (this->noiseRemovalStddevMulThresh);
                        sor.filter (*((*it)->cloud));
                    }
                    return out;
                }

                auto mid = beg + len/2;
                auto handle = std::async(std::launch::async, &VeloFrames::noiseRemovalPart<RandomIt1>, this, divisionNumber, beg, mid);
                auto out1 = VeloFrames::noiseRemovalPart<RandomIt1>(divisionNumber, mid, end);
                auto out = handle.get();

                return out & out1;
            }

    };

    class VeloFrameViewer : protected pcl::visualization::PCLVisualizer
    {
        private:
            bool pause;
            bool showBackground;
            bool realTime;
            bool startTimeReset;
            double playSpeedRate;
            double pointSize;
            std::vector<boost::shared_ptr<VeloFrame>> frames;
            std::queue<boost::shared_ptr<VeloFrame>> queue;
            std::chrono::steady_clock::time_point startTime;
            std::chrono::microseconds startTimestamp;
            boost::function<void (const pcl::visualization::KeyboardEvent&, void*)> externalKeyboardEventOccurred;

        public:
            VeloFrameViewer() : pcl::visualization::PCLVisualizer()
            {
                this->pause = false;
                this->showBackground = false;
                this->realTime = true;
                this->playSpeedRate = 1.0;
                this->pointSize = 1.0;
                this->startTime = std::chrono::steady_clock::now();
                this->startTimestamp = std::chrono::microseconds(int64_t(0));
                this->addCoordinateSystem( 3.0, "coordinate" );
                this->setBackgroundColor( 0.0, 0.0, 0.0, 0 );
                this->initCameraParameters();
                this->setCameraPosition( 0.0, 0.0, 2000.0, 0.0, 1.0, 0.0, 0 );

            }

            void addFrames(const VeloFrames &vf)
            {
                for(auto it : vf.frames)
                {
                    boost::shared_ptr<VeloFrame> temp(new VeloFrame);
                    *temp = *it;
                    this->frames.push_back(temp);
                }
                std::sort(this->frames.begin(), this->frames.end(), [](const auto &a, const auto &b){ return a->minTimestamp < b->minTimestamp; });
                for(int i = 0; i < (this->frames.size()-1); ++i)
                {
                    if(this->frames[i]->minTimestamp == this->frames[i+1]->minTimestamp)
                    {
                        this->frames.erase(this->frames.begin() + i);
                        --i;
                        continue;
                    }
                }
            }

            void run()
            {
                boost::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>> handler( new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>( "intensity" ) );
                boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>> handler1( new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>( 1.0, 1.0, 1.0 ) );
                std::chrono::microseconds nextTimestamp;

                for(auto it : this->frames)
                {
                    this->queue.push(it);
                }

                this->startTime = std::chrono::steady_clock::now();
                this->startTimestamp = this->queue.front()->minTimestamp - std::chrono::microseconds(int64_t(1000000/5));
                nextTimestamp = this->queue.front()->minTimestamp;
                while(!this->wasStopped())
                {
                    this->spinOnce();
                    
                    if(this->pause) continue;

                    if(this->startTimeReset)
                    {
                        this->startTime = std::chrono::steady_clock::now();
                        this->startTimestamp = this->queue.front()->minTimestamp - std::chrono::microseconds(int64_t(1000000/5));
                        this->startTimeReset = false;
                    }

                    bool stay = (((std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - this->startTime).count()*this->playSpeedRate) < (nextTimestamp-this->startTimestamp).count()) && this->realTime);

                    if(stay) continue;

                    auto it = this->queue.front();
                    this->queue.pop();
                    
                    handler->setInputCloud( it->cloud );
                    if( !this->updatePointCloud( it->cloud, *handler, "cloud" ) ){
                        this->addPointCloud( it->cloud, *handler, "cloud" );
                    }
                    this->queue.push(it);
                    
                    if(nextTimestamp > this->queue.front()->midTimestamp)
                    {
                        this->startTime = std::chrono::steady_clock::now();
                        this->startTimestamp = this->queue.front()->minTimestamp - std::chrono::microseconds(int64_t(1000000/5));
                    }
                    nextTimestamp = this->queue.front()->minTimestamp;
                }
            }

            inline void registerKeyboardCallback (void (*callback) (const pcl::visualization::KeyboardEvent&, void*), void* cookie = NULL)
            {
                this->externalKeyboardEventOccurred = boost::bind(callback, _1, cookie);
                this->pcl::visualization::PCLVisualizer::registerKeyboardCallback(&VeloFrameViewer::keyboardEventOccurred, *this);
            }

            void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
            {
                if(event.isCtrlPressed())
                {
                    if(((event.getKeySym() == "b")||(event.getKeySym() == "B"))&&(event.keyDown()))
                    {
                        this->showBackground = !this->showBackground;
                        std::cout << "showBackground: " << ((this->showBackground == true)? "ON" : "OFF") << std::endl;
                    }
                    else if((event.getKeySym() == "space")&&(event.keyDown()))
                    {
                        this->realTime = !this->realTime;
                        std::cout << "realTime: " << ((this->realTime == true)? "ON" : "OFF") << std::endl;
                    }
                    else if((event.getKeySym() == "Up")&&(event.keyDown()))
                    {
                        if(this->playSpeedRate < 10.0) this->playSpeedRate += 0.25;
                        this->startTimeReset = true;
                        std::cout << "playSpeedRate: " << this->playSpeedRate << std::endl;
                    }
                    else if((event.getKeySym() == "Down")&&(event.keyDown()))
                    {
                        if(this->playSpeedRate > 0.25) this->playSpeedRate -= 0.25;
                        this->startTimeReset = true;
                        std::cout << "playSpeedRate: " << this->playSpeedRate << std::endl;
                    }
                    else
                    {
                        this->externalKeyboardEventOccurred(event, nothing);
                    }
                }
                else
                {
                    if((event.getKeySym() == "space")&&(event.keyDown()))
                    {
                        this->pause = !this->pause;
                        std::cout << "viewer_pause: " << ((this->pause == true)? "ON" : "OFF") << std::endl;
                    }
                    else if((event.getKeySym() == "KP_Add")&&(event.keyDown()))
                    {
                        if(this->pointSize < 10.0) this->pointSize += 0.5;
                    }
                    else if((event.getKeySym() == "KP_Subtract")&&(event.keyDown()))
                    {
                        if(this->pointSize > 1.0) this->pointSize -= 0.5;
                    }
                    else if(((event.getKeySym() == "h")||(event.getKeySym() == "H"))&&(event.keyDown()))
                    {
                        std::cout << std::endl;
                        std::cout << "\033[1;33m";  //yellow
                        std::cout << "| VeloFrameViewer added:" << std::endl;
                        std::cout << "\033[0m";     //default color
                        std::cout << "-------         " << std::endl;
                        std::cout << "    CTRL + b, B " << " : show background (on/off)" << std::endl;
                        std::cout << "    CTRL + space" << " : switch between real-time play and play as fast as possible (on/off)" << std::endl;
                        std::cout << "    CTRL + Up   " << " : play speed rate +0.25 (max 10.0x)" << std::endl;
                        std::cout << "    CTRL + Down " << " : play speed rate -0.25 (min 0.25x)" << std::endl;
                        std::cout << "    space       " << " : switch between play and pause" << std::endl;
                        std::cout << "    KP_Add      " << " : point size +0.5 (max 10)" << std::endl;
                        std::cout << "    KP_Substract" << " : point size -0.5 (min 1)" << std::endl;
                    }
                    else
                    {
                        this->externalKeyboardEventOccurred(event, nothing);
                    }
                }
            }
    };
}

#endif