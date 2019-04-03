#include <mutex>
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
//#include <thread>
//#include <pcl/io/vlp_grabber.h>
#include "../3rdparty/args/args.hxx"
#include "../include/microStopwatch.h"
#include "../include/velodyne/pcap_cache.h"
#include "../include/function.h"
#include "../include/backgroundAndDynamic.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef boost::shared_ptr<PointCloudT> PointCloudPtrT;


pcl::visualization::PCLVisualizer::Ptr viewer;

bool viewerPause = false;
bool viewerMeanKUp = false;
bool viewerMeanKDown = false;
bool viewerStddevMulThreshUp = false;
bool viewerStddevMulThreshDown = false;

bool dynamicObject = false;
args::ArgumentParser parser("This is a test program.", "This goes after the options.");
args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});

args::Group requirementGroup(parser, "This group is all required:", args::Group::Validators::All);
    args::ValueFlagList<std::string> inputPcapArg(requirementGroup, "inputPcap1", "input pcap1", {'p'});
args::Group optionalGroup(parser, "This group is all optional:", args::Group::Validators::DontCare);
    args::ValueFlag<std::string> backgroundPcdArg(optionalGroup, "backgroundPcd", "background pcd", {"bg"});
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing);

void mouseEventOccurred(const pcl::visualization::MouseEvent& event, void* nothing)
{
    if((event.getButton() == pcl::visualization::MouseEvent::MouseButton::LeftButton)||(event.getButton() == pcl::visualization::MouseEvent::MouseButton::MiddleButton)||(event.getButton() == pcl::visualization::MouseEvent::MouseButton::RightButton)) {
        if(event.getType() == pcl::visualization::MouseEvent::MouseButtonPress) {
            /*system("xte \'mousedown 4\'");
            system("xte \'mouseup 4\'");
            system("xte \'mousedown 5\'");
            system("xte \'mouseup 5\'");*/
        }
    }
}

int main(int argc, char * argv[])
{
    try
    {
        parser.ParseCLI(argc, argv);
        
        myClass::MicroStopwatch tt1("pcap cache convert");
        
        std::vector<boost::filesystem::path> pcapPaths;
        for(auto &pcapPath : args::get(inputPcapArg)) {
            pcapPaths.push_back(pcapPath);
        }

        boost::filesystem::path backgroundPcdPath{args::get(backgroundPcdArg)};

        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud(new pcl::PointCloud<pcl::PointXYZ>);
        //PointCloudPtrT cloud(new PointCloudT);
        
        velodyne::PcapCache<PointT> pcapCache("/home/user/Downloads/pcapcache");

        for(int i = 0; i < pcapPaths.size(); i++) {
            pcapCache.add(pcapPaths[i].string(), "default");
        }    
        pcapCache.convert();

        viewer.reset(new pcl::visualization::PCLVisualizer( "Velodyne Viewer" ));
        viewer->registerKeyboardCallback(keyboardEventOccurred, (void*) NULL);
        viewer->registerMouseCallback(&mouseEventOccurred, (void*) NULL);
        viewer->addCoordinateSystem( 3.0, "coordinate" );
        viewer->setCameraPosition( 0.0, 0.0, 1000.0, 0.0, 1.0, 0.0, 0 );

        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> back(new pcl::PointCloud<pcl::PointXYZ>());

        int compareFrameNumber = 1000;
        double resolution = 10.0;
        uint64_t displayFrameIndex = 0;
        int meanKNumber = 50;
        double stddevMulThreshNumber = 1.0;

        back = myFunction::getBackground<pcl::PointXYZ>(pcapCache, compareFrameNumber, resolution);

        velodyne::PcapCache<PointT> pcapCacheNoBack("/home/user/Downloads/pcapcache/noBack");

        for(int i = 0; i < pcapPaths.size(); i++) {
            pcapCacheNoBack.add(pcapPaths[i].string(), "default");
        }    
		pcapCacheNoBack.addBack(back, resolution);
        pcapCacheNoBack.convert();

        int objectStopFrameRange = 100;
        std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> nochangeFrameNumber;

        //nochangeFrameNumber = myFunction::nochangeFrameIndex<pcl::PointXYZ>(pcapCacheNoBack, objectStopFrameRange, objectNoChangesFramePoints, objectNoChangesFramePoints2, resolution, meanKNumber, stddevMulThreshNumber);
        
        std::cout << "Complete nochangeFrameNumber!" << std::endl;

		//myFunction::updateCloud(viewer, back, "back", 255, 255, 255);
        while( !viewer->wasStopped() ){
            viewer->spinOnce();
            /*
            if(viewerMeanKUp == true){ 
                            meanKNumber++; 
                            std::cout<< "meanKNumber=" << meanKNumber << "  stddevMulThreshNumber=" << stddevMulThreshNumber << endl; 
                            viewerMeanKUp = false;
                                    }
            if(viewerMeanKDown == true){ 
                            meanKNumber--; 
                            std::cout<< "meanKNumber=" << meanKNumber << "  stddevMulThreshNumber=" << stddevMulThreshNumber << endl; 
                            viewerMeanKDown = false;
            }
            if(viewerStddevMulThreshUp == true){ 
                            stddevMulThreshNumber++; 
                            std::cout<< "meanKNumber=" << meanKNumber << "  stddevMulThreshNumber=" << stddevMulThreshNumber << endl; 
                            viewerStddevMulThreshUp = false;
            }
            if(viewerStddevMulThreshDown == true){ 
                            stddevMulThreshNumber--; 
                            std::cout<< "meanKNumber=" << meanKNumber << "  stddevMulThreshNumber=" << stddevMulThreshNumber << endl; 
                            viewerStddevMulThreshDown = false;
            }
            */
/*
            cloud = (dynamicObject == true) ? 
                myFunction::getChanges<pcl::PointXYZ>(nochangeFrameNumber[displayFrameIndex], myFunction::getStatisticalOutlierRemoval<pcl::PointXYZ>(pcapCacheNoBack.get(displayFrameIndex), meanKNumber, stddevMulThreshNumber), resolution) : 
                pcapCache.get(displayFrameIndex);
*/
            cloud = (dynamicObject == true) ? 
                pcapCacheNoBack.get(displayFrameIndex) : 
                pcapCache.get(displayFrameIndex);

            

            if(cloud->points.size() == 0) continue;

            myFunction::updateCloud<pcl::PointXYZ>(viewer, cloud, "cloud", 1.0, false, 0.0, 2000.0);
            //myFunction::updateCloud(viewer, nochangeFrameNumber[displayFrameIndex], "tempNowFiltered", 255, 255, 255);

            if(++displayFrameIndex >= pcapCache.totalFrame)displayFrameIndex = 0;
        }

        while(1);
    }
    catch (args::Help)
    {
        std::cout << parser;
        return 0;
    }
    catch (args::ParseError e)
    {
        std::cout << e.what() << std::endl;
        parser.Help(std::cout);
        return 1;
    }
    catch (args::ValidationError e)
    {
        std::cout << e.what() << std::endl;
        parser.Help(std::cout);
        return 1;
    }

    return 0;
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
{
    if((event.getKeySym() == "a")&&(event.keyDown()))
    {
        dynamicObject = !dynamicObject;
    }
    else if((event.getKeySym() == "q")&&(event.keyDown())){
        viewerMeanKUp = true;
    }
    else if((event.getKeySym() == "w")&&(event.keyDown())){
         viewerMeanKDown = true;
    }
    else if((event.getKeySym() == "e")&&(event.keyDown())){
         viewerStddevMulThreshUp = true;
    }
    else if((event.getKeySym() == "r")&&(event.keyDown())){
         viewerStddevMulThreshDown = true;
    }
    else if((event.getKeySym() == "space")&&(event.keyDown()))
    {
        viewerPause = !viewerPause;
    }
    else 
    {
        std::cout << "Keyboard pressed: " << ((event.isAltPressed())? "Alt + " : "") << ((event.isCtrlPressed())? "Ctrl + " : "") << ((event.isShiftPressed())? "Shift + " : "") << event.getKeySym() << std::endl;
    }
}
