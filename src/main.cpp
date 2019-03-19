#include <mutex>
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/point_types.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
//#include <thread>
//#include <pcl/io/vlp_grabber.h>
#include "../3rdparty/args/args.hxx"
#include "../include/microStopwatch.h"
#include "../include/velodyne/velodyne.h"
#include "../include/velodyne/velodyne_grabber.h"
#include "../include/function.h"
#include "../include/lasers.h"
#include <X11/Xlib.h>
#include <X11/Xutil.h>

std::mutex cloudMutex;
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> inputCloud_1;
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> inputCloud_2;

velodyne::VLP16Grabber grabber;
pcl::visualization::PCLVisualizer::Ptr viewer;
uint64_t startTime;
uint64_t startTime2;

bool viewerPause = false;
double totalTheta = 0.0;
pcl::PointXYZ pointOrigin;
pcl::PointXYZ pointMoved;
int jjj = 0;

boost::filesystem::path tmp_path{"/tmp/LiDar"};

args::ArgumentParser parser("This is a test program.", "This goes after the options.");
args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});

args::Group requirementGroup(parser, "This group is all required:", args::Group::Validators::All);
    args::ValueFlag<std::string> inputPcapArg_1(requirementGroup, "inputPcap1", "input pcap1", {"p1"});
    args::ValueFlag<std::string> inputPcapArg_2(requirementGroup, "inputPcap2", "input pcap2", {"p2"});
args::Group optionalGroup(parser, "This group is all optional:", args::Group::Validators::DontCare);
    args::ValueFlag<std::string> backgroundPcdArg(optionalGroup, "backgroundPcd", "background pcd", {"bg"});
    
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing);

void mouseEventOccurred(const pcl::visualization::MouseEvent& event, void* nothing)
{
    std::cout << std::to_string(jjj++) << std::endl;
    if((event.getButton() == pcl::visualization::MouseEvent::MouseButton::LeftButton)||(event.getButton() == pcl::visualization::MouseEvent::MouseButton::MiddleButton)||(event.getButton() == pcl::visualization::MouseEvent::MouseButton::RightButton)) {
        if(event.getType() == pcl::visualization::MouseEvent::MouseButtonPress) {
            system("xte \'mousedown 4\'");
            system("xte \'mouseup 4\'");
            system("xte \'mousedown 5\'");
            system("xte \'mouseup 5\'");
        }
    }
}

int main(int argc, char * argv[])
{
    try
    {
        parser.ParseCLI(argc, argv);
        
        myClass::MicroStopwatch tt1;
        myClass::MicroStopwatch tt2;
        myClass::MicroStopwatch tt3;
        
        boost::filesystem::path pcapPath_1{args::get(inputPcapArg_1)};
        boost::filesystem::path pcapPath_2{args::get(inputPcapArg_2)};
        boost::filesystem::path backgroundPcdPath{args::get(backgroundPcdArg)};

        boost::shared_ptr<velodyne::VLP16> vlp16_1(new velodyne::VLP16);
        boost::shared_ptr<velodyne::VLP16> vlp16_2(new velodyne::VLP16);

        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> combinedCloud;
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> back(new pcl::PointCloud<pcl::PointXYZ>());

        if(!myFunction::fileExists(tmp_path.string()))
        {
            mkdir(tmp_path.string().c_str(), 0777);
        }

        if(!vlp16_1->open(pcapPath_1.string()))
        {
            std::cout << std::endl << "Error : load " << pcapPath_1.string() << " failed" << std::endl;
            return false;
        }

        if(!vlp16_2->open(pcapPath_2.string()))
        {
            std::cout << std::endl << "Error : load " << pcapPath_2.string() << " failed" << std::endl;
            return false;
        }

        vlp16_2->setOffset(-0.15708, -1044.3277129076, -94.3663384411, 37);

        viewer.reset(new pcl::visualization::PCLVisualizer( "Velodyne Viewer" ));
        viewer->registerKeyboardCallback(&keyboardEventOccurred, (void*) NULL);
        viewer->registerMouseCallback(&mouseEventOccurred, (void*) NULL);
        viewer->addCoordinateSystem( 3.0, "coordinate" );
        viewer->setCameraPosition( 0.0, 0.0, 1000.0, 0.0, 1.0, 0.0, 0 );

        //boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_1(myFunction::XYZ_to_XYZRGB<pcl::PointXYZ>(inputCloud_1, false));
        //boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_2(myFunction::XYZ_to_XYZRGB<pcl::PointXYZ>(inputCloud_2, false));
        combinedCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        //combinedCloud = myFunction::combineCloud<pcl::PointXYZ>(inputCloud_1, inputCloud_2);


        //myFunction::showCloud<pcl::PointXYZ>(viewer, combinedCloud, "combinedCloud", 1.0, false, 0.0, 2000.0);
        //myFunction::showCloud(viewer, cloud_1, "cloud_1");
        //myFunction::showCloud(viewer, cloud_2, "cloud_2");


        if(backgroundPcdArg) {
            pcl::io::loadPCDFile(backgroundPcdPath.string(), *back);
        }

        boost::mutex mutex;
        boost::function<void( const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& )> function =
            [ &combinedCloud, &mutex ]( const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& ptr ){
                    std::cout << "DDDD" << std::endl;
                boost::mutex::scoped_lock lock( mutex );
                    std::cout << "EEEE" << std::endl;

                combinedCloud = ptr;
                    std::cout << "FFFF" << std::endl;
            };// VLP Grabber


        grabber.add(vlp16_1, 0);
        grabber.add(vlp16_2, 2);
        grabber.registerCallback(function);
        //grabber.toFolder(tmp_path.string());

        
        grabber.start();
        int i = 0;



        while( !viewer->wasStopped() ){
            viewer->spinOnce();
            boost::mutex::scoped_try_lock lock( mutex );
            if( lock.owns_lock() && combinedCloud ){
                myFunction::updateCloud<pcl::PointXYZ>(viewer, combinedCloud, "combinedCloud", 1.0, false, 0.0, 2000.0);

            }
        }


        while( !viewer->wasStopped() ){
            viewer->spinOnce();

            if(viewerPause) continue;
            
            tt1.tic();
            inputCloud_1 = myFunction::getChanges<pcl::PointXYZ>(back, inputCloud_1, 1.0);
            tt1.toc_print_string();
            tt2.tic();
            inputCloud_2 = myFunction::getChanges<pcl::PointXYZ>(back, inputCloud_2, 1.0);
            tt2.toc_print_string();


            //auto cloud_1 = myFunction::XYZ_to_XYZRGB<pcl::PointXYZ>(inputCloud_1, false);
            //cloud_2 = myFunction::XYZ_to_XYZRGB<pcl::PointXYZ>(inputCloud_2, false);
            tt3.tic();
            combinedCloud = myFunction::combineCloud<pcl::PointXYZ>(inputCloud_1, inputCloud_2);
            tt3.toc_print_string();
            /*
            auto noChanges_1 = myFunction::getNoChanges<pcl::PointXYZ>(back, combinedCloud, 1.0);
            auto noChanges_2 = myFunction::getNoChanges<pcl::PointXYZ>(combinedCloud, back, 10.0);

            tt3.tic();
            back = myFunction::combineCloud<pcl::PointXYZ>(noChanges_1, noChanges_2);
            myFunction::removeTooClosedPoint<pcl::PointXYZ>(back, 0.5);
            pcl::io::savePCDFileBinaryCompressed("back.pcd", *(back));

            tt3.toc_print_string();
            
*/
            myFunction::updateCloud<pcl::PointXYZ>(viewer, combinedCloud, "combinedCloud", 1.0, false, 0.0, 2000.0);

            myFunction::updateCloud(viewer, myFunction::fillColor<pcl::PointXYZ>(back, 255,255,255), "back");

            //myFunction::updateCloud<pcl::PointXYZ>(viewer, inputCloud_1, "inputCloud_1", 1.0, false, 0.0, 2000.0);            
            //Function::updateCloud(viewer, cloud_2, "cloud_2");
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
    if((event.getKeySym() == "Up")&&(event.keyDown()))
    {
        
    }
    else 
    {
        std::cout << "Keyboard pressed: " << ((event.isAltPressed())? "Alt + " : "") << ((event.isCtrlPressed())? "Ctrl + " : "") << ((event.isShiftPressed())? "Shift + " : "") << event.getKeySym() << std::endl;
    }
}
