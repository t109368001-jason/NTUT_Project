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

#include <pcl/point_representation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new PointCloud) {};
};

struct PCDComparator
{
  bool operator () (const PCD& p1, const PCD& p2)
  {
    return (p1.f_name < p2.f_name);
  }
};


// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    grid.setLeafSize (0.05, 0.05, 0.05);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }


  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (10);  
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);



  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (100);
  for (int i = 0; i < 300; ++i)
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource (points_with_normals_src);
    reg.align (*reg_result);

		//accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.1);
    
    prev = reg.getLastIncrementalTransformation ();

  }

	//
  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  //
  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

  PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);

	PCL_INFO ("Press q to continue the registration.\n");
  //add the source to the transformed target
  *output += *cloud_src;
  
  final_transform = targetToSource;
 }

std::mutex cloudMutex;
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> inputCloud_1;
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> inputCloud_2;

velodyne::VLP16Grabber grabber;
pcl::visualization::PCLVisualizer::Ptr viewer;
uint64_t startTime;
uint64_t startTime2;

bool viewerPause = false;

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
        Eigen::Matrix4f m;
        m(0,3) = 1;
        std::cout << m << std::endl;
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
                boost::mutex::scoped_lock lock( mutex );

                combinedCloud = ptr;
            };// VLP Grabber


        grabber.add(vlp16_1, 0);
        grabber.add(vlp16_2, 2);
        grabber.registerCallback(function);
        //grabber.toFolder(tmp_path.string());

        vlp16_1->moveToNext(100);
        *vlp16_1 >> inputCloud_1;
        vlp16_2->moveToNext(100);
        *vlp16_2 >> inputCloud_2;
        Eigen::Matrix4f transformMatrix;
        combinedCloud = myFunction::combineCloud<pcl::PointXYZ>(inputCloud_1, inputCloud_2);
        myFunction::updateCloud<pcl::PointXYZ>(viewer, combinedCloud, "combinedCloud1", 1.0, true, 0.0, 2000.0);
        pairAlign(inputCloud_1, inputCloud_2, combinedCloud, transformMatrix);
        std::cout << transformMatrix << std::endl;
        //[ 0.999926    -0.00503847 -0.0111259  -4.22016
        //  0.0052693    0.99977     0.020816    5.87466
        //  0.0110184   -0.020873    0.999721    19.8655
        // -0           -0          -0           1       ]
        //combinedCloud = myFunction::combineCloud<pcl::PointXYZ>(inputCloud_1, inputCloud_2);
        myFunction::updateCloud<pcl::PointXYZ>(viewer, combinedCloud, "combinedCloud2", 1.0, false, 0.0, 2000.0);
        
        viewer->spin();

        return 0;
        
        grabber.start();
        
        int i = 0;

        while( !viewer->wasStopped() ){
            viewer->spinOnce();
            boost::mutex::scoped_try_lock lock( mutex );
            if( lock.owns_lock() && combinedCloud ){
                myFunction::updateCloud<pcl::PointXYZ>(viewer, combinedCloud, "combinedCloud", 1.0, false, 0.0, 2000.0);

            }
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
