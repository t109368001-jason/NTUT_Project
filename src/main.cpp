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

//#include "../include/velodyne/velodyne.h"
//#include "../include/velodyne/velodyne_grabber.h"
#include "../include/velodyne/pcap_cache.h"
#include "../include/function.h"
//#include "../include/lasers.h"
/*
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
  reg.setMaximumIterations (2);
  for (int i = 0; i < 50; ++i)
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
*/

std::mutex cloudMutex;
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> inputCloud_1;
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> inputCloud_2;

//velodyne::VLP16Grabber grabber;
pcl::visualization::PCLVisualizer::Ptr viewer;
uint64_t startTime;
uint64_t startTime2;

bool viewerPause = false;
bool dynamicObject = false;
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
        boost::filesystem::path pcapPath_1{args::get(inputPcapArg_1)};
        boost::filesystem::path pcapPath_2{args::get(inputPcapArg_2)};
        boost::filesystem::path backgroundPcdPath{args::get(backgroundPcdArg)};

        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud(new pcl::PointCloud<pcl::PointXYZ>);

        boost::shared_ptr<Eigen::Matrix4f> m(new Eigen::Matrix4f);

        m->operator()(0,0) = 0.98370500f ;m->operator()(0,1) = -0.1792490f; m->operator()(0,2) = -0.0138919f; m->operator()(0,3) = -1030.70f;
        m->operator()(1,0) = 0.17952300f ;m->operator()(1,1) = 0.98350800f; m->operator()(1,2) = 0.02196290f; m->operator()(1,3) = -257.209f;
        m->operator()(2,0) = 0.00972594f ;m->operator()(2,1) = -0.0240989f; m->operator()(2,2) = 0.99966200f; m->operator()(2,3) = 56.53390f;
        m->operator()(3,0) = 0.00000000f ;m->operator()(3,1) = 0.00000000f; m->operator()(3,2) = 0.00000000f; m->operator()(3,3) = 1.000000f;
        viewer.reset(new pcl::visualization::PCLVisualizer( "Velodyne Viewer" ));
        viewer->registerKeyboardCallback(&keyboardEventOccurred, (void*) NULL);
        viewer->registerMouseCallback(&mouseEventOccurred, (void*) NULL);
        viewer->addCoordinateSystem( 3.0, "coordinate" );
        viewer->setCameraPosition( 0.0, 0.0, 1000.0, 0.0, 1.0, 0.0, 0 );
        
        velodyne::PcapCache<pcl::PointXYZ> pcapCache("/tmp/pcapCache/");
        if(!pcapCache.add(pcapPath_1.string())) return 0;
        if(!pcapCache.add(pcapPath_2.string(), 2, m)) return 0;
        tt1.tic();
        if(!pcapCache.convert()) {
          std::cout << "pcap cahce convert error" << std::endl;
          return 0;
        }
        tt1.toc_print_string();


        std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> combineCloud;
        std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> backCloud;
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> back(new pcl::PointCloud<pcl::PointXYZ>());
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> temp(new pcl::PointCloud<pcl::PointXYZ>());
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> temp1(new pcl::PointCloud<pcl::PointXYZ>());
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> combineTemp(new pcl::PointCloud<pcl::PointXYZ>());
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> combineTemp1(new pcl::PointCloud<pcl::PointXYZ>());

        int offsetFrame = 400;
        int totalFrame = 5380 - offsetFrame;
        int compareFrameNumber = 100;
        int times = 4;


        for(int i = 0; i < (compareFrameNumber*times); i++)
        {
            combineCloud.push_back(pcapCache.get(offsetFrame + i*totalFrame/compareFrameNumber/times));
        }
        std::cout<< combineCloud.size() << std::endl;

        for(int j = 0; j < times; j++)
        {
            temp = combineCloud[j];
            temp1 = combineCloud[j];
            for(int i = (j + times); i < combineCloud.size(); i = i + times)
            {
                temp = myFunction::getNoChanges<pcl::PointXYZ>(temp, combineCloud[i], 0.1);
                temp1 = myFunction::getNoChanges<pcl::PointXYZ>(combineCloud[i], temp1, 0.1);
            }

            combineTemp = myFunction::getNoChanges<pcl::PointXYZ>(temp, temp1, 0.1);
            combineTemp1 = myFunction::getNoChanges<pcl::PointXYZ>(temp1, temp, 0.1);

            *back = *combineTemp + *combineTemp1;
            backCloud.push_back(back);
        }

        back = backCloud[0];

        for(int i = 1; i < backCloud.size(); i++)
        {
            back = myFunction::getNoChanges<pcl::PointXYZ>(back, backCloud[i], 0.1);
        }
        
        if(back->points.size() == 0) back = backCloud[0];

        std::cout<< back->points.size() << std::endl;

        uint64_t displayFrameIndex = 0;
        while( !viewer->wasStopped() ){
            viewer->spinOnce();
            //cloud = back;
            ///
            if(dynamicObject == true)
              cloud = myFunction::getChanges<pcl::PointXYZ>(back, combineCloud[displayFrameIndex], 50.0);
            else 
              cloud = combineCloud[displayFrameIndex];

            if(++displayFrameIndex >= combineCloud.size()) {
              displayFrameIndex = 0;
            }

            if(cloud->points.size() == 0) continue;
            //*///
            myFunction::updateCloud<pcl::PointXYZ>(viewer, cloud, "cloud", 1.0, false, 0.0, 2000.0);
            
        }
/* 
        uint64_t displayFrameIndex = 0;
        while( !viewer->wasStopped() ){
            viewer->spinOnce();
            cloud = pcapCache.get(displayFrameIndex);
            myFunction::updateCloud<pcl::PointXYZ>(viewer, cloud, "cloud", 1.0, false, 0.0, 2000.0);
            if(++displayFrameIndex >= pcapCache.totalFrame) {
              displayFrameIndex=0;
            }
        }
*/
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
    else 
    {
        std::cout << "Keyboard pressed: " << ((event.isAltPressed())? "Alt + " : "") << ((event.isCtrlPressed())? "Ctrl + " : "") << ((event.isShiftPressed())? "Shift + " : "") << event.getKeySym() << std::endl;
    }
}
