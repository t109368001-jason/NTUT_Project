#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <thread>
#include <pcl/io/vlp_grabber.h>

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;

PCLVisualizer::Ptr viewer;
std::string filename;
uint64_t startTime;
uint64_t startTime2;

void keyboardEventOccurred(const KeyboardEvent& event, void* nothing)
{
  if (event.getKeySym() == "y" && event.keyDown()) // Flat shading
  {
    viewer->setShapeRenderingProperties(PCL_VISUALIZER_SHADING,
                                        PCL_VISUALIZER_SHADING_FLAT, filename);
  }
  else if (event.getKeySym() == "t" && event.keyDown()) // Gouraud shading
  {
    viewer->setShapeRenderingProperties(PCL_VISUALIZER_SHADING,
                                        PCL_VISUALIZER_SHADING_GOURAUD, filename);
  }
  else if (event.getKeySym() == "n" && event.keyDown()) // Phong shading
  {
    viewer->setShapeRenderingProperties(PCL_VISUALIZER_SHADING,
                                        PCL_VISUALIZER_SHADING_PHONG, filename);
  }
}

void poission_surface(std::string str, pcl::PolygonMesh &poission)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;
	pcl::io::loadPCDFile(str, cloud_blob);
	pcl::fromPCLPointCloud2(cloud_blob, *cloud);

	pcl::CentroidPoint<pcl::PointXYZ> centroid;
	pcl::PointXYZ cent;
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		centroid.add(cloud->points[i]);
	}
	centroid.get(cent);
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		cloud->points[i].x -= cent.x;
		cloud->points[i].y -= cent.y;
		cloud->points[i].z -= cent.z;
	}

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(30);
	n.compute(*normals);

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	pcl::Poisson<pcl::PointNormal> poisson;
	poisson.setDepth(8);
	poisson.setSolverDivide(8);
	poisson.setIsoDivide(8);
	poisson.setPointWeight(4.0f);
	poisson.setInputCloud(cloud_with_normals);

	poisson.reconstruct(poission);
}

void timer1()
{
	while(true)
	{
		if((clock() - startTime) > 1000000)
		{
			startTime = clock();
			std::cout << filename << std::endl;
		}
	}
}

typedef pcl::PointXYZI PointType;

void pcl_viewer()
{/*
    pcl::PolygonMesh mesh;
    poission_surface(filename, mesh);
    //pcl::io::loadPolygonFileSTL(argv[stl_file_indices[0]], mesh);

    viewer.reset(new PCLVisualizer);
    viewer->registerKeyboardCallback(&keyboardEventOccurred, (void*) NULL);
    viewer->addPolygonMesh(mesh, filename);
    //viewer->addPointCloud(cloud, filename);
    viewer->spin();
*/

    // Point Cloud
    pcl::PointCloud<PointType>::ConstPtr cloud;
    // PCL Visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer( "Velodyne Viewer" ) );
    viewer->addCoordinateSystem( 3.0, "coordinate" );
    viewer->setBackgroundColor( 0.0, 0.0, 0.0, 0 );
    viewer->initCameraParameters();
    viewer->setCameraPosition( 0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0 );

    pcl::visualization::PointCloudColorHandler<PointType>::Ptr handler;

        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerGenericField<PointType>( "intensity" ) );
        handler = color_handler;

    // Retrieved Point Cloud Callback Function
    boost::mutex mutex;
    boost::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> function =
        [ &cloud, &mutex ]( const pcl::PointCloud<PointType>::ConstPtr& ptr ){
            boost::mutex::scoped_lock lock( mutex );

            cloud = ptr;
        };// VLP Grabber
    boost::shared_ptr<pcl::VLPGrabber> grabber;
        std::cout << "Capture from PCAP..." << std::endl;
        grabber = boost::shared_ptr<pcl::VLPGrabber>( new pcl::VLPGrabber( filename ) );

    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback( function );

    // Start Grabber
    grabber->start();

    while( !viewer->wasStopped() ){
        // Update Viewer
        viewer->spinOnce();

        boost::mutex::scoped_try_lock lock( mutex );
        if( lock.owns_lock() && cloud ){
            // Update Point Cloud
            handler->setInputCloud( cloud );
            if( !viewer->updatePointCloud( cloud, *handler, "cloud" ) ){
                viewer->addPointCloud( cloud, *handler, "cloud" );
            }
        }
    }

    // Stop Grabber
    grabber->stop();

    // Disconnect Callback Function
    if( connection.connected() ){
        connection.disconnect();
    }
}

int main(int argc, char * argv[])
{
  std::vector<int> file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcap");
  if (file_indices.empty())
  {
    PCL_ERROR ("Please provide file as argument\n");
    return 1;
  }
  filename = argv[file_indices[0]];

  std::thread t1(timer1);
  std::thread t2(pcl_viewer);
  t1.join();
  t2.join();
	while(1);
  return 0;
}
