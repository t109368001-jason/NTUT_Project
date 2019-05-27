#ifndef VELODYNE_GUI_HPP_
#define VELODYNE_GUI_HPP_
#include <QMainWindow>
#include <QGuiApplication>
#include <QApplication>
#include <QMenuBar>
#include <QAction>
#include <QFileDialog>
#include <QStatusBar>
#include <QStyle>

#include <boost/filesystem.hpp>
#include <vtk-6.3/QVTKWidget.h>
#include <vtk-6.3/vtkRenderWindow.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>

#include <microStopwatch.hpp>
#include <basic_function.hpp>
#include <velodyne/function.hpp>
#include <velodyne/pcap_cache.hpp>
#include <velodyne/gui_pcl_viewer.hpp>
#include <velodyne/merge_tool.hpp>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace velodyne {
    class GUI : public QMainWindow
    {
    public:

        GUI(QWidget *parent = nullptr);
        
    private:
        static const std::string singlePcapViewerTmpFolder;
        enum Mode {
            PCD_VIEWER = 0,
            PCAP_VIEWER_1 = 1,
            PCAP_VIEWER_2 = 2,
            PCAP_MERGE_TOOL = 3,
            COUNT = 4
        };

        QMenu *fileMenu;
        QAction *pcdViewerAction;
        QAction *pcapsViewer1Action;
        QAction *pcapsViewer2Action;
        QAction *mergeToolAction;
        QAction *quit;

        GUIPCLViewer *guiViewer;
        MergeTool *mergeTool;

        void makeMenuBar();

        void openFile(Mode mode);
    };
}

using namespace velodyne;
GUI::GUI(QWidget *parent) : QMainWindow(parent) {

    if(getuid()) throw std::runtime_error( "Please run as root" );

    setWindowTitle("velydyne gui");
    setWindowState(Qt::WindowMaximized);
    statusBar()->showMessage("Initializing");

    makeMenuBar();
    
    QGridLayout *layout = new QGridLayout;

    guiViewer = new GUIPCLViewer(this);
    setCentralWidget(guiViewer);
    
    statusBar()->showMessage("Ready");
}

void GUI::makeMenuBar() {
    pcdViewerAction = new QAction(style()->standardIcon(QStyle::StandardPixmap::SP_DialogOpenButton), "&pcdViewer", this);
    pcapsViewer1Action = new QAction(style()->standardIcon(QStyle::StandardPixmap::SP_DialogOpenButton), "&pcaps n buffer", this);
    pcapsViewer2Action = new QAction(style()->standardIcon(QStyle::StandardPixmap::SP_DialogOpenButton), "&pcaps std", this);
    mergeToolAction = new QAction(style()->standardIcon(QStyle::StandardPixmap::SP_DialogOpenButton), "&mergeTool", this);
    quit = new QAction(style()->standardIcon(QStyle::StandardPixmap::SP_DialogCloseButton), "&Quit", this);
    pcdViewerAction->setShortcut(tr("CTRL+1"));
    pcapsViewer1Action->setShortcut(tr("CTRL+2"));
    pcapsViewer2Action->setShortcut(tr("CTRL+3"));
    mergeToolAction->setShortcut(tr("CTRL+4"));
    quit->setShortcut(tr("CTRL+Q"));

    fileMenu = menuBar()->addMenu("File");
    fileMenu->addAction(pcdViewerAction);
    fileMenu->addAction(pcapsViewer1Action);
    fileMenu->addAction(pcapsViewer2Action);
    fileMenu->addAction(mergeToolAction);
    fileMenu->addSeparator();
    fileMenu->addAction(quit);
    
    connect(quit, &QAction::triggered, qApp, &QApplication::quit);
    connect(pcdViewerAction, &QAction::triggered, std::bind(&GUI::openFile, this, Mode::PCD_VIEWER));
    connect(pcapsViewer1Action, &QAction::triggered, std::bind(&GUI::openFile, this, Mode::PCAP_VIEWER_1));
    connect(pcapsViewer2Action, &QAction::triggered, std::bind(&GUI::openFile, this, Mode::PCAP_VIEWER_2));
    connect(mergeToolAction, &QAction::triggered, std::bind(&GUI::openFile, this, Mode::PCAP_MERGE_TOOL));
}

void GUI::openFile(Mode mode) {
    
    if(mode == Mode::PCD_VIEWER) {
        
        QStringList filenames = QFileDialog::getOpenFileNames(this,
                    "Select one or more files to open",
                    "$HOME",
                    "pcd files (*.pcd)" );
        
        if( !filenames.isEmpty() )
        {
            /*
            std::ofstream ofs("comparison.csv");
            //pcl::io::compression_Profiles_e compressionProfile = pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
            pcl::io::compression_Profiles_e compressionProfile = pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
            //pcl::io::OctreePointCloudCompression<pcl::PointXYZ> PointCloudEncoder(compressionProfile, true);
            pcl::io::OctreePointCloudCompression<pcl::PointXYZ> PointCloudDecoder;
            ofs << "name" << "," << "total points" << "," << "mae_custom" << "," << "origin" << "," << "custom" << std::endl;
            */
            for (int i =0;i<filenames.count();i++) {
                PointCloudPtrT cloud(new PointCloudT);
                pcl::io::loadPCDFile(QFile::encodeName(filenames[i]).toStdString(), *cloud);
                guiViewer->addItem<PointCloudPtrT>(QFile::encodeName(filenames[i]).toStdString(), cloud);
                /*
                boost::filesystem::path origin_file{QFile::encodeName(filenames[i]).toStdString()};
                boost::filesystem::path origin_path{};
                boost::filesystem::path custom_file{origin_file.parent_path().string() + "/../custom/" + origin_file.stem().string() + ".pcd"};
                boost::filesystem::path custom_back{origin_file.parent_path().string() + "/../custom/back_50.000000.pcd"};

                PointCloudPtrT cloud_origin(new PointCloudT);
                PointCloudPtrT cloud_custom(new PointCloudT);
                PointCloudPtrT cloud_custom_back(new PointCloudT);
                
                pcl::io::loadPCDFile(origin_file.string(), *cloud_origin);
                pcl::io::loadPCDFile(custom_file.string(), *cloud_custom);
                pcl::io::loadPCDFile(custom_back.string(), *cloud_custom_back);
                std::cout << cloud_custom->points.size() << " " << cloud_custom_back->points.size() << std::endl;
                *cloud_custom_back += *cloud_custom;
                std::cout << cloud_custom->points.size() << " " << cloud_custom_back->points.size() << std::endl;
                
                std::vector<double> error_custom;
                pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_custom;
                
                kdtree_custom.setInputCloud (cloud_custom_back);

                for(int i = 0; i < cloud_origin->points.size(); i++) {
                    {
                        std::vector<int> pointIdxNKNSearch(1);
                        std::vector<float> pointNKNSquaredDistance(1);
                        if ( kdtree_custom.nearestKSearch (cloud_origin->points[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ) {
                            error_custom.push_back(pointNKNSquaredDistance[0]);
                        } 
                        else {
                            error_custom.push_back(std::sqrt(cloud_origin->points[i].x*cloud_origin->points[i].x+cloud_origin->points[i].y*cloud_origin->points[i].y+cloud_origin->points[i].z*cloud_origin->points[i].z));
                        }
                    }
                }
                
                double mae_custom = 0;
                for(auto &error : error_custom) {
                    mae_custom += std::fabs(error);
                }
                mae_custom /= error_custom.size();
                
                std::cout << origin_file.stem().string() << "," << cloud_origin->points.size() << "," << mae_custom << "," 
                    << boost::filesystem::file_size(origin_file) << ","
                    << boost::filesystem::file_size(custom_file) << std::endl;
                ofs << origin_file.stem().string() << "," << cloud_origin->points.size() << "," << mae_custom << "," 
                    << boost::filesystem::file_size(origin_file) << ","
                    << boost::filesystem::file_size(custom_file) << std::endl;
                    */
            }
            /*
            ofs.close();
            boost::filesystem::permissions("comparison.csv",
                            boost::filesystem::perms::all_all);
                            */
 
        }
    } else if(mode == Mode::PCAP_VIEWER_1) {
        QStringList filenames = QFileDialog::getOpenFileNames(this,
                    "Select one or more files to open",
                    "$HOME",
                    "pcap files (*.pcap)" );

        if( !filenames.isEmpty() )
        {
            std::vector<std::string> pcapFilenames;
            for (int i =0;i<filenames.count();i++) {
                pcapFilenames.push_back(QFile::encodeName(filenames[i]).toStdString());
            }

            double resolution = 50.0;

            boost::filesystem::path outputDir(pcapFilenames[0]);
            outputDir = outputDir.parent_path().string() + "/pcap_cache_data/";
            boost::shared_ptr<PcapCache> pcapCache(new PcapCache(outputDir.string()));
            for(auto pcapFilename : pcapFilenames) {
                pcapCache->addPcap(pcapFilename, "default");
            }    
            pcapCache->setBackgroundResolution(resolution, true);
            pcapCache->setRange(2500, 4499);
            pcapCache->convert();
            guiViewer->addItem<boost::shared_ptr<PcapCache>>("pcaps", pcapCache);
        }
    } else if(mode == Mode::PCAP_VIEWER_2) {
        QStringList filenames = QFileDialog::getOpenFileNames(this,
                    "Select one or more files to open",
                    "$HOME",
                    "pcap files (*.pcap)" );

        if( !filenames.isEmpty() )
        {
            std::vector<std::string> pcapFilenames;
            for (int i =0;i<filenames.count();i++) {
                pcapFilenames.push_back(QFile::encodeName(filenames[i]).toStdString());
            }

            double resolution = 50.0;

            boost::filesystem::path outputDir(pcapFilenames[0]);
            outputDir = outputDir.parent_path().string() + "/pcap_cache_data/";
            boost::shared_ptr<PcapCache> pcapCache(new PcapCache(outputDir.string()));
            for(auto pcapFilename : pcapFilenames) {
                pcapCache->addPcap(pcapFilename, "default");
            }    
            pcapCache->setBackgroundResolution(resolution, false);
            pcapCache->setRange(2500, 4499);
            pcapCache->convert();
            guiViewer->addItem<boost::shared_ptr<PcapCache>>("pcaps", pcapCache);
        }
    } else if(mode == Mode::PCAP_MERGE_TOOL) {
            QStringList filenames = QFileDialog::getOpenFileNames(this,
                        "Select one or more files to open",
                        QDir::currentPath(),
                        "pcap files (*.pcap)" );
            if( !filenames.isEmpty() )
            {

                std::vector<PointCloudPtrT> clouds;
                std::mutex mutex;
                mergeTool = new MergeTool(mutex);
                clouds.resize(filenames.count());

                for (int i =0;i<filenames.count();i++) {
                    clouds[i].reset(new PointCloudT);
                    mergeTool->add(QFile::encodeName(filenames[i]).toStdString(), clouds[i]);
                    guiViewer->addItem<PointCloudPtrT>(std::to_string(i), clouds[i]);
                }

                guiViewer->registerKeyboardCallback(&MergeTool::keyboardEventOccurred, *mergeTool);
            }
    } 
}

#endif // VELODYNE_GUI_VIEWER_HPP_