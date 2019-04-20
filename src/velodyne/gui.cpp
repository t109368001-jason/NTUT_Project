#include <velodyne/gui.h>
#include <QGuiApplication>
#include <QApplication>
#include <QFileDialog>
#include <QStyle>
#include <boost/filesystem.hpp>
#include <vtk-6.3/QVTKWidget.h>
#include <vtk-6.3/vtkRenderWindow.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>
#include <velodyne/pcap_cache.h>
#include <velodyne/function.h>
#include <microStopwatch.hpp>
#include <basic_function.hpp>

namespace velodyne {
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef boost::shared_ptr<PointCloudT> PointCloudPtrT;
        GUI::GUI(QWidget *parent) : QMainWindow(parent) {
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
            pcapsViewerAction = new QAction(style()->standardIcon(QStyle::StandardPixmap::SP_DialogOpenButton), "&pcapsViewer", this);
            mergeToolAction = new QAction(style()->standardIcon(QStyle::StandardPixmap::SP_DialogOpenButton), "&mergeTool", this);
            quit = new QAction(style()->standardIcon(QStyle::StandardPixmap::SP_DialogCloseButton), "&Quit", this);
            pcdViewerAction->setShortcut(tr("CTRL+1"));
            pcapsViewerAction->setShortcut(tr("CTRL+2"));
            mergeToolAction->setShortcut(tr("CTRL+3"));
            quit->setShortcut(tr("CTRL+Q"));

            fileMenu = menuBar()->addMenu("File");
            fileMenu->addAction(pcdViewerAction);
            fileMenu->addAction(pcapsViewerAction);
            fileMenu->addAction(mergeToolAction);
            fileMenu->addSeparator();
            fileMenu->addAction(quit);
            
            connect(quit, &QAction::triggered, qApp, &QApplication::quit);
            connect(pcdViewerAction, &QAction::triggered, std::bind(&GUI::openFile, this, Mode::PCD_VIEWER));
            connect(pcapsViewerAction, &QAction::triggered, std::bind(&GUI::openFile, this, Mode::PCAP_VIEWER));
            connect(mergeToolAction, &QAction::triggered, std::bind(&GUI::openFile, this, Mode::PCAP_MERGE_TOOL));
        }

        void GUI::openFile(Mode mode) {
            
            if(mode == Mode::PCD_VIEWER) {
                
                QStringList filenames = QFileDialog::getOpenFileNames(this,
                            "Select one or more files to open",
                            QDir::currentPath(),
                            "pcd files (*.pcd)" );
                
                if( !filenames.isEmpty() )
                {
                    std::vector<std::string> pcapFilenames;
                    for (int i =0;i<filenames.count();i++) {
                        PointCloudPtrT cloud(new PointCloudT);
                        pcl::io::loadPCDFile(QFile::encodeName(filenames[i]).toStdString(), *cloud);
                        guiViewer->addItem<PointCloudPtrT>(QFile::encodeName(filenames[i]).toStdString(), cloud);
                    }
                }
            } else if(mode == Mode::PCAP_VIEWER) {
                QStringList filenames = QFileDialog::getOpenFileNames(this,
                            "Select one or more files to open",
                            QDir::currentPath(),
                            "pcap files (*.pcap)" );

                if( !filenames.isEmpty() )
                {
                    std::vector<std::string> pcapFilenames;
                    for (int i =0;i<filenames.count();i++) {
                        pcapFilenames.push_back(QFile::encodeName(filenames[i]).toStdString());
                    }

                    int totalFrame = std::numeric_limits<int>::max();
                    
                    boost::shared_ptr<PcapCache> pcapCache(new PcapCache("/home/xian-jie/Downloads/tmp/"));
                    for(auto pcapFilename : pcapFilenames) {
                        totalFrame = std::min(getPcapTotalFrame(pcapFilename), totalFrame);
                        pcapCache->add(pcapFilename, "default");
                    }    

                    pcapCache->convert();

                    guiViewer->addItem<boost::shared_ptr<PcapCache>>("pcaps", pcapCache);
                }
            } else if(mode == Mode::PCAP_MERGE_TOOL) {
                mergeToolGUIPlugin = new MergeToolGUIPlugin(this);
                mergeToolGUIPlugin->set();
            } 
        }
}
