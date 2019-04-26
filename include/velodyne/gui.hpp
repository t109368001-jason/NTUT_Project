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

namespace velodyne {
    class GUI : public QMainWindow
    {
    public:

        GUI(QWidget *parent = nullptr);
        
    private:
        static const std::string singlePcapViewerTmpFolder;
        enum Mode {
            PCD_VIEWER = 0,
            PCAP_VIEWER = 1,
            PCAP_MERGE_TOOL = 2,
            COUNT = 3
        };

        QMenu *fileMenu;
        QAction *pcdViewerAction;
        QAction *pcapsViewerAction;
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
                    "$HOME",
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
                    "$HOME",
                    "pcap files (*.pcap)" );

        if( !filenames.isEmpty() )
        {
            std::vector<std::string> pcapFilenames;
            for (int i =0;i<filenames.count();i++) {
                pcapFilenames.push_back(QFile::encodeName(filenames[i]).toStdString());
            }

            int compareFrameNumber = 100;
            double resolution = 10.0;
            int backNumber = 1000000;

            boost::filesystem::path outputDir(pcapFilenames[0]);
            outputDir = outputDir.parent_path().string() + "/pcap_cache_data/";
            boost::shared_ptr<PcapCache> pcapCache(new PcapCache(outputDir.string()));
            for(auto pcapFilename : pcapFilenames) {
                pcapCache->add(pcapFilename, "default");
            }    
            pcapCache->addBack(backNumber, compareFrameNumber, resolution);
            pcapCache->convert();
            //pcapCache->showback(false);

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