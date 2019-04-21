#ifndef VELODYNE_PCD_GUI_PLUGIN_HPP_
#define VELODYNE_PCD_GUI_PLUGIN_HPP_
#include <QMenu>
#include <QMenuBar>
#include <QToolBar>
#include <QStatusBar>
#include <QHBoxLayout>
#include <QTimer>
#include <QFileDialog>
#include <QStyle>
#include <QProgressBar>
#include <QSlider>
#include <QObject>
#include <QTextEdit>
#include <QLabel>
#include <QThread>
#include <QMutex>
#include <QTimer>
#include <QWizardPage>
#include <QLineEdit>
#include <QDialog>
#include <QWizard>
#include <QObject>
#include <basic_function.hpp>
namespace velodyne {
    class PCDGUIPlugin : public QWidget {
    public:
        QMainWindow *parent;

        QAction *open;
        
        QToolBar *mediaToolBar;
        QAction *play;
        QLabel *labelMin;
        QSlider *slider;
        QLabel *labelMax;

        pcl::visualization::PCLVisualizer::Ptr viewer;
        PcapCache *pcapCache;
        PointCloudPtrT cloud;

        PCDGUIPlugin(QMainWindow *parent = nullptr) : QWidget(parent), parent(parent) { }


        void set() {
                QStringList filenames = QFileDialog::getOpenFileNames(parent,
                            "Select one or more files to open",
                            QDir::currentPath(),
                            "pcd files (*.pcd)" );
                if( !filenames.isEmpty() )
                {
                    viewer.reset(new pcl::visualization::PCLVisualizer( "Velodyne Viewer", false));
                    viewer->registerKeyboardCallback(&PCDGUIPlugin::keyboardEventOccurred, *this);
                    viewer->registerMouseCallback(&PCDGUIPlugin::mouseEventOccurred, *this);
                    viewer->registerPointPickingCallback(&PCDGUIPlugin::pointPickingEventOccurred, *this);
                    viewer->registerAreaPickingCallback(&PCDGUIPlugin::areaPickingEventOccurred, *this);
                    viewer->addCoordinateSystem( 3.0, "coordinate" );
                    viewer->setCameraPosition( 0.0, 0.0, 4000.0, 0.0, 1.0, 0.0, 0 );

                    std::vector<std::string> pcapFilenames;
                    for (int i =0;i<filenames.count();i++) {
                        PointCloudPtrT cloud(new PointCloudT);
                        pcl::io::loadPCDFile(QFile::encodeName(filenames[i]).toStdString(), *cloud);
                        myFunction::updateCloud(viewer, cloud, "cloud" + std::to_string(i));
                    }

                }
        }

        void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
        {
            if((event.getKeySym() == "space")&&(event.keyDown()))
            {
                
            }
            else 
            {
                std::cout << "Keyboard pressed: " << ((event.isAltPressed())? "Alt + " : "") << ((event.isCtrlPressed())? "Ctrl + " : "") << ((event.isShiftPressed())? "Shift + " : "") << event.getKeySym() << std::endl;
            }
        }

        void mouseEventOccurred(const pcl::visualization::MouseEvent& event, void* nothing)
        {
            if((event.getButton() == pcl::visualization::MouseEvent::MouseButton::LeftButton)||(event.getButton() == pcl::visualization::MouseEvent::MouseButton::MiddleButton)||(event.getButton() == pcl::visualization::MouseEvent::MouseButton::RightButton)) {
                if(event.getType() == pcl::visualization::MouseEvent::MouseButtonPress) {
                    
                }
            }
        }

        void pointPickingEventOccurred(const pcl::visualization::PointPickingEvent& event, void* nothing) {

        }

        void areaPickingEventOccurred(const pcl::visualization::AreaPickingEvent& event, void* nothing) {

        }

    };
}

#endif