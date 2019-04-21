#ifndef VELODYNE_MERGE_TOOL_GUI_PLUGIN_HPP_
#define VELODYNE_MERGE_TOOL_GUI_PLUGIN_HPP_
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
#include <velodyne/merge_tool.hpp>
namespace velodyne {
    class MergeToolGUIPlugin : public QWidget {
    public:
        QAction *open;
        
        QToolBar *mediaToolBar;
        QAction *play;
        QLabel *labelMin;
        QSlider *slider;
        QLabel *labelMax;

        QTimer *viewerTimer;
        std::mutex viewerMutex;
        bool viewerPause;
        int64_t currentCloudIdx;

        pcl::visualization::PCLVisualizer::Ptr viewer;
        MergeTool *mergeTool;
        std::vector<PointCloudPtrT> clouds;

        MergeToolGUIPlugin(QMainWindow *parent = nullptr) : QWidget(parent), currentCloudIdx(0), viewerPause(true) { }

        void set() {
                QStringList filenames = QFileDialog::getOpenFileNames(this,
                            "Select one or more files to open",
                            QDir::currentPath(),
                            "pcap files (*.pcap)" );
                if( !filenames.isEmpty() )
                {
                    mergeTool = new MergeTool(viewerMutex);
                    clouds.resize(filenames.count());

                    for (int i =0;i<filenames.count();i++) {
                        clouds[i].reset(new PointCloudT);
                        mergeTool->add(QFile::encodeName(filenames[i]).toStdString(), clouds[i]);
                    }

                    viewer.reset(new pcl::visualization::PCLVisualizer( "Velodyne Viewer", false));
                    viewer->registerKeyboardCallback(&MergeTool::keyboardEventOccurred, *mergeTool);
                    viewer->registerMouseCallback(&MergeToolGUIPlugin::mouseEventOccurred, *this);
                    viewer->registerPointPickingCallback(&MergeToolGUIPlugin::pointPickingEventOccurred, *this);
                    viewer->registerAreaPickingCallback(&MergeToolGUIPlugin::areaPickingEventOccurred, *this);
                    viewer->addCoordinateSystem( 3.0, "coordinate" );
                    viewer->setCameraPosition( 0.0, 0.0, 4000.0, 0.0, 1.0, 0.0, 0 );

                    viewerTimer = new QTimer(this);
                    connect(viewerTimer, &QTimer::timeout, this, &MergeToolGUIPlugin::run);
                    viewerTimer->start(100);
                }
        }

        void run() {
            for(int i = 0; i < clouds.size(); i++) {
                uint8_t r, g, b;
                myFunction::valueToRGB(r, g, b, float(i)/float(clouds.size()-1));
                myFunction::updateCloud(viewer, clouds[i], "cloud" + std::to_string(i), r, g, b);
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
            std::lock_guard<std::mutex> guard(viewerMutex);
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