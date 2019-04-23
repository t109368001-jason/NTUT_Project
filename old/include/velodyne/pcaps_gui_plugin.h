#ifndef VELODYNE_PCAPS_GUI_PLUGIN_HPP_
#define VELODYNE_PCAPS_GUI_PLUGIN_HPP_
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
    class PcapsGUIPlugin : public QWidget {
    public:
        QMainWindow *parent;

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
        PcapCache *pcapCache;
        PointCloudPtrT cloud;

        std::vector<pcl::PointXYZ> pickedPoints;
        int pickCount;

        PcapsGUIPlugin(QMainWindow *parent = nullptr) : QWidget(parent), parent(parent), currentCloudIdx(0), viewerPause(true), pickedPoints(2), pickCount(0) { }

        int setCurrentCloudIdx_blocking(int currentCloudIdx, QWidget* obj = nullptr) {
            while(setCurrentCloudIdx(currentCloudIdx, obj) != 0);
        }
        int setCurrentCloudIdx(int currentCloudIdx, QWidget* obj = nullptr) {
            int result = 0;
            if(currentCloudIdx >= pcapCache->totalFrame) return 1;
            try {
                if(viewerMutex.try_lock()) {
                    cloud = pcapCache->get(currentCloudIdx);
                    myFunction::updateCloud(viewer, cloud, "cloud", 4000.0);
                    viewerMutex.unlock();
                    this->currentCloudIdx = currentCloudIdx;
                    bool sliderOldState = slider->blockSignals(true);
                    if(obj != slider) slider->setValue(currentCloudIdx);
                    slider->blockSignals(sliderOldState);
                } else {
                    result = 2;
                }
            } catch(std::exception e) {
                result = 3;
            }
            
            return result;
        }

        void run() {
            if(!viewerPause) {
                if(setCurrentCloudIdx(currentCloudIdx+1) == 1) {
                    setCurrentCloudIdx(0);
                }
            }
        }

        void set() {
                QStringList filenames = QFileDialog::getOpenFileNames(parent,
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
                    
                    pcapCache = new PcapCache;
                    for(auto pcapFilename : pcapFilenames) {
                        totalFrame = std::min(getPcapTotalFrame(pcapFilename), totalFrame);
                        pcapCache->add(pcapFilename, "default");
                    }    

                    pcapCache->convert();

                    viewer.reset(new pcl::visualization::PCLVisualizer( "Velodyne Viewer", false));
                    viewer->registerKeyboardCallback(&PcapsGUIPlugin::keyboardEventOccurred, *this);
                    viewer->registerMouseCallback(&PcapsGUIPlugin::mouseEventOccurred, *this);
                    viewer->registerPointPickingCallback(&PcapsGUIPlugin::pointPickingEventOccurred, *this);
                    viewer->registerAreaPickingCallback(&PcapsGUIPlugin::areaPickingEventOccurred, *this);
                    viewer->addCoordinateSystem( 3.0, "coordinate" );
                    viewer->setCameraPosition( 0.0, 0.0, 4000.0, 0.0, 1.0, 0.0, 0 );
                    
                    initialMediaToolBar();

                    setCurrentCloudIdx(0);
                    viewerTimer = new QTimer(this);
                    connect(viewerTimer, &QTimer::timeout, this, &PcapsGUIPlugin::run);
                    viewerTimer->start(100);
                }
        }

        QToolBar *getToolBar() {
            return mediaToolBar;
        }

        void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
        {
            if((event.getKeySym() == "space")&&(event.keyDown()))
            {
                playTriggered();
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
            pcl::PointXYZ point;

            event.getPoint(point.x, point.y, point.z);

            pickedPoints[pickCount] = point;
            
            std::cout << "Picked : " << pickedPoints[pickCount];
            pickCount = (pickCount + 1) % 2;
            std::cout << ", distance between last" << pickedPoints[pickCount] <<  " is " << myFunction::distance(pickedPoints[0], pickedPoints[1]) << std::endl;
        }

        void areaPickingEventOccurred(const pcl::visualization::AreaPickingEvent& event, void* nothing) {
            
        }

        void initialMediaToolBar() {
            mediaToolBar = new QToolBar;
            play = new QAction(parent->style()->standardIcon(QStyle::StandardPixmap::SP_MediaPlay), "&Play");
            labelMin = new QLabel;
            slider = new QSlider(Qt::Orientation::Horizontal);
            labelMax = new QLabel;
            labelMin->setText(std::to_string(pcapCache->beg).c_str());
            labelMax->setText(std::to_string(pcapCache->totalFrame-1).c_str());
            slider->setRange(pcapCache->beg, pcapCache->totalFrame-1);
            slider->setFixedWidth(100);
            mediaToolBar->addAction(play);
            mediaToolBar->addWidget(labelMin);
            mediaToolBar->addWidget(slider);
            mediaToolBar->addWidget(labelMax);
            connect(play, &QAction::triggered, this, &PcapsGUIPlugin::playTriggered);
            connect(slider, &QSlider::valueChanged, this, &PcapsGUIPlugin::sliderValueChanged);
        }


        void playTriggered() {
            viewerPause = !viewerPause;
            if(viewerPause) {
                play->setIcon(parent->style()->standardIcon(QStyle::StandardPixmap::SP_MediaPlay));
            } else {
                play->setIcon(parent->style()->standardIcon(QStyle::StandardPixmap::SP_MediaPause));
            }
        }

        void sliderValueChanged(int value) {
            setCurrentCloudIdx_blocking(slider->value(), slider);
        }

    };
}

#endif