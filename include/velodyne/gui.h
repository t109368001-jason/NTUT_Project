#ifndef VELODYNE_GUI_HPP_
#define VELODYNE_GUI_HPP_
#include <qapplication.h>
#include <qpushbutton.h>
#include <qmainwindow.h>
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
#include <boost/filesystem.hpp>
#include <vtk-6.3/QVTKWidget.h>
#include <vtk-6.3/vtkRenderWindow.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <include/pcap_cache.h>
#include <include/velodyne/function.h>
#include <include/microStopwatch.h>
#include <include/basic_function.h>
#include <include/velodyne/pcaps_gui_plugin.h>
#include <include/velodyne/pcd_gui_plugin.h>
#include <include/velodyne/merge_tool_gui_plugin.h>

namespace velodyne {
    class GUI : public QMainWindow
    {
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef boost::shared_ptr<PointCloudT> PointCloudPtrT;

    static const std::string singlePcapViewerTmpFolder;

    public:
    
        enum Mode {
            PCD_VIEWER = 0,
            PCAP_VIEWER = 1,
            PCAP_MERGE_TOOL = 2,
            COUNT = 3
        };

        QVTKWidget *qvtk;
        QTimer *qvtkTimer;
        QMenu *fileMenu;
        QAction *pcdViewerAction;
        QAction *pcapsViewerAction;
        QAction *mergeToolAction;
        QAction *quit;

        PcapsGUIPlugin *pcapsGUIPlugin;
        PCDGUIPlugin *pcdGUIPlugin;
        MergeToolGUIPlugin *mergeToolGUIPlugin;

        GUI(QWidget *parent = nullptr) {
            setWindowTitle("velydyne gui");
            setWindowState(Qt::WindowMaximized);
            statusBar()->showMessage("Initializing");

            makeMenuBar();
            
            QGridLayout *layout = new QGridLayout;

            qvtk = new QVTKWidget(this);
            layout->addWidget(qvtk, 0,0);

            setCentralWidget(new QWidget);
            centralWidget()->setLayout(layout);

            statusBar()->showMessage("Ready");
        }
        
        void makeMenuBar() {
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

        void run() {
            qvtk->update();
        }

        void openFile(Mode mode) {
            
            if(mode == Mode::PCD_VIEWER) {
                pcdGUIPlugin = new PCDGUIPlugin(this);
                pcdGUIPlugin->set();

                qvtk->SetRenderWindow(pcdGUIPlugin->viewer->getRenderWindow());
                qvtk->setSizePolicy(QSizePolicy::Policy::Preferred, QSizePolicy::Policy::Preferred);
                pcdGUIPlugin->viewer->setupInteractor (qvtk->GetInteractor (), qvtk->GetRenderWindow ());
                qvtk->update ();
            } else if(mode == Mode::PCAP_VIEWER) {
                pcapsGUIPlugin = new PcapsGUIPlugin(this);
                pcapsGUIPlugin->set();

                qvtk->SetRenderWindow(pcapsGUIPlugin->viewer->getRenderWindow());
                qvtk->setSizePolicy(QSizePolicy::Policy::Preferred, QSizePolicy::Policy::Preferred);
                pcapsGUIPlugin->viewer->setupInteractor (qvtk->GetInteractor (), qvtk->GetRenderWindow ());
                qvtk->update ();
                addToolBar(pcapsGUIPlugin->getToolBar());
                
                qvtkTimer = new QTimer(this);
                connect(qvtkTimer, &QTimer::timeout, this, &GUI::run);
                qvtkTimer->start();
            } else if(mode == Mode::PCAP_MERGE_TOOL) {
                mergeToolGUIPlugin = new MergeToolGUIPlugin(this);
                mergeToolGUIPlugin->set();

                qvtk->SetRenderWindow(mergeToolGUIPlugin->viewer->getRenderWindow());
                qvtk->setSizePolicy(QSizePolicy::Policy::Preferred, QSizePolicy::Policy::Preferred);
                mergeToolGUIPlugin->viewer->setupInteractor (qvtk->GetInteractor (), qvtk->GetRenderWindow ());
                qvtk->update ();
                
                qvtkTimer = new QTimer(this);
                connect(qvtkTimer, &QTimer::timeout, this, &GUI::run);
                qvtkTimer->start();
            } 
        }
    };
}

#endif // VELODYNE_GUI_VIEWER_HPP_