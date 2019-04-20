#ifndef VELODYNE_GUI_HPP_
#define VELODYNE_GUI_HPP_
#include <QMainWindow>
#include <QMenu>
#include <QAction>
#include <velodyne/gui_pcl_viewer.hpp>
#include <velodyne/merge_tool_gui_plugin.h>

namespace velodyne {
    class GUI : public QMainWindow
    {
    static const std::string singlePcapViewerTmpFolder;

    public:
    
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

        MergeToolGUIPlugin *mergeToolGUIPlugin;

        GUIPCLViewer *guiViewer;

        GUI(QWidget *parent = nullptr);
        
        void makeMenuBar();

        void openFile(Mode mode);
    };
}

#endif // VELODYNE_GUI_VIEWER_HPP_