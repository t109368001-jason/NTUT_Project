#ifndef VELODYNE_GUI_PCL_VIEWER_H_
#define VELODYNE_GUI_PCL_VIEWER_H_
#include <QTimer>
#include <QHBoxLayout>
#include <QFormLayout>
#include <vtk-6.3/QVTKWidget.h>
#include <velodyne/gui_pcl_viewer_item.hpp>
#include <pcl/visualization/pcl_visualizer.h>

namespace velodyne {
    typedef pcl::PointXYZ PointT;
    typedef boost::shared_ptr<pcl::PointXYZ> PointPtrT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef boost::shared_ptr<PointCloudT> PointCloudPtrT;
    typedef boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> PointCloudRGBPtrT;
    typedef boost::shared_ptr<PcapCache> PcapCachePtrT;
    typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> ViewerPtrT;

    class GUIPCLViewer : public QWidget {
    public:
        ViewerPtrT viewer;

        QTimer *timer;
        QHBoxLayout *hLayout;
        QFormLayout *vLayout;
        QVTKWidget *qvtk;

        std::vector<GUIPCLViewerItem*> items;

        GUIPCLViewer(QWidget *parent);
        
        template<typename _ItemT>
        void addItem(std::string name, _ItemT &item) {
            GUIPCLViewerItem *obj;
            obj = new GUIPCLViewerItem(this);
            obj->addItem<_ItemT>(name, item);
            vLayout->addRow(obj);
            if(this->items.size() == 0) {
                connect(timer, &QTimer::timeout, this, &GUIPCLViewer::refresh);
                timer->start();
            }
            this->items.push_back(obj);
        }

        void refresh();

        void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing);

        void mouseEventOccurred(const pcl::visualization::MouseEvent& event, void* nothing);

        void pointPickingEventOccurred(const pcl::visualization::PointPickingEvent& event, void* nothing);

        void areaPickingEventOccurred(const pcl::visualization::AreaPickingEvent& event, void* nothing);

    };
}
#endif