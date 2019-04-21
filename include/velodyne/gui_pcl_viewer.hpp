#ifndef VELODYNE_GUI_PCL_VIEWER_H_
#define VELODYNE_GUI_PCL_VIEWER_H_
#include <QTimer>
#include <QHBoxLayout>
#include <QFormLayout>
#include <vtk-6.3/QVTKWidget.h>
#include <velodyne/gui_pcl_viewer_item.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>

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

        std::string pickedPointsItemName;
        bool removePickPoint;

        GUIPCLViewer(QWidget *parent);
        
        template<typename _ItemT>
        GUIPCLViewerItem * addItem(std::string name, _ItemT &item);

        void refresh();

        void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing);
        
        template<typename T>
        void registerKeyboardCallback (void (T::*callback) (const pcl::visualization::KeyboardEvent&, void*), T& instance, void* cookie = NULL)
        {
            viewer->registerKeyboardCallback(boost::bind (callback,  boost::ref (instance), _1, cookie));
        }

        void mouseEventOccurred(const pcl::visualization::MouseEvent& event, void* nothing);

        void pointPickingEventOccurred(const pcl::visualization::PointPickingEvent& event, void* nothing);

        void areaPickingEventOccurred(const pcl::visualization::AreaPickingEvent& event, void* nothing);

        GUIPCLViewerItem* getItemPtrByName(std::string name) {
            for(auto &item : items) {
                if(item->getName() == name) return item;
            }
            return nullptr;
        }

    };
}

using namespace velodyne;
GUIPCLViewer::GUIPCLViewer(QWidget *parent) : QWidget(parent), removePickPoint(false), pickedPointsItemName("picked points") {
    QWidget *vLayoutWidget = new QWidget(this);
    hLayout = new QHBoxLayout(this);
    vLayout = new QFormLayout(this);
    qvtk = new QVTKWidget(this);

    this->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
    vLayoutWidget->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Minimum);
    qvtk->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);

    this->setLayout(hLayout);
    vLayoutWidget->setLayout(vLayout);

    hLayout->addWidget(vLayoutWidget);
    hLayout->addWidget(qvtk);

    viewer.reset(new pcl::visualization::PCLVisualizer( "Velodyne Viewer", false));
    viewer->registerKeyboardCallback(&GUIPCLViewer::keyboardEventOccurred, *this);
    viewer->registerMouseCallback(&GUIPCLViewer::mouseEventOccurred, *this);
    viewer->registerPointPickingCallback(&GUIPCLViewer::pointPickingEventOccurred, *this);
    viewer->registerAreaPickingCallback(&GUIPCLViewer::areaPickingEventOccurred, *this);
    viewer->addCoordinateSystem( 3.0, "coordinate" );
    viewer->setCameraPosition( 0.0, 0.0, 4000.0, 0.0, 1.0, 0.0, 0 );
            
    qvtk->SetRenderWindow(this->viewer->getRenderWindow());
    qvtk->setSizePolicy(QSizePolicy::Policy::Preferred, QSizePolicy::Policy::Preferred);
    this->viewer->setupInteractor (qvtk->GetInteractor (), qvtk->GetRenderWindow ());
    qvtk->update ();
        
    timer = new QTimer(this);
}

template<typename _ItemT>
GUIPCLViewerItem * GUIPCLViewer::addItem(std::string name, _ItemT &item) {
    GUIPCLViewerItem *obj;
    obj = new GUIPCLViewerItem(this);
    obj->setItem<_ItemT>(name, item);
    vLayout->addRow(obj);
    if(this->items.size() == 0) {
        connect(timer, &QTimer::timeout, this, &GUIPCLViewer::refresh);
        timer->start(16);
    }
    this->items.push_back(obj);
    return obj;
}

void GUIPCLViewer::refresh() {
    for(auto &item : items) {
        GUIPCLViewerItem::Color color = item->getColor();
        std::string name = item->getName();
        PointCloudPtrT cloud(new PointCloudT);

        cloud = item->getCloud();

        if(color.data == 0) {
            myFunction::updateCloud(viewer, cloud, name, color.r, color.g, color.b);
        } else {
            myFunction::updateCloud(viewer, cloud, name, double(color.data) * 100.0);
        }
    }
    qvtk->update();
}

void GUIPCLViewer::keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing) {
    if((event.getKeySym() == "space")&&(event.keyDown()))
    {
        //playTriggered();
    }
    else if(((event.getKeySym() == "Shift_L")||(event.getKeySym() == "Shift_R"))&&(event.keyUp()))
    {
        removePickPoint = true;
    }
    else 
    {
        std::cout << "Keyboard pressed: " << ((event.isAltPressed())? "Alt + " : "") << ((event.isCtrlPressed())? "Ctrl + " : "") << ((event.isShiftPressed())? "Shift + " : "") << event.getKeySym() << std::endl;
    }
}

void GUIPCLViewer::mouseEventOccurred(const pcl::visualization::MouseEvent& event, void* nothing) {
    if((event.getButton() == pcl::visualization::MouseEvent::MouseButton::LeftButton)||(event.getButton() == pcl::visualization::MouseEvent::MouseButton::MiddleButton)||(event.getButton() == pcl::visualization::MouseEvent::MouseButton::RightButton)) {
        if(event.getType() == pcl::visualization::MouseEvent::MouseButtonPress) {
            
        }
    }
}

void GUIPCLViewer::pointPickingEventOccurred(const pcl::visualization::PointPickingEvent& event, void* nothing) {
    PointPtrT pointPtr(new PointT);

    event.getPoint(pointPtr->x, pointPtr->y, pointPtr->z);
    
    GUIPCLViewerItem *item = getItemPtrByName(pickedPointsItemName);
    if(!item) {
        item = addItem(pickedPointsItemName, pointPtr);
    } else {
        if(removePickPoint) {
            item->resetItem<PointPtrT>(pickedPointsItemName, pointPtr);
            removePickPoint = false;
        } else {
            item->addItem<PointPtrT>(pointPtr);
        }
    }

    PointCloudPtrT cloud(new PointCloudT);
    cloud = item->getCloud();
    if(cloud->points.size() > 1) {
        PointT p1, p2;
        p1 = *(cloud->points.begin());
        p2 = *(cloud->points.end()-1);
        
        double d = std::sqrt((p1.x-p2.x)*(p1.x-p2.x)
        +(p1.y-p2.y)*(p1.y-p2.y)
        +(p1.z-p2.z)*(p1.z-p2.z))/100.0;

        GUIPCLViewerItem::Color color = item->getColor();

        viewer->removeShape("pickedPointsLine");
        viewer->addLine(p1, p2, color.r, color.g, color.b, "pickedPointsLine");
        pickedPointsItemName = "pick points( " + std::to_string(d) + " m)";
        item->resetItem<PointCloudPtrT>(pickedPointsItemName, cloud);
    } else {
        viewer->removeShape("pickedPointsLine");
        pickedPointsItemName = "pick points";
        item->resetItem<PointCloudPtrT>(pickedPointsItemName, cloud);
    }

    //pickedPoints[pickCount] = point;
    
    std::cout << "Picked : " << *pointPtr << std::endl;
    //pickCount = (pickCount + 1) % 2;
    //std::cout << ", distance between last" << pickedPoints[pickCount] <<  " is " << myFunction::distance(pickedPoints[0], pickedPoints[1]) << std::endl;
}

void GUIPCLViewer::areaPickingEventOccurred(const pcl::visualization::AreaPickingEvent& event, void* nothing) {
    
}

#endif