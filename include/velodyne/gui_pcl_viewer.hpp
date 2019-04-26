#ifndef VELODYNE_GUI_PCL_VIEWER_H_
#define VELODYNE_GUI_PCL_VIEWER_H_
#include <QTimer>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QScrollArea>
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
        QScrollArea *scrollArea;
        QWidget *vLayoutWidget;
        QFormLayout *vLayout;
        QVTKWidget *qvtk;

        std::vector<GUIPCLViewerItem*> items;

        int pickedPointsCount;
        bool newpickedPointsItem;
        std::string pickedPointsItemName;

        GUIPCLViewer(QWidget *parent);
        
        template<typename _ItemT>
        GUIPCLViewerItem * addItem(std::string name, _ItemT &item);

        void refresh();

        void deleteItem(std::string deleteName);

        void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing);
        
        template<typename T>
        void registerKeyboardCallback (void (T::*callback) (const pcl::visualization::KeyboardEvent&, void*), T& instance, void* cookie = NULL);

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
GUIPCLViewer::GUIPCLViewer(QWidget *parent) : QWidget(parent), pickedPointsCount(0), newpickedPointsItem(true) {
    scrollArea = new QScrollArea(this);
    vLayoutWidget = new QWidget(this);
    hLayout = new QHBoxLayout(this);
    vLayout = new QFormLayout(vLayoutWidget);
    qvtk = new QVTKWidget(this);

    this->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
    vLayoutWidget->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
    qvtk->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
    scrollArea->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);

    scrollArea->setWidget(vLayoutWidget);
    scrollArea->setWidgetResizable(true);

    hLayout->addWidget(scrollArea, 20);
    hLayout->addWidget(qvtk, 80);

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

    this->setLayout(hLayout);
}



template<typename T>
void GUIPCLViewer::registerKeyboardCallback (void (T::*callback) (const pcl::visualization::KeyboardEvent&, void*), T& instance, void* cookie)
{
    viewer->registerKeyboardCallback(boost::bind (callback,  boost::ref (instance), _1, cookie));
}

template<typename _ItemT>
GUIPCLViewerItem * GUIPCLViewer::addItem(std::string name, _ItemT &item) {
    GUIPCLViewerItem *obj;
    QPushButton *delItem = new QPushButton(style()->standardIcon(QStyle::StandardPixmap::SP_DockWidgetCloseButton), "", this);
    obj = new GUIPCLViewerItem(vLayoutWidget);
    obj->setItem<_ItemT>(name, item);
    vLayout->addRow(obj, delItem);
    delItem->setFixedSize(20,20);
    connect(delItem, &QPushButton::clicked, std::bind(&GUIPCLViewer::deleteItem, this, name));
    if(this->items.size() == 0) {
        connect(timer, &QTimer::timeout, this, &GUIPCLViewer::refresh);
        timer->start(16);
    }
    this->items.push_back(obj);
    return obj;
}

void GUIPCLViewer::deleteItem(std::string deleteName) {
    for(int i = 0; i < items.size(); i++) {
        std::string name = items[i]->getName();
        if(deleteName == name.substr(0, deleteName.size())) {
            viewer->removePointCloud(name);
            viewer->removeShape(name);
            items.erase(items.begin() + i);
            vLayout->removeRow(i);
            break;
        }
    }
    qvtk->update();
}

void GUIPCLViewer::refresh() {
    for(auto &item : items) {
        item->updateView(viewer);
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
        if(!newpickedPointsItem) pickedPointsCount++;
        newpickedPointsItem = true;
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
    PointT p;

    event.getPoint(p.x, p.y, p.z);

    if(p.x == 0 && p.y == 0 && p.z == 0) return;

    std::vector<PointT> points;
    points.push_back(p);
    if(newpickedPointsItem) {
        pickedPointsItemName = "picked points " + std::to_string(pickedPointsCount);
        addItem<std::vector<PointT>>(pickedPointsItemName, points);
        newpickedPointsItem = false;
    } else {
        GUIPCLViewerItem *item = getItemPtrByName(pickedPointsItemName);
        std::cout << item << std::endl;
        item->addItem<std::vector<PointT>>(points);
    }

    std::cout << "Picked : " << p << std::endl;
}

void GUIPCLViewer::areaPickingEventOccurred(const pcl::visualization::AreaPickingEvent& event, void* nothing) {
    
}

#endif