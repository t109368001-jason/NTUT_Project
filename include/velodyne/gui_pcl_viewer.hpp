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
        myClass::MicroStopwatch tt1;
        myClass::MicroStopwatch tt2;
        myClass::MicroStopwatch tt3;
        myClass::MicroStopwatch tt4;
        myClass::MicroStopwatch tt5;

        ViewerPtrT viewer;
        QTimer *timer;
        QHBoxLayout *hLayout;
        QWidget *vLayoutWidget;
        QFormLayout *vLayout;
        QVTKWidget *qvtk;

        std::vector<GUIPCLViewerItem*> items;

        int pickPointsCount;
        std::vector<std::string> pickedPointsItemNames;

        GUIPCLViewer(QWidget *parent);
        
        template<typename _ItemT>
        GUIPCLViewerItem * addItem(std::string name, _ItemT &item);

        void refresh();

        void deleteItem(std::string deleteName);

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
GUIPCLViewer::GUIPCLViewer(QWidget *parent) : QWidget(parent), pickPointsCount(0), tt1("refresh"), tt2("refresh get"), tt3("refresh show") {
    vLayoutWidget = new QWidget(this);
    hLayout = new QHBoxLayout(this);
    vLayout = new QFormLayout;
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
    QPushButton *delItem = new QPushButton(style()->standardIcon(QStyle::StandardPixmap::SP_DockWidgetCloseButton), "", this);
    obj = new GUIPCLViewerItem(vLayoutWidget);
    obj->setItem<_ItemT>(name, item);
    vLayout->addRow(obj, delItem);
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
            viewer->removeShape(name+" line");
            items.erase(items.begin() + i);
            vLayout->removeRow(i);
            break;
        }
    }
    qvtk->update();
}

void GUIPCLViewer::refresh() {
    for(auto &item : items) {
        if(!item) continue;
        PointCloudPtrT cloud(new PointCloudT);
        std::string name = item->getName();
        cloud = item->getCloud();
        if(item->isVisable()) {
            GUIPCLViewerItem::Color color = item->getColor();
            if(color.data == 0) {
                myFunction::updateCloud(viewer, cloud, name, color.r, color.g, color.b);
            } else {
                myFunction::updateCloud(viewer, cloud, name, double(color.data) * 100.0);
            }
            if(viewer->removeShape(name+" line")) {
                PointT p1, p2;
                p1 = *(cloud->points.begin());
                p2 = *(cloud->points.end()-1);
                viewer->addLine(p1, p2, double(color.r)/255.0, double(color.g)/255.0, double(color.b)/255.0, item->getName()+" line");
            }
        } else {
            viewer->removePointCloud(name);
            if(viewer->removeShape(name+" line")) {
                PointT p1, p2;
                p1 = *(cloud->points.begin());
                p2 = *(cloud->points.end()-1);
                viewer->addLine(p1, p2, 0.0, 0.0, 0.0, item->getName()+" line");
            }
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
        if(pickedPointsItemNames.size() > pickPointsCount) pickPointsCount++;
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
    GUIPCLViewerItem *item = nullptr;
    if(pickedPointsItemNames.size() > pickPointsCount) {
        item = getItemPtrByName(pickedPointsItemNames[pickPointsCount]);
        item->addItem<PointPtrT>(pointPtr);
    } else {
        pickedPointsItemNames.push_back("picked points " + std::to_string(pickPointsCount) + " ");
        item = addItem("picked points " + std::to_string(pickPointsCount) + " ", pointPtr);
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

        viewer->removePointCloud(pickedPointsItemNames[pickPointsCount]);
        viewer->removeShape(pickedPointsItemNames[pickPointsCount]+" line");
        pickedPointsItemNames[pickPointsCount] = "picked points " + std::to_string(pickPointsCount) + " ( " + std::to_string(d) + " m)";
        viewer->addLine(p1, p2, double(color.r)/255.0, double(color.g)/255.0, double(color.b)/255.0, pickedPointsItemNames[pickPointsCount]+" line");
        item->resetItem<PointCloudPtrT>(pickedPointsItemNames[pickPointsCount], cloud);
    }

    //pickedPoints[pickCount] = point;
    
    std::cout << "Picked : " << *pointPtr << std::endl;
    //pickCount = (pickCount + 1) % 2;
    //std::cout << ", distance between last" << pickedPoints[pickCount] <<  " is " << myFunction::distance(pickedPoints[0], pickedPoints[1]) << std::endl;
}

void GUIPCLViewer::areaPickingEventOccurred(const pcl::visualization::AreaPickingEvent& event, void* nothing) {
    
}

#endif