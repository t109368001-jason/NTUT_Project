#include <velodyne/gui_pcl_viewer.hpp>
#include <vtkRenderWindow.h>
namespace velodyne {
    typedef pcl::PointXYZ PointT;
    typedef boost::shared_ptr<pcl::PointXYZ> PointPtrT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef boost::shared_ptr<PointCloudT> PointCloudPtrT;
    typedef boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> PointCloudRGBPtrT;
    typedef boost::shared_ptr<PcapCache> PcapCachePtrT;
    typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> ViewerPtrT;

        GUIPCLViewer::GUIPCLViewer(QWidget *parent) : QWidget(parent) {
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

        void GUIPCLViewer::keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
        {
            if((event.getKeySym() == "space")&&(event.keyDown()))
            {
                //playTriggered();
            }
            else 
            {
                std::cout << "Keyboard pressed: " << ((event.isAltPressed())? "Alt + " : "") << ((event.isCtrlPressed())? "Ctrl + " : "") << ((event.isShiftPressed())? "Shift + " : "") << event.getKeySym() << std::endl;
            }
        }

        void GUIPCLViewer::mouseEventOccurred(const pcl::visualization::MouseEvent& event, void* nothing)
        {
            if((event.getButton() == pcl::visualization::MouseEvent::MouseButton::LeftButton)||(event.getButton() == pcl::visualization::MouseEvent::MouseButton::MiddleButton)||(event.getButton() == pcl::visualization::MouseEvent::MouseButton::RightButton)) {
                if(event.getType() == pcl::visualization::MouseEvent::MouseButtonPress) {
                    
                }
            }
        }

        void GUIPCLViewer::pointPickingEventOccurred(const pcl::visualization::PointPickingEvent& event, void* nothing) {
            pcl::PointXYZ point;

            event.getPoint(point.x, point.y, point.z);

            //pickedPoints[pickCount] = point;
            
            std::cout << "Picked : " << point;
            //pickCount = (pickCount + 1) % 2;
            //std::cout << ", distance between last" << pickedPoints[pickCount] <<  " is " << myFunction::distance(pickedPoints[0], pickedPoints[1]) << std::endl;
        }

        void GUIPCLViewer::areaPickingEventOccurred(const pcl::visualization::AreaPickingEvent& event, void* nothing) {
            
        }
}