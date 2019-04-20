#include <velodyne/gui_pcl_viewer_item.hpp>
#include <QColorDialog>
namespace velodyne {
    typedef pcl::PointXYZ PointT;
    typedef boost::shared_ptr<pcl::PointXYZ> PointPtrT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef boost::shared_ptr<PointCloudT> PointCloudPtrT;
    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGBT;
    typedef boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> PointCloudRGBPtrT;
    typedef boost::shared_ptr<PcapCache> PcapCachePtrT;
    typedef boost::variant<PointPtrT, PointCloudPtrT, PcapCachePtrT> GUIPCLViewerItemT;
        GUIPCLViewerItem::GUIPCLViewerItem(QWidget *parent) : QWidget(parent), color(Color{0xff, 0xff, 0xff, 0x00}) {
            layout = new QGridLayout(this);
            nameLabel = new QLabel(this);
            colorButton = new QPushButton(this);
            nameLabel->setFixedHeight(20);
            colorButton->setFixedHeight(20);
            layout->addWidget(nameLabel, 0, 0);
            layout->addWidget(colorButton, 0, 1);
            adjustSize();
        }

        void GUIPCLViewerItem::colorButtonUpdateColor() {
            QPalette pal = colorButton->palette();
            pal.setColor(QPalette::Button, QColor(color.r, color.g, color.b));
            colorButton->setAutoFillBackground(true);
            colorButton->setPalette(pal);
            colorButton->update();
        }

        void GUIPCLViewerItem::colorButtonClicked() {
            QColor newColor = QColorDialog::getColor(QColor(255, 255, 255), this);
            color.r = newColor.red();
            color.g = newColor.green();
            color.b = newColor.blue();
            colorButtonUpdateColor();
        }

        GUIPCLViewerItem::Color GUIPCLViewerItem::getColor() {
            return color;
        }

        std::string GUIPCLViewerItem::getName() {
            return name;
        }

        PointCloudPtrT GUIPCLViewerItem::getCloud() {
            PointCloudPtrT cloud(new PointCloudT);
            switch(item.which()) {
                case 0:
                    cloud->points.push_back(*boost::get<PointPtrT>(item));
                    cloud->width = static_cast<uint32_t>(cloud->points.size());
		            cloud->height = 1;
                    item = cloud;
                    goto UPDATE_CLOUD_CLOUD_PTR;
                case 1:
                    cloud = boost::get<PointCloudPtrT>(item);
UPDATE_CLOUD_CLOUD_PTR:
                    return cloud;
                    break;
                case 2:
                    cloud = boost::get<PcapCachePtrT>(this->item)->get(mediaTool->getFrameId());

                    color.data = 40;
                    goto UPDATE_CLOUD_CLOUD_PTR;
            }
        }
}