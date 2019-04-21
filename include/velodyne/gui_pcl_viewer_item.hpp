#ifndef VELODYNE_GUI_PCL_VIEWER_ITEM_H_
#define VELODYNE_GUI_PCL_VIEWER_ITEM_H_
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <boost/variant.hpp>
#include <velodyne/pcap_cache.hpp>
#include <velodyne/media_widget.hpp>
#include <QColorDialog>

namespace velodyne {
    typedef boost::shared_ptr<pcl::PointXYZ> PointPtrT;
    typedef boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> PointCloudPtrT;
    typedef boost::shared_ptr<PcapCache> PcapCachePtrT;
    typedef boost::variant<PointPtrT, PointCloudPtrT, PcapCachePtrT> GUIPCLViewerItemT;
    class GUIPCLViewerItem : public QWidget {
    public:
        union Color{
            struct { 
                uint8_t b;      //LSB
                uint8_t g;
                uint8_t r;
                uint8_t data;
            };
            uint32_t rgb;
        };

        GUIPCLViewerItem(QWidget *parent = nullptr);

        Color getColor();

        std::string getName();

        PointCloudPtrT getCloud();

        template<typename _ItemT>
        void setItem(std::string name, _ItemT &item);

        template<typename _ItemT>
        void resetItem(std::string name, _ItemT &item);

        template<typename _ItemT>
        void addItem(_ItemT &item);

    private:
        QGridLayout *layout;
        QLabel *nameLabel;
        QPushButton *colorButton;

        GUIPCLViewerItemT item;
        MediaWidget *mediaTool;

        std::string name;
        Color color;

        void colorButtonUpdateColor();

        void colorButtonClicked();

    };
}

using namespace velodyne;
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

GUIPCLViewerItem::Color GUIPCLViewerItem::getColor() {
    return color;
}

std::string GUIPCLViewerItem::getName() {
    return name;
}

PointCloudPtrT GUIPCLViewerItem::getCloud() {
    PointCloudPtrT cloud(new PointCloudT);
    switch(item.which()) {
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

template<typename _ItemT>
void GUIPCLViewerItem::setItem(std::string name, _ItemT &item) {
    this->name = name;
    this->item = item;
    nameLabel->setText(name.c_str());
    colorButtonUpdateColor();
    colorButton->setFixedSize(20, 20);
    connect(colorButton, &QPushButton::clicked, this, &GUIPCLViewerItem::colorButtonClicked);
    PointCloudPtrT cloud(new PointCloudT);
    switch(this->item.which()) {
        case 0:
            cloud->points.push_back(*boost::get<PointPtrT>(this->item));
            cloud->width = static_cast<uint32_t>(cloud->points.size());
            cloud->height = 1;
            this->item = cloud;
            break;
        case 1:

            break;
        case 2:
            delete colorButton;
            colorButton = nullptr;
            mediaTool = new MediaWidget(boost::get<PcapCachePtrT>(this->item)->beg, boost::get<PcapCachePtrT>(this->item)->totalFrame-1, this);
            this->layout->addWidget(mediaTool, 0, 1);
            break;
    }
}

template<typename _ItemT>
void GUIPCLViewerItem::resetItem(std::string name, _ItemT &item) {
    this->name = name;
    this->item = item;
    nameLabel->setText(name.c_str());
    PointCloudPtrT cloud(new PointCloudT);
    switch(this->item.which()) {
        case 0:
            cloud->points.push_back(*boost::get<PointPtrT>(this->item));
            cloud->width = static_cast<uint32_t>(cloud->points.size());
            cloud->height = 1;
            this->item = cloud;
            break;
        case 1:

            break;
        case 2:
            break;
    }
}

template<typename _ItemT>
void GUIPCLViewerItem::addItem(_ItemT &item) {
    GUIPCLViewerItemT tmp = item;
    PointCloudPtrT cloud;
    switch(this->item.which()) {
        case 1:
            cloud = boost::get<PointCloudPtrT>(this->item);
            switch (tmp.which())
            {
                case 0:
                    cloud->points.push_back(*boost::get<PointPtrT>(tmp));
                    break;
                case 1:
                    std::copy(boost::get<PointCloudPtrT>(tmp)->points.begin(),
                        boost::get<PointCloudPtrT>(tmp)->points.end(),
                        std::back_inserter(cloud->points)
                    );
                    break;
            }
            cloud->width = static_cast<uint32_t>(cloud->points.size());
            cloud->height = 1;
            break;
    }
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
#endif