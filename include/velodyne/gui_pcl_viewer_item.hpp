#ifndef VELODYNE_GUI_PCL_VIEWER_ITEM_H_
#define VELODYNE_GUI_PCL_VIEWER_ITEM_H_
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <boost/variant.hpp>
#include <velodyne/pcap_cache.h>
#include <velodyne/media_widget.h>

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
        void addItem(std::string &name, _ItemT &item) {
            this->name = name;
            this->item = item;
            nameLabel->setText(name.c_str());
            colorButtonUpdateColor();
            colorButton->setFixedSize(20, 20);
            connect(colorButton, &QPushButton::clicked, this, &GUIPCLViewerItem::colorButtonClicked);
            switch(this->item.which()) {
                case 0:

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
#endif