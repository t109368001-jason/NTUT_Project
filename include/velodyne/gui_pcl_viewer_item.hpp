#ifndef VELODYNE_GUI_PCL_VIEWER_ITEM_H_
#define VELODYNE_GUI_PCL_VIEWER_ITEM_H_
#include <QCheckBox>
#include <QColorDialog>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <boost/variant.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <velodyne/pcap_cache.hpp>
#include <velodyne/media_widget.hpp>

namespace velodyne {
    typedef pcl::PointXYZ PointT;
    typedef std::vector<PointT> PointsT;
    typedef boost::shared_ptr<pcl::PointCloud<PointT>> PointCloudPtrT;
    typedef boost::shared_ptr<PcapCache> PcapCachePtrT;
    typedef boost::variant<PointsT, PointCloudPtrT, PcapCachePtrT> GUIPCLViewerItemT;
    typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> ViewerPtrT;
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
        ~GUIPCLViewerItem() {
            if(mediaTool_) {
                delete mediaTool_;
                mediaTool_ = nullptr;
            }
        }

        std::string getName();

        template<typename _ItemT>
        void addItem(_ItemT &item);
        template<typename _ItemT>
        void setItem(std::string name, _ItemT &item);

        void updateView(ViewerPtrT &viewer);

    private:
        QGridLayout *layout_;
        QLabel *nameLabel_;
        QPushButton *colorButton_;
        QPushButton *modeButton_;
        QCheckBox *checkBox_;
        MediaWidget *mediaTool_;

        std::string name_;
        
        GUIPCLViewerItemT item_;

        Color color_;

        bool isChanged_;

        void colorButtonClicked();
        void colorButtonUpdateColor();

    };
}

using namespace velodyne;
GUIPCLViewerItem::GUIPCLViewerItem(QWidget *parent) : QWidget(parent), color_(Color{0xff, 0xff, 0xff, 0x00}), colorButton_(nullptr), mediaTool_(nullptr), modeButton_(nullptr) {
    layout_ = new QGridLayout(this);
    nameLabel_ = new QLabel(this);
    checkBox_ = new QCheckBox("", this);

    checkBox_->setChecked(true);

    layout_->addWidget(checkBox_, 0, 0);
    layout_->addWidget(nameLabel_, 0, 2);
    layout_->setContentsMargins(0,0,0,0);
    
    setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

    setLayout(layout_);

    connect(checkBox_, &QCheckBox::stateChanged, [this](){isChanged_ = true;});
}

std::string GUIPCLViewerItem::getName() {
    return name_;
}

template<typename _ItemT>
void GUIPCLViewerItem::addItem(_ItemT &item) {
    GUIPCLViewerItemT itemIn = item;
    switch(itemIn.which()) {
        case 0:
            std::copy(boost::get<PointsT>(itemIn).begin(), boost::get<PointsT>(itemIn).end(), std::back_inserter(boost::get<PointsT>(item_)));
            break;
        case 1:
            std::copy(boost::get<PointCloudPtrT>(itemIn)->points.begin(), boost::get<PointCloudPtrT>(itemIn)->points.end(), std::back_inserter(boost::get<PointCloudPtrT>(item_)->points));
            boost::get<PointCloudPtrT>(item_)->width = static_cast<uint32_t>(boost::get<PointCloudPtrT>(item_)->points.size());
            boost::get<PointCloudPtrT>(item_)->height = 1;

            break;
    }
    isChanged_ = true;
}

template<typename _ItemT>
void GUIPCLViewerItem::setItem(std::string name, _ItemT &item) {
    name_ = name;
    nameLabel_->setText(name_.c_str());
    item_ = item;
    switch(item_.which()) {
        case 0:
        case 1:
            if(!colorButton_) {
                colorButton_ = new QPushButton(this);
                colorButton_->setFixedSize(20,20);
                colorButtonUpdateColor();
                connect(colorButton_, &QPushButton::clicked, this, &GUIPCLViewerItem::colorButtonClicked);
                layout_->addWidget(colorButton_, 0, 1);
            }
            if(mediaTool_) {
                delete mediaTool_;
                mediaTool_ = nullptr;
            }
            if(modeButton_) {
                delete modeButton_;
                modeButton_ = nullptr;
            }
            break;
        case 2:
            if(colorButton_) {
                delete colorButton_;
                colorButton_ = nullptr;
            }
            if(!mediaTool_) {
                mediaTool_ = new MediaWidget(boost::get<PcapCachePtrT>(item_)->beg, boost::get<PcapCachePtrT>(item_)->totalFrame-1, this);
                layout_->addWidget(mediaTool_, 0, 3);
            }
            if(!modeButton_) {
                modeButton_ = new QPushButton(this);
                modeButton_->setFixedSize(20,20);
                layout_->addWidget(modeButton_, 0, 1);
                connect(modeButton_, &QPushButton::clicked, std::bind(&PcapCache::nextMode, boost::get<PcapCachePtrT>(this->item_)));
            }
            color_.data = 50;
            break;
    }
    isChanged_ = true;
}

void GUIPCLViewerItem::updateView(ViewerPtrT &viewer) {
    if(item_.which() == 2) {
        if(boost::get<PcapCachePtrT>(item_)->currentFrameId != mediaTool_->getFrameId()) isChanged_ = true;
    }
    if(isChanged_) {
        viewer->removeShape(name_);
        viewer->removePointCloud(name_);
        if(checkBox_->checkState() == Qt::CheckState::Checked) {
            if(item_.which() == 0) {
                if(boost::get<PointsT>(item_).size() > 1) {
                    PointT p1 = boost::get<PointsT>(item_).front();
                    PointT p2 = boost::get<PointsT>(item_).back();
                    double d = std::sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z))/100.0;
                    nameLabel_->setText((name_ + "(" + std::to_string(d) + " m)").c_str());
                    viewer->addLine(p1, p2, double(color_.r)/255.0, double(color_.g)/255.0, double(color_.b)/255.0, name_);
                }
            } else if(item_.which() == 1) {
                myFunction::updateCloud(viewer, boost::get<PointCloudPtrT>(item_), name_, color_.r, color_.g, color_.b);
            } else {
                myFunction::updateCloud(viewer, boost::get<PcapCachePtrT>(item_)->get(mediaTool_->getFrameId()), name_, double(color_.data) * 100.0);
            }
        }
        isChanged_ = false;
    }
}

void GUIPCLViewerItem::colorButtonClicked() {
    QColor newColor = QColorDialog::getColor(QColor(color_.r, color_.g, color_.b), this);
    color_.r = newColor.red();
    color_.g = newColor.green();
    color_.b = newColor.blue();
    colorButtonUpdateColor();
}

void GUIPCLViewerItem::colorButtonUpdateColor() {
    QPalette pal = colorButton_->palette();
    pal.setColor(QPalette::Button, QColor(color_.r, color_.g, color_.b));
    colorButton_->setAutoFillBackground(true);
    colorButton_->setPalette(pal);
    colorButton_->update();
    isChanged_ = true;
}

#endif