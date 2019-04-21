#ifndef PCAP_MEDIA_WIDGET_H_
#define PCAP_MEDIA_WIDGET_H_

#include <QPushButton>
#include <QLabel>
#include <QSlider>
#include <QLabel>
#include <QTimer>
#include <QStyle>
#include <QLayout>

namespace velodyne{
    class MediaWidget : public QWidget {
    public:
        MediaWidget(int beg, int end, QWidget *parent);
        void start(int delay_ms = 100);
        void pause();
        void stop();
        int getFrameId();
    private:
        QPushButton *play_;
        QLabel *labelMin_;
        QSlider *slider_;
        QLabel *labelMax_;
        QTimer *timer_;

        int beg_;
        int end_;
        int frameId_;
        bool pause_;

        void playTriggered();

        void sliderValueChanged(int value);

        bool setFrameId(int frameId, QWidget* obj = nullptr);

        void run();

    };
}

using namespace velodyne;
MediaWidget::MediaWidget(int beg, int end, QWidget *parent) : QWidget(parent), pause_(true), beg_(beg), end_(end) {
    QHBoxLayout *layout = new QHBoxLayout(this);
    play_ = new QPushButton(style()->standardIcon(QStyle::StandardPixmap::SP_MediaPlay), "", this);
    labelMin_ = new QLabel(std::to_string(beg_).c_str(), this);
    slider_ = new QSlider(Qt::Orientation::Horizontal, this);
    labelMax_ = new QLabel(std::to_string(end_).c_str(), this);
    timer_ = new QTimer(this);

    slider_->setRange(beg_, end_);
    slider_->setFixedWidth(100);

    layout->addWidget(play_);
    layout->addWidget(labelMin_);
    layout->addWidget(slider_);
    layout->addWidget(labelMax_);

    setLayout(layout);

    connect(play_, &QPushButton::clicked, this, &MediaWidget::playTriggered);
    connect(slider_, &QSlider::valueChanged, this, &MediaWidget::sliderValueChanged);
    connect(timer_, &QTimer::timeout, this, &MediaWidget::run);
    setFrameId(0);
}
void MediaWidget::start(int delay_ms) {
    pause_ = false;
    timer_->start(100);
}
void MediaWidget::pause() {
    pause_ = true;
    timer_->stop();
}
void MediaWidget::stop() {
    pause_ = true;
    timer_->stop();
    setFrameId(0);
}
int MediaWidget::getFrameId() {
    return frameId_;
}
void MediaWidget::playTriggered() {
    if(pause_) start();
    else pause();
    play_->setIcon(style()->standardIcon(pause_ ? QStyle::StandardPixmap::SP_MediaPlay : QStyle::StandardPixmap::SP_MediaPause));
}

void MediaWidget::sliderValueChanged(int value) {
    setFrameId(slider_->value(), slider_);
}

bool MediaWidget::setFrameId(int frameId, QWidget* obj) {
    frameId_ = frameId;
    if(frameId_ > end_) return false;

    bool sliderOldState = slider_->blockSignals(true);
    if(obj != slider_) slider_->setValue(frameId_);
    slider_->blockSignals(sliderOldState);
    
    return true;
}

void MediaWidget::run() {
    if(!pause_) {
        if(!setFrameId(frameId_+1)) {
            setFrameId(0);
        }
    }
}

#endif