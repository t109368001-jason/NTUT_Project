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

        int getFrameId();

        void start(int delay_ms = 100);
        void pause();

    private:
        QPushButton *playButton_;
        QLabel *minLabel_;
        QSlider *peekSlider_;
        QLabel *maxLabel_;
        QTimer *timer_;

        int beg_;
        int end_;
        int frameId_;
        bool pause_;

        void run();

        void setFrameId(int frameId, QWidget* obj = nullptr);
        
        void playButtonTriggered();
        void peekSliderValueChanged(int value);

    };
}

using namespace velodyne;
MediaWidget::MediaWidget(int beg, int end, QWidget *parent) : QWidget(parent), beg_(beg), end_(end), frameId_(0), pause_(true) {
    QHBoxLayout *layout = new QHBoxLayout(this);
    playButton_ = new QPushButton(style()->standardIcon(QStyle::StandardPixmap::SP_MediaPlay), "", this);
    minLabel_ = new QLabel(std::to_string(beg_).c_str(), this);
    peekSlider_ = new QSlider(Qt::Orientation::Horizontal, this);
    maxLabel_ = new QLabel(std::to_string(end_).c_str(), this);
    timer_ = new QTimer(this);

    peekSlider_->setRange(beg_, end_);
    peekSlider_->setFixedWidth(100);

    layout->addWidget(playButton_);
    layout->addWidget(minLabel_);
    layout->addWidget(peekSlider_);
    layout->addWidget(maxLabel_);

    setLayout(layout);

    connect(playButton_, &QPushButton::clicked, this, &MediaWidget::playButtonTriggered);
    connect(peekSlider_, &QSlider::valueChanged, this, &MediaWidget::peekSliderValueChanged);
    connect(timer_, &QTimer::timeout, this, &MediaWidget::run);
    setFrameId(0);
}

int MediaWidget::getFrameId() {
    return frameId_;
}

void MediaWidget::start(int delay_ms) {
    pause_ = false;
    timer_->start(delay_ms);
}

void MediaWidget::pause() {
    pause_ = true;
    timer_->stop();
}

void MediaWidget::run() {
    if(!pause_) {
        setFrameId((frameId_+1 < end_) ? frameId_+1 : 0);
    }
}

void MediaWidget::setFrameId(int frameId, QWidget* obj) {
    frameId_ = frameId;

    bool sliderOldState = peekSlider_->blockSignals(true);
    if(obj != peekSlider_) peekSlider_->setValue(frameId_);
    peekSlider_->blockSignals(sliderOldState);
}

void MediaWidget::playButtonTriggered() {
    if(pause_) start();
    else pause();
    playButton_->setIcon(style()->standardIcon(pause_ ? QStyle::StandardPixmap::SP_MediaPlay : QStyle::StandardPixmap::SP_MediaPause));
}

void MediaWidget::peekSliderValueChanged(int value) {
    setFrameId(peekSlider_->value(), peekSlider_);
}

#endif