#ifndef PCAP_MEDIA_WIDGET_H_
#define PCAP_MEDIA_WIDGET_H_

#include <QPushButton>
#include <QLabel>
#include <QSlider>
#include <QLabel>
#include <QTimer>

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

#endif