#ifndef STREAMTHREAD_H
#define STREAMTHREAD_H

#include <QObject>
#include <QThread>
#include <QString>

#include <memory>

namespace wirispro_manager_panel {

class StreamThread : public QThread
{
    Q_OBJECT
    public:
        StreamThread(bool* stream, const QString & ssrc) :  _ssrc(ssrc), _stream(stream) {}
    Q_SIGNALS:
        void imageSourceChanged();
    protected:
        virtual void run() = 0;
        QString _ssrc;
        bool* _stream;
};

} // namespace wirispro_manager_panel
#endif // STREAMTHREAD_H