#ifndef RTMAPPER_LOGWIDGET_H
#define RTMAPPER_LOGWIDGET_H

#include <mutex>

#include <QPlainTextEdit>
#include <QTimer>

#include <GSLAM/core/GSLAM.h>


class LogTextSink : public QWidget, public GSLAM::LogSink
{
    Q_OBJECT
public:
    LogTextSink(QWidget* parent,GSLAM::Svar config);
    virtual ~LogTextSink();

public:
    void send(GSLAM::LogSeverity severity,
                      const char* full_filename,
                      const char* base_filename,
                      int line,
                      const struct tm* tm_time,
                      const char* message,
                      size_t message_len);
    virtual void WaitTillSent(){return;}

    void saveLogFile(const QString &logFolder);
signals:
    void signalUpdated();
public slots:
    void refresh();

protected:
    QPlainTextEdit *log_widget_;
    std::mutex mutex_;

    typedef std::pair<std::string, GSLAM::LogSeverity> MessageType;
    std::vector<MessageType> message_vec_;

    const int MAXIMUM_BLOCK_COUNT;
};

#endif // end of RTMAPPER_LOGWIDGET_H

