#include <QHBoxLayout>
#include <QScrollBar>

#include <stdio.h>
#include <GSLAM/core/Svar.h>
#include <QDateTime>
#include <QDir>

#include "LogPanel.h"

LogTextSink::LogTextSink(QWidget *parent,GSLAM::Svar config):QWidget(parent),
    MAXIMUM_BLOCK_COUNT(10000)
{
    setObjectName("Logger");
    setProperty("area","right");
    QHBoxLayout *layout = new QHBoxLayout();
    layout->setMargin(2);
    setLayout(layout);

    log_widget_ = new QPlainTextEdit(this);
    log_widget_->setMaximumBlockCount(MAXIMUM_BLOCK_COUNT);
    log_widget_->setReadOnly(true);
    layout->addWidget(log_widget_);

    connect(this,SIGNAL(signalUpdated()),this,SLOT(refresh()));

    GSLAM::AddLogSink(this);

    QString logFolder = QString::fromLocal8Bit((svar.GetString("MainWindow.UserFolder", "")+"/log").c_str());
//    saveLogFile(logFolder);
    this->show();
}

LogTextSink::~LogTextSink()
{
    GSLAM::RemoveLogSink(this);
}

void LogTextSink::refresh()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if(!message_vec_.empty())
    {
        std::string msg;
        for(const auto &message : message_vec_)
        {
            msg+=message.first;
        }
        msg.pop_back();
        log_widget_->appendPlainText(QString::fromLocal8Bit(msg.c_str()));
    }
    QScrollBar *scrollbar = log_widget_->verticalScrollBar();
    if (scrollbar&&scrollbar->maximum()==scrollbar->sliderPosition())
        scrollbar->setSliderPosition(scrollbar->maximum());
    message_vec_.clear();
}


void LogTextSink::send(GSLAM::LogSeverity severity, const char *full_filename, const char *base_filename,
                       int line, const tm *tm_time, const char *message, size_t message_len)
{
//    std::ostringstream message_stream;
//    message_stream.fill('0');

//    message_stream << "[" << GSLAM::LogSeverityNames[severity][0]
//                   << std::setw(2) << 1 + tm_time->tm_mon
//                   << std::setw(2) << tm_time->tm_mday
//                   << ' '
//                   << std::setw(2) << tm_time->tm_hour << ':'
//                   << std::setw(2) << tm_time->tm_min << ':'
//                   << std::setw(2) << tm_time->tm_sec
//                   << ' '
//                   << full_filename << ':' << line << "] ";

//    message_stream << std::string(message, message_len);
//    std::string message_str = message_stream.str();

    {
        std::lock_guard<std::mutex> lock(mutex_);
        message_vec_.push_back(MessageType(std::string(message, message_len),severity));
    }
    emit signalUpdated();
}

void LogTextSink::saveLogFile(const QString &logFolder)
{
    QDateTime currentDateTime =QDateTime::currentDateTime();
    QString currentDate =currentDateTime.toString("yyyy-MM-dd_hh-mm-ss");
    QString logPath = logFolder + "/" + currentDate;

    QDir dir(logFolder);
    if( !dir.exists() )
        dir.mkdir(logFolder);
}

GSLAM_REGISTER_PANEL(log,LogTextSink);
