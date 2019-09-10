#pragma once

#include <QWidget>
#include <QDockWidget>
#include <QComboBox>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QPainter>
#include <QLineEdit>
#include <QKeyEvent>
#include <QTreeWidget>
#include <QHeaderView>
#include <QPushButton>
#include <GSLAM/core/GSLAM.h>

namespace GSLAM{

class PushButton : public QPushButton
{
    Q_OBJECT
public:
    PushButton(const QString& name,const std::string& topic,QWidget* parent=NULL)
        :QPushButton(name,parent)
    {
        connect(this, SIGNAL(clicked(bool)), this, SLOT(triggerdSlot()));
        _pub=messenger.advertise<bool>(topic);
    }

    PushButton(const QString& name,QWidget* parent,SvarFunction func)
        :QPushButton(name,parent){
        _func=func;
        connect(this, SIGNAL(triggered()), this, SLOT(triggerdSlot()));
    }

public slots:
    void triggerdSlot(bool){
        if(_func.isFunction()) _func();
        else _pub.publish(true);
    }
private:
    Publisher _pub;
    Svar      _func;
};

class PropertyItem : public QObject,public QTreeWidgetItem
{
    Q_OBJECT
public:
    PropertyItem(QTreeWidgetItem* parent,QString name,Svar value,Svar updateFunc=Svar())
        :QObject(),QTreeWidgetItem(parent,QStringList() << name),_value(value),_updateFunc(updateFunc){
        if(parent)
            parent->addChild(item());
    }

    virtual QWidget* widget(){return nullptr;}
    virtual QTreeWidgetItem* item(){return dynamic_cast<QTreeWidgetItem*>(this);}

    Svar _value;
    Svar _updateFunc;
};

class BoolPropertyItem : public PropertyItem
{
    Q_OBJECT
public:
    BoolPropertyItem(QTreeWidgetItem* parent,QString name,Svar value,Svar updateFunc=Svar())
        :PropertyItem(parent,name,value,updateFunc){
        _widget=new QCheckBox();
        _widget->setChecked(value.as<bool>());
        connect(_widget,SIGNAL(toggled(bool)),this,SLOT(slotUpdated(bool)));
    }
public slots:
    void slotUpdated(bool value){
        _value=value;
        if(_updateFunc.isFunction())
            _updateFunc(value);
    }
public:
    virtual QWidget* widget(){return _widget;}
    QCheckBox* _widget;
};

class JsonPropertyItem: public PropertyItem{
    Q_OBJECT
public:
    JsonPropertyItem(QTreeWidgetItem* parent,QString name,Svar value,Svar updateFunc=Svar())
        :PropertyItem(parent,name,value,updateFunc){
        _widget=new QLineEdit();
        _widget->setText(value.castAs<std::string>().c_str());
        connect(_widget,SIGNAL(editingFinished()),this,SLOT(slotUpdated()));
    }
public slots:
    void slotUpdated(){
        bool ok;
        if(_value.is<int>()){
            int value=_widget->text().toInt(&ok);
            if(ok) _value=value;
        }
        else if(_value.is<double>()){
            double value=_widget->text().toDouble(&ok);
            if(ok) _value=value;
        }
        else if(_value.is<std::string>()){
            _value=_widget->text().toStdString();
        }

        if(_updateFunc.isFunction())
            _updateFunc(_value);
    }
public:
    virtual QWidget* widget(){return _widget;}
    QLineEdit* _widget;
};

class TopicPropertyItem: public PropertyItem{
    Q_OBJECT
public:
    TopicPropertyItem(QTreeWidgetItem* parent,QString name,Svar value,Svar updateFunc=Svar())
        :PropertyItem(parent,name,value,updateFunc){
        _widget=new QComboBox();
        _widget->setIconSize(QSize(1, 26));
        updateTable();
//        connect(_widget,SIGNAL(currentIndexChanged(QString)),this,SLOT(slotUpdated(QString)));
        connect(_widget,SIGNAL(activated(QString)),this,SLOT(updateTable(QString)));
    }
public slots:
    void slotUpdated(QString topic){
        if(_updateFunc.isFunction()){
            LOG(INFO)<<"Changing topic to "<<topic.toStdString();
            _updateFunc(topic.toStdString());
        }
    }
    void updateTable(QString topic=0){
        for(Publisher pub:messenger.getPublishers())
            if(pub.getTypeName()==_value.as<SvarClass>().name())
            {
                if(_widget->findText(pub.getTopic().c_str())<0)
                    _widget->addItem(pub.getTopic().c_str());
            }

        if(_updateFunc.isFunction()){
            LOG(INFO)<<"Changing topic to "<<topic.toStdString();
            _updateFunc(topic.toStdString());
        }
    }
public:
    virtual QWidget* widget(){return _widget;}
    QLineEdit* _line;
    QComboBox* _widget;
};

class DisplayTree:public QTreeWidget{
public:
    DisplayTree(QWidget* parent):
        QTreeWidget(parent){
        this->header()->setVisible(false);
        this->setColumnCount(2);
        setColumnWidth(0,150);
        subDisplay=messenger.subscribe("qviz/display",[this](Svar display){
            this->addDisplay(display);
        });
    }

    void addDisplay(Svar display){
        std::string name=display.get<std::string>("name","Display");
        displays[name]=display;
        QTreeWidgetItem* display_item=new QTreeWidgetItem(this, QStringList() << name.c_str(),1);
        std::string icon=display.get<std::string>("icon","");
        if(!icon.empty())
            display_item->setIcon(0,QIcon(icon.c_str()));

        Svar args=display["config"]["__builtin__"]["args"];
        for(int i=0;i<args.size();i++){
            Svar arg=args[i];
            std::string name=arg[0].castAs<std::string>();
            Svar value =display["config"][name];
            std::string des=arg[2].castAs<std::string>();
            PropertyItem* item=nullptr;
            Svar callback=display["config"]["updated_callback"][name];
            if(value.is<bool>())
                item=new BoolPropertyItem(display_item,name.c_str(),value,callback);
            else if(value.is<int>()||value.is<double>()||value.is<std::string>())
                item=new JsonPropertyItem(display_item,name.c_str(),value,callback);
            else if(value.isClass())
                item=new TopicPropertyItem(display_item,name.c_str(),value,callback);
            if(item){
                item->setToolTip(0,des.c_str());
                setItemWidget(item,1,item->widget());
            }
        }
        addTopLevelItem(display_item);
    }

    Svar displays;
    Subscriber subDisplay;
};

class PanelDisplays : public QWidget
{
public:
    PanelDisplays(QWidget* parent,Svar config)
        :QWidget(parent),_config(config){
        tree=new DisplayTree(nullptr);
        QVBoxLayout* layout(new QVBoxLayout(this));
        layout->addWidget(tree);
        QHBoxLayout* layoutBottom(new QHBoxLayout());
        layoutBottom->addWidget(new QPushButton("Add",this));
        layoutBottom->addWidget(new QPushButton("Delete",this));
        layout->addLayout(layoutBottom);
        collectPlugins();
        setObjectName("Display Panel");
        setProperty("icon","");
    }

    void collectPlugins();

    std::map<std::string,Svar> display_plugins;
    Svar _config;
    DisplayTree* tree;
};

}
