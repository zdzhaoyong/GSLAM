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
#include <QMenu>
#include <GSLAM/core/GSLAM.h>
#include <GSLAM/core/Display.h>

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

class MessengerAction : public QAction
{
    Q_OBJECT
public:
    MessengerAction(const QString& name,std::string topic,QMenu* parent=NULL)
        :QAction(parent)
    {
        setText(name);
        if(parent)
            parent->addAction(this);
        connect(this, SIGNAL(triggered()), this, SLOT(triggerdSlot()));
        _pub=messenger.advertise<bool>(topic);
    }

    explicit MessengerAction(const QString& name,QMenu* parent,Svar func)
        :QAction(parent){
        _func=func;
        setText(name);
        if(parent)
            parent->addAction(this);
        connect(this, SIGNAL(triggered()), this, SLOT(triggerdSlot()));
    }
public slots:
    void triggerdSlot(){
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
    PropertyItem(PropertyItem* parent,QString name,Svar value,Svar updateFunc=Svar())
        : QObject(),QTreeWidgetItem(parent,QStringList() << name),
          _parent(parent),_value(value),_updateFunc(updateFunc){
        setObjectName(name);
        if(parent)
            parent->addChild(item());
    }

    virtual QWidget* widget(){return nullptr;}

    virtual void update(){
//        LOG(INFO)<<"Enter "<<objectName().toStdString();
        if(_updateFunc.isFunction()) {
//            LOG(INFO)<<"Update to "<<_value;
            _updateFunc();return;
        }
//        else LOG(INFO)<<"No callback.";
        if(!_parent) return;
        PropertyItem* pit=dynamic_cast<PropertyItem*>(_parent);
        if(pit)
        {
            pit->update();
//            LOG(INFO)<<objectName().toStdString()<<" update to "<<_value;
        }
    }

    virtual QTreeWidgetItem* item(){return dynamic_cast<QTreeWidgetItem*>(this);}

    QTreeWidgetItem* _parent;
    Svar _value;
    Svar _updateFunc;
};

class BoolPropertyItem : public PropertyItem
{
    Q_OBJECT
public:
    BoolPropertyItem(PropertyItem* parent,QString name,Svar value,Svar updateFunc=Svar())
        :PropertyItem(parent,name,value,updateFunc){
        _widget=new QCheckBox();
        _widget->setChecked(value.as<bool>());
        connect(_widget,SIGNAL(toggled(bool)),this,SLOT(slotUpdated(bool)));
    }
public slots:
    void slotUpdated(bool value){
        _value=value;
        update();
    }
public:
    virtual QWidget* widget(){return _widget;}
    QCheckBox* _widget;
};

class JsonPropertyItem: public PropertyItem{
    Q_OBJECT
public:
    JsonPropertyItem(PropertyItem* parent,QString name,Svar value,Svar updateFunc=Svar())
        :PropertyItem(parent,name,value,updateFunc){
        _widget=new QLineEdit();
        std::stringstream sst;
        sst<<value;
        _widget->setText(sst.str().c_str());
        if(value.is<int>()||value.is<double>()||value.is<std::string>());
        else _widget->setEnabled(false);
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
        else _widget->setEnabled(false);

        update();
    }
public:
    virtual QWidget* widget(){return _widget;}
    QLineEdit* _widget;
};

class TopicPropertyItem: public PropertyItem{
    Q_OBJECT
public:
    TopicPropertyItem(PropertyItem* parent,QString name,Svar value,Svar updateFunc=Svar())
        :PropertyItem(parent,name,value,updateFunc),_topic(value.as<Topic>()){
        _widget=new QComboBox();
        _widget->setIconSize(QSize(1, 26));
        _widget->setEditable(true);
        updateTable("");
//        connect(_widget,SIGNAL(currentIndexChanged(QString)),this,SLOT(slotUpdated(QString)));
        connect(_widget,SIGNAL(activated(QString)),this,SLOT(updateTable(QString)));
        _sub=messenger.subscribe("messenger/newpub",[this](Publisher pub){
            this->updateTable("");
        });
    }
public slots:
    void updateTable(QString topic){
        for(Publisher pub:messenger.getPublishers())
            if(pub.getTypeName()==_topic._class.as<SvarClass>().name())
            {
                if(_widget->findText(pub.getTopic().c_str())<0)
                    _widget->addItem(pub.getTopic().c_str());
            }
        if(topic.isEmpty()&&_widget->count()>=1)
        {
            _widget->setCurrentIndex(0);
//            LOG(INFO)<<_widget->currentIndex()<<_widget->currentText().toStdString();
            _topic._name=_widget->currentText().toStdString();
        }
        else
            _topic._name=topic.toStdString();
        update();
    }
public:
    virtual QWidget* widget(){return _widget;}
    QLineEdit* _line;
    QComboBox* _widget;
    Topic&     _topic;
    Subscriber _sub;
};

class DisplayTree:public QTreeWidget{
    Q_OBJECT
public:
    DisplayTree(QWidget* parent,std::string topic=""):
        QTreeWidget(parent){
//        this->header()->setVisible(false);
#if QT_VERSION>=0x050000
#else
        header()->setResizeMode(QHeaderView::Interactive);
#endif
        this->setColumnCount(2);
        setColumnWidth(0,150);
        subDisplay=messenger.subscribe(topic,[this](Svar display){
            this->addDisplay(display,"",nullptr,Svar());
        });

        this->setContextMenuPolicy(Qt::CustomContextMenu);

        connect(this, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(slotMenu(const QPoint&)));
    }

    PropertyItem* addObjectDisplay(Svar display,std::string name,PropertyItem* parent,Svar callback){
//        LOG(INFO)<<display;
        if(display.exist("__cbk__"))
            callback=display["__cbk__"];
        if(display.exist("__name__"))
            name=display["__name__"].castAs<std::string>();
//        LOG(INFO)<<callback;

        PropertyItem* item=new PropertyItem(parent,name.c_str(),display,callback);
        if(display.exist("__icon__"))
            item->setIcon(0,QIcon(display["__icon__"].castAs<std::string>().c_str()));
        if(display.exist("__doc__"))
            item->setToolTip(0,display["__doc__"].castAs<std::string>().c_str());


        for(auto child:display.as<SvarObject>()._var){
            if(child.first.empty()||child.first.front()=='_') continue;
            addDisplay(child.second,child.first,item,display["__cbk__"+child.first]);
        }
        if(parent)
            parent->addChild(item);
        else
            addTopLevelItem(item);
        return item;
    }

    PropertyItem* addVectorDisplay(Svar display,std::string name,PropertyItem* parent,Svar callback){
        PropertyItem* item=new PropertyItem(parent,name.c_str(),display,callback);
        item->setText(0,name.c_str());
        std::vector<Svar> var=display.as<SvarArray>()._var;
        for(int i=0;i<var.size();i++){
            addDisplay(var[i],std::to_string(i),item);
        }
        if(parent)
            parent->addChild(item);
        else
            addTopLevelItem(item);
        return item;
    }

    PropertyItem* addDisplay(Svar display,std::string name="",PropertyItem* parent=nullptr,Svar callback=Svar()){
        if(display.isObject()) return addObjectDisplay(display,name,parent,callback);
        if(display.isArray()) return addVectorDisplay(display,name,parent,callback);

        PropertyItem* item=nullptr;
        if(display.is<bool>())
            item=new BoolPropertyItem(parent,name.c_str(),display,callback);
        else if(display.is<Topic>())
            item=new TopicPropertyItem(parent,name.c_str(),display,callback);
        else //if(display.is<int>()||display.is<double>()||display.is<std::string>())
            item=new JsonPropertyItem(parent,name.c_str(),display,callback);

        if(item){
            setItemWidget(item,1,item->widget());
            addTopLevelItem(item);
        }
        return item;
    }

public slots:
    void slotMenu(const QPoint& pos){
        PropertyItem *item = dynamic_cast<PropertyItem *>(this->itemAt(pos));
        if(!item) return;
        Svar value=item->_value;
        if(!value.isObject()) return;
        Svar menuVar=value["__menu__"];
        if(!menuVar.isObject()) return;
        QMenu menu(this);
        for(std::pair<std::string,Svar> v:menuVar.as<SvarObject>()._var)
        {
            menu.addAction(new MessengerAction(v.first.c_str(),nullptr,v.second));
        }
        menu.exec(QCursor::pos());
    }
public:

    Svar displays;
    Subscriber subDisplay;
};

class PanelDisplays : public QWidget
{
public:
    PanelDisplays(QWidget* parent,Svar config)
        :QWidget(parent),_config(config){
        tree=new DisplayTree(nullptr,"qviz/display");
        QVBoxLayout* layout(new QVBoxLayout(this));
        layout->addWidget(tree);
//        QHBoxLayout* layoutBottom(new QHBoxLayout());
//        layoutBottom->addWidget(new QPushButton("Add",this));
//        layoutBottom->addWidget(new QPushButton("Delete",this));
//        layout->addLayout(layoutBottom);
        collectPlugins();
        setObjectName("Display Panel");
        setProperty("stay",true);
        setProperty("area","left");
    }

    void collectPlugins();

    Svar _config;
    DisplayTree* tree;
};

}
