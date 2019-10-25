#pragma once

#include <QWidget>
#include <QDockWidget>
#include <QComboBox>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QPainter>
#include <QLineEdit>
#include <QLabel>
#include <QKeyEvent>
#include <QTreeWidget>
#include <QHeaderView>
#include <QPushButton>
#include <QMenu>
#include <QColorDialog>
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
    PropertyItem(PropertyItem* parent,QTreeWidget* tree,QString name,Svar value,Svar updateFunc=Svar())
        : QObject(),QTreeWidgetItem(parent,QStringList() << name),
          _parent(parent),_tree(tree),_value(value),_updateFunc(updateFunc){
        setObjectName(name);

        if(parent)
            parent->addChild(item());
        else{
            _tree->addTopLevelItem(this);
        }

        if(value.isObject()){
            if(value.exist("__icon__"))
                setIcon(0,QIcon(value["__icon__"].castAs<std::string>().c_str()));
            if(value.exist("__doc__"))
                setToolTip(0,value["__doc__"].castAs<std::string>().c_str());
        }
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

    virtual void execMenu(){}

    virtual void clicked(int col){}

    QTreeWidgetItem* _parent;
    QTreeWidget*     _tree;
    Svar _value;
    Svar _updateFunc;
};

class BoolPropertyItem : public PropertyItem
{
    Q_OBJECT
public:
    BoolPropertyItem(PropertyItem* parent,QTreeWidget* tree,QString name,Svar value,Svar updateFunc=Svar())
        :PropertyItem(parent,tree,name,value,updateFunc){
        _widget=new QCheckBox();
        _widget->setChecked(value.as<bool>());
        connect(_widget,SIGNAL(toggled(bool)),this,SLOT(slotUpdated(bool)));
        _tree->setItemWidget(this,1,widget());
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
    JsonPropertyItem(PropertyItem* parent,QTreeWidget* tree,QString name,Svar value,Svar updateFunc=Svar())
        :PropertyItem(parent,tree,name,value,updateFunc){
        _widget=new QLineEdit();
        std::stringstream sst;
        sst<<value;
        _widget->setText(sst.str().c_str());
        if(value.is<int>()||value.is<double>()||value.is<std::string>());
        else _widget->setEnabled(false);
        connect(_widget,SIGNAL(editingFinished()),this,SLOT(slotUpdated()));
        _tree->setItemWidget(this,1,widget());
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
    TopicPropertyItem(PropertyItem* parent,QTreeWidget* tree,QString name,Svar value,Svar updateFunc=Svar())
        :PropertyItem(parent,tree,name,value,updateFunc),_topic(value.as<Topic>()){
        _widget=new QComboBox();
        _widget->setIconSize(QSize(1, 26));
        _widget->setEditable(true);
        updateTable("");
//        connect(_widget,SIGNAL(currentIndexChanged(QString)),this,SLOT(slotUpdated(QString)));
        connect(_widget,SIGNAL(activated(QString)),this,SLOT(updateTable(QString)));
        _sub=messenger.subscribe("messenger/newpub",[this](Publisher pub){
            this->updateTable("");
        });
        _tree->setItemWidget(this,1,widget());
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

class ColorPropertyItem: public PropertyItem{
    Q_OBJECT
public:
    ColorPropertyItem(PropertyItem* parent,QTreeWidget* tree,QString name,Svar value,Svar updateFunc=Svar())
        :PropertyItem(parent,tree,name,value,updateFunc){
        Point3ub color=value.as<Point3ub>();
        _widget=new QPushButton();
        QPalette palette = _widget->palette();
        palette.setColor(QPalette::Button, QColor(color.x,color.y,color.z));
        _widget->setPalette(palette);
        _widget->setAutoFillBackground(true);
        _widget->setFlat(true);

        connect(_widget,SIGNAL(clicked(bool)),this,SLOT(slotUpdated(bool)));
        _tree->setItemWidget(this,1,widget());
    }
public slots:
    void slotUpdated(bool){
        Point3ub color=_value.as<Point3ub>();
        QColor   c = QColorDialog::getColor(QColor(color.x,color.y,color.z));
        QPalette palette = _widget->palette();
        palette.setColor(QPalette::Button, c);
        _widget->setPalette(palette);
        _value=Point3ub(c.red(),c.green(),c.blue());

        update();
    }
public:
    virtual QWidget* widget(){return _widget;}
    QPushButton* _widget;
};

class ObjectPropertyItem: public PropertyItem{
    Q_OBJECT
public:
    ObjectPropertyItem(PropertyItem* parent,QTreeWidget* tree,QString name,Svar value,Svar updateFunc=Svar());

    void execMenu(){
        Svar menuVar=_value["__menu__"];
        QMenu menu(_tree);
        if(menuVar.isObject())
        for(std::pair<std::string,Svar> v:menuVar.as<SvarObject>()._var)
        {
            if(v.first=="Delete") continue;
            menu.addAction(new MessengerAction(v.first.c_str(),nullptr,v.second));
        }
        SvarFunction deleteThis=[this](){
            if(_value["__menu__"].exist("Delete")){
                _value["__menu__"]["Delete"]();
            }
            auto objParent=dynamic_cast<ObjectPropertyItem*>(_parent);
            if(objParent){
                objParent->_value.erase(this->objectName().toStdString());
            }
            delete this;
        };
        menu.addAction(new MessengerAction("Delete",nullptr,deleteThis));

        menu.exec(QCursor::pos());
    }

    virtual void clicked(int col){
        if(col!=1) return;
        if(!_value["visible"].is<bool>()) return;
        bool& visible=_value["visible"].as<bool>();
        visible=!visible;
        if(visible)
            setIcon(1,QIcon(":/icon/visiable.png"));
        else
            setIcon(1,QIcon(":/icon/nVisiable.png"));
        Svar callback=_value["__cbk__visible"];
        if(callback.isFunction()) callback();
        else update();
    }
};

class ArrayPropertyItem: public PropertyItem{
    Q_OBJECT
public:
    ArrayPropertyItem(PropertyItem* parent,QTreeWidget* tree,QString name,Svar value,Svar updateFunc=Svar())
        : PropertyItem(parent,tree,name,value,updateFunc){
        std::vector<Svar> var=value.as<SvarArray>()._var;
        for(int i=0;i<var.size();i++){
            std::string name=std::to_string(i);
            Svar display=var[i];
            Svar callback=Svar();
            if     (display.isObject()) new ObjectPropertyItem(this,tree,name.c_str(),display,callback);
            else if(display.isArray())  new ArrayPropertyItem(this,tree,name.c_str(),display,callback);
            else if(display.is<bool>())  new BoolPropertyItem(this,tree,name.c_str(),display,callback);
            else if(display.is<Topic>())  new TopicPropertyItem(this,tree,name.c_str(),display,callback);
            else if(display.is<Point3ub>()) new ColorPropertyItem(this,tree,name.c_str(),display,callback);
            else  new JsonPropertyItem(this,tree,name.c_str(),display,callback);
        }
    }
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
            emit signalInsertPlay(display);
        });

        this->setContextMenuPolicy(Qt::CustomContextMenu);

        connect(this, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(slotMenu(const QPoint&)));
        connect(this,SIGNAL(signalInsertPlay(Svar)),this,SLOT(slotInsertPlay(Svar)));
        connect(this, SIGNAL(itemClicked(QTreeWidgetItem*,int)),
                this, SLOT(slotItemClicked(QTreeWidgetItem*,int)));
    }

    PropertyItem* addDisplay(Svar display,std::string name="",PropertyItem* parent=nullptr,Svar callback=Svar()){
        if     (display.isObject()) return  new ObjectPropertyItem(parent,this,name.c_str(),display,callback);
        else if(display.isArray()) return   new ArrayPropertyItem(parent,this,name.c_str(),display,callback);
        else if(display.is<bool>()) return  new BoolPropertyItem(parent,this,name.c_str(),display,callback);
        else if(display.is<Topic>()) return new TopicPropertyItem(parent,this,name.c_str(),display,callback);
        else return new JsonPropertyItem(parent,this,name.c_str(),display,callback);
    }
signals:
    void signalInsertPlay(Svar display);
public slots:
    void slotInsertPlay(Svar display){
        auto it=displays.find(display.value()->ptr());
        if(it!=displays.end())
            delete it->second;
        displays[display.value()->ptr()]=addDisplay(display);
    }

    void slotMenu(const QPoint& pos){
        PropertyItem *item = dynamic_cast<PropertyItem *>(this->itemAt(pos));
        if(!item) return;
        item->execMenu();
    }

    void slotItemClicked(QTreeWidgetItem* item,int col){
        auto ptr=dynamic_cast<PropertyItem*>(item);
        if(!ptr) return;
        ptr->clicked(col);
    }
public:
    std::map<const void*,PropertyItem*> displays;

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
