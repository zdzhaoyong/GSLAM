#include <QWidget>
#include <QListWidget>
#include <QDockWidget>
#include <QTableWidget>
#include <QHeaderView>
#include <GSLAM/core/GSLAM.h>

namespace GSLAM {

class PanelPubsub : QDockWidget
{
    Q_OBJECT
public:
    PanelPubsub(QWidget* parent,Svar config):QDockWidget(parent){
        table=new QTableWidget(this);
        setWidget(table);
        table->setColumnCount(3);
        table->setHorizontalHeaderLabels({"Name","Type","PayLoad"});

        QHeaderView *HorzHdr = table->horizontalHeader();
    #if QT_VERSION>=0x050000
        HorzHdr->setSectionResizeMode(QHeaderView::Stretch);
        HorzHdr->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    #else
        HorzHdr->setResizeMode(QHeaderView::Stretch);
    #endif

        data["newSub"]=messenger.subscribe("messenger/newsub",[this](Subscriber sub){
            this->updateTable();
        });
        data["newPub"]=messenger.subscribe("messenger/newpub",[this](Publisher pub){
            this->updateTable();
        });
        updateTable();
    }

    QTableWidgetItem* setValue(int row,int col,QString val)
    {
        if(table->item(row,col)!=NULL)
        {
            table->item(row,col)->setText(val);
            return table->item(row,col);
        }
        else
        {
            QTableWidgetItem* item=new QTableWidgetItem();
            item->setText(val);
            table->setItem(row,col,item);
            return item;
        }
    }

    void   updateTable()
    {
        std::vector<std::array<std::string,3>> pubsubs;

        for(Publisher pub:messenger.getPublishers()){
            pubsubs.push_back({pub.getTopic(),"Publisher",pub.getTypeName()});
        }
        for(Subscriber pub:messenger.getSubscribers()){
            pubsubs.push_back({pub.getTopic(),"Subscriber",pub.getTypeName()});
        }

        if(table->rowCount()!=pubsubs.size())
        {
            table->setRowCount(pubsubs.size());
            int i=0;
            for(auto it:pubsubs)
            {
                setValue(i,0,it[0].c_str())->setFlags(Qt::ItemIsEditable);
                setValue(i,1,it[1].c_str())->setFlags(Qt::ItemIsEditable);
                setValue(i,2,it[2].c_str())->setFlags(Qt::ItemIsEditable);
                i++;
            }
        }
        else{
            for(int i=0;i<table->rowCount();i++)
            {
                for(int j=0;j<3;j++)
                    table->item(i,j)->setText(pubsubs[i][j].c_str());
            }
        }
        update();
    }

    QTableWidget* table;
    Svar          data;
};

}
