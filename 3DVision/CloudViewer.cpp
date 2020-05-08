#include "CloudViewer.h"
#include "ui_CloudViewer.h"

CloudViewer::CloudViewer(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::CloudViewer)
{
    ui->setupUi(this);
    /***** Slots connection of QMenuBar and QToolBar *****/
    // File (connect)
    connect(ui->actionOpen,SIGNAL(triggered(bool)),this,SLOT(open()));
    connect(ui->actionClear,SIGNAL(triggered(bool)),this,SLOT(clear()));
    connect(ui->actionExit,SIGNAL(triggered(bool)),this,SLOT(exit()));

    /***** Slots connection of QMenuBar and QToolBar *****/
    // Process (connect)
    connect(ui->actionICP,SIGNAL(triggered(bool)),this,SLOT(ICPAlgorithm()));

    initial();
}

CloudViewer::~CloudViewer()
{
    delete ui;
}

void CloudViewer::initial()
{
    setWindowIcon(QIcon(tr(":/Resources/images/logo.png")));
    setWindowTitle("3DVision");

    //init pointcloud
    mycloud.cloud.reset(new PointCloudT);
    mycloud.cloud->resize(1);
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    //viewer->addPointCloud(cloud, "cloud");

    ui->screen->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->screen->GetInteractor(), ui->screen->GetRenderWindow());
    ui->screen->update();
    viewer->setBackgroundColor(255.0, 255.0, 255.0);
    viewer->setWindowBorders(true);
    setConsoleTable();

    consoleLog("Software start", "FileName", "FilePath", "KZVision");


}


void CloudViewer::open()
{
    QStringList filePathList = QFileDialog::getOpenFileNames(this,tr("Open point cloud file"),toQString(mycloud.fileDir),toQString(fileIO.getInputFormatsStr()));
    if(filePathList.isEmpty()) return;

    // Clear cache
    // TODO: abstract a function
    mycloud_vec.clear();
    total_points = 0;
    ui->dataTree->clear();
    viewer->removeAllPointClouds();

    doOpen(filePathList);
}

void CloudViewer::doOpen(const QStringList& filePathList)
{
    //open pointCloud one by one
    for(int i = 0;i <filePathList.size();i++)
    {
        timeStart();
        mycloud.cloud.reset(new PointCloudT); //reset cloud
        QFileInfo fileinfo(filePathList[i]);
        std::string filePath = fromQString(fileinfo.filePath());
        std::string fileName = fromQString(fileinfo.fileName());

        //**************start loading pointcloud************//
        ui->statusbar->showMessage(fileinfo.fileName() + ": " + QString::number(i) + "/" + QString::number(filePathList.size())+" point cloud loading...");

        mycloud = fileIO.load(fileinfo);
        if(!mycloud.isValid)
        {
            qDebug("invalid cloud");
            continue;
        }

        mycloud.viewer = viewer;
        mycloud_vec.push_back(mycloud);

        timeCostSecond = timeOff();

        consoleLog("Open",
            toQString(mycloud.fileName),
            toQString(mycloud.filePath),
            "Time cost: " + timeCostSecond + " s, Points: " + QString::number(mycloud.cloud->points.size()));

        QTreeWidgetItem *cloudName = new QTreeWidgetItem(QStringList()
            << toQString(mycloud.fileName));
        cloudName->setIcon(0, QIcon(":/Resources/images/logo.png"));
        ui->dataTree->addTopLevelItem(cloudName);

        total_points += mycloud.cloud->points.size();


    }

    ui->statusbar->showMessage("");
    showPointcloudAdd();
    setPropertyTable();

}

void CloudViewer::exit()
{

}

void CloudViewer::clear()
{

}

void CloudViewer::setConsoleTable()
{
    // 设置输出窗口
    QStringList header2;
    header2 << "Time" << "Operation" << "Operation object" << "Details" << "Note";
    ui->consoleTable->setHorizontalHeaderLabels(header2);
    ui->consoleTable->setColumnWidth(0, 150);
    ui->consoleTable->setColumnWidth(1, 200);
    ui->consoleTable->setColumnWidth(2, 200);
    ui->consoleTable->setColumnWidth(3, 300);

    //ui->consoleTable->setEditTriggers(QAbstractItemView::NoEditTriggers); //设置不可编辑
    ui->consoleTable->verticalHeader()->setDefaultSectionSize(26); //设置行距

    ui->consoleTable->setContextMenuPolicy(Qt::CustomContextMenu);
}

void CloudViewer::consoleLog(QString operation, QString subname, QString filename, QString note)
{
    if (enable_console == false) {
        return;
    }
    int rows = ui->consoleTable->rowCount();
    ui->consoleTable->setRowCount(++rows);
    QDateTime time = QDateTime::currentDateTime();//获取系统现在的时间
    QString time_str = time.toString("yyyy-MM-dd hh:mm:ss"); //设置显示格式
    ui->consoleTable->setItem(rows - 1, 0, new QTableWidgetItem(time_str));
    ui->consoleTable->setItem(rows - 1, 1, new QTableWidgetItem(operation));
    ui->consoleTable->setItem(rows - 1, 2, new QTableWidgetItem(subname));
    ui->consoleTable->setItem(rows - 1, 3, new QTableWidgetItem(filename));
    ui->consoleTable->setItem(rows - 1, 4, new QTableWidgetItem(note));

    ui->consoleTable->scrollToBottom(); // 滑动自动滚到最底部
}

void CloudViewer::showPointcloud() {
    for (int i = 0; i != mycloud_vec.size(); i++)
    {
        viewer->updatePointCloud(mycloud_vec[i].cloud, mycloud_vec[i].cloudId);
    }
    //viewer->resetCamera();
    ui->screen->update();
}

void CloudViewer::showPointcloudAdd()
{
    for (int i = 0; i < mycloud_vec.size(); i++) {
        viewer->addPointCloud(mycloud_vec[i].cloud, mycloud_vec[i].cloudId);
        viewer->updatePointCloud(mycloud_vec[i].cloud, mycloud_vec[i].cloudId);
    }
    viewer->resetCamera();
    ui->screen->update();
}
void CloudViewer::setPropertyTable(){}

/**
 * 对齐对象模板集合到一个示例点云
 * 调用命令格式 ./template_alignment2 ./data/object_templates.txt ./data/person.pcd
 *                  程序                          多个模板的文本文件         目标点云
 * 调用命令格式 ./template_alignment2 ./data/object_templates2.txt ./data/target.pcd
 *                  程序                          多个模板的文本文件         目标点云
 *
 * 实时的拍照得到RGB和深度图
 * 合成目标点云图
 * 通过直通滤波框定范围（得到感兴趣区域）
 * 将感兴趣区域进行降采样（提高模板匹配效率）
 */
void CloudViewer::ICPAlgorithm()
{
    qDebug("icp!!!!!!!!!!!!!!!!!!");

}
