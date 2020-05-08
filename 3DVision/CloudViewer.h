#ifndef CLOUDVIEWER_H
#define CLOUDVIEWER_H

#include <QMainWindow>

#include <vtkAutoInit.h>
#include <vtkRenderWindow.h>

#include <opencv2/opencv.hpp>

#include "MyCloud.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>  // loadPolygonFileOBJ

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PolygonMesh.h>

#include <QString>
#include <QFileDialog>
#include <QMessageBox>
#include <QStatusBar>

#include "FileIO.h"
#include "Tools.h"
#include "MyAlgorithm.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using std::vector;
using std::string;
using std::map;

namespace Ui {
class CloudViewer;
}

class CloudViewer : public QMainWindow
{
    Q_OBJECT

public:
    explicit CloudViewer(QWidget *parent = 0);
    ~CloudViewer();

private slots:
    void open();
    void clear();
    void ICPAlgorithm();
    void exit();

private:
    Ui::CloudViewer *ui;

    pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud;
    MyCloud mycloud;
    std::vector<MyCloud> mycloud_vec;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    long total_points = 0; //Total amount of points in the viewer


    QString timeCostSecond = "0";
    void doOpen(const QStringList& filePathList);
    void consoleLog(QString operation, QString subname, QString filename, QString note);

    void showPointcloudAdd();
    void setPropertyTable();
    void showPointcloud();
    FileIO fileIO;

    QString save_filename;

    bool enable_console = true; // console 的可用状态

    void initial();
    void setConsoleTable();
};

#endif // CLOUDVIEWER_H
