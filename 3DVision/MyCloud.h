#ifndef MYCLOUD_H
#define MYCLOUD_H


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <string>
#include <QFileInfo>

#include "Tools.h"

using namespace std;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class MyCloud
{
public:
    MyCloud();
    ~MyCloud();

    bool isValid = false;
    PointCloudT::Ptr cloud;
    pcl::PolygonMesh::Ptr mesh; // polygon mesh pointer **build 3dModel pointer**

    string filePath;
    string fileDir;
    string fileName;
    string fileSuffix; //file name suffx  defualt:*.ply

    string cloudId;      // cloud id in `viewer`: "cloud-" + fileName
    string meshId;       // mesh id in `viewer`: "mesh-" + fileName

    bool hasCloud = false;
    bool hasMesh = false;

    bool visible = true;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    string currentMode = "point";
    vector<string> supportedMode;


    void setPointColor(int r,int g,int b);
    void setPointAlpha(int a);
    void setShowMode(const string &mode);
    void showCloud();
    void hideCloud();
    void showMesh();
    void hideMesh();
    void show();
    void hide();

    void init();
    void init(const QFileInfo& fileInfo, bool hasCloud, bool hasMesh);

    static MyCloud getInvalidMyCloud() {
        MyCloud myCloud;
        myCloud.isValid = false;
        return myCloud;
    }

};

#endif // MYCLOUD_H
