#include "MyCloud.h"
#include <pcl/conversions.h>

MyCloud::MyCloud()
{
    cloud.reset(new PointCloudT);
    mesh.reset(new pcl::PolygonMesh);
}

MyCloud::~MyCloud(){}

void MyCloud::init(const QFileInfo& fileInfo, bool hasCloudParam, bool hasMeshParam)
{
    hasCloud = hasCloudParam;
    hasMesh  = hasMeshParam;

    filePath = fromQString(fileInfo.filePath());
    fileDir = fromQString(fileInfo.path());
    fileName = fromQString(fileInfo.fileName());
    fileSuffix = fromQString(fileInfo.suffix());

    if (!hasCloud && !hasMesh) {
        isValid = false;
        return;
    }

    isValid = true;
    if (hasMesh) {
        meshId = "mesh-" + fileName;
        cloudId = "cloud-" + fileName;
        pcl::fromPCLPointCloud2(mesh->cloud, *cloud);
        setPointAlpha(255);
        supportedMode = {"point", "mesh", "point+mesh"};
    }
    if (hasCloud) {
        cloudId = "cloud-" + fileName;
        setPointAlpha(255);
        supportedMode = {"point"};
    }

    // default show node
    currentMode = "point";
}

void MyCloud::setPointColor(int r, int g, int b) {
    for(int i = 0; i != cloud->points.size(); ++i) {
        cloud->points[i].r = r;
        cloud->points[i].g = g;
        cloud->points[i].b = b;
    }

}

void MyCloud::setPointAlpha(int a) {
    for (int i = 0; i != cloud->points.size(); ++i) {
        cloud->points[i].a = a;
    }
}

void MyCloud::setShowMode(const string& mode) {
    currentMode = mode;
    show();
}

void MyCloud::showCloud() {
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY, 1.0, cloudId, 0);
}

void MyCloud::hideCloud() {
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY, 0.0, cloudId, 0);

}

void MyCloud::showMesh() {
    if (meshId == "") return; // no mesh
    viewer->addPolygonMesh(*mesh, meshId);
}

void MyCloud::hideMesh() {
    if (meshId == "") return;
    viewer->removePolygonMesh(meshId);
}

void MyCloud::show() {
    if (currentMode == "point") {
        hideMesh();
        showCloud();
    }
    else if (currentMode == "mesh") {
        hideCloud();
        showMesh();
    }
    else if (currentMode == "point+mesh") {
        showCloud();
        showMesh();
    }
}

void MyCloud::hide() {
    hideCloud();
    hideMesh();
}









