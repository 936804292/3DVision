#include "MainViewer.h"
#include "ui_MainViewer.h"

MainViewer::MainViewer(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainViewer)
{
    ui->setupUi(this);
}

MainViewer::~MainViewer()
{
    delete ui;
}
