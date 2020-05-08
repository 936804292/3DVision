#include "CloudViewer.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    CloudViewer m;
    m.show();

    return a.exec();
}
