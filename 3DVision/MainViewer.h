#ifndef MAINVIEWER_H
#define MAINVIEWER_H

#include <QMainWindow>


namespace Ui {
class MainViewer;
}

class MainViewer : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainViewer(QWidget *parent = 0);
    ~MainViewer();

private:
    Ui::MainViewer *ui;

};





















#endif // MAINVIEWER_H
