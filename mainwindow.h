#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "camerainterface.h"
#include "kinectinterface.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();


private:
    Ui::MainWindow *ui;
	KinectInterface *camera;
	//CameraInterface *camera;


private slots:
    void startButtonPressed();
    void stopButtonPressed();


};

#endif // MAINWINDOW_H
