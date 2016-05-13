#include "mainwindow.h"
#include "build/ui_mainwindow.h"
#include <qDebug>
#include <QWidget>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //camera = new CameraInterface();
    camera = new KinectInterface();

    //if button is pressed, call function in this class
    connect(ui->startButton,SIGNAL(pressed()),this,SLOT(startButtonPressed()));
    connect(ui->stopButton,SIGNAL(pressed()),this,SLOT(stopButtonPressed()));

    //if [CameraInterface *camera] send QPixmap data to this class, update it to [ui->videoWidget]
    connect(camera, SIGNAL(getImageFromCamera(QPixmap)), ui->videoWidget, SLOT(setPixmap(QPixmap)));
}

MainWindow::~MainWindow()
{
    delete ui;
}

/*
* SLOT FUNCTION
*/
void MainWindow::startButtonPressed()
{
    qDebug() << "MainWindow::startButtonPressed";

    camera->startCapture();
}
void MainWindow::stopButtonPressed()
{
    qDebug() << "MainWindow::stopButtonPressed";

    camera->stopCapture();
}
