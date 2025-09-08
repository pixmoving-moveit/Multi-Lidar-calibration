#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.setWindowTitle("Lidar Calibration Tool");
    w.setWindowIcon(QIcon(":/icon/images/cali.png"));
    w.show();
    return a.exec();
}
