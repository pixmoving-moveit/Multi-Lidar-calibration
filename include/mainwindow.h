#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTableWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHeaderView>
#include <QCheckBox>
#include <QMessageBox>
#include <QFileDialog>
#include <QDateTime>
#include <QIcon>
#include <QDebug>
#include "rclcomm.h"
#include "check.h"
#include <iostream>
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    rclcomm *commNode;

    void setupGroundTable(int rowCnt);
    void setupStereoTable(int rowCnt);
    void setupModifyTable();

public slots:
    void updateCalcRelInfo();

private slots:
    void on_checkBox_model_stateChanged(int arg1);
    void on_checkBox_locker_yaw_stateChanged(int arg1);
    void on_Button_ground_clicked();
    void on_Button_trans_setZero_clicked();
    void on_Button_trans_modify_clicked();
    void on_Button_rotate_setZero_clicked();
    void on_Button_rotate_modify_clicked();
    void on_tabWidget_currentChanged(int index);
    void on_Button_stereo_clicked();
    void on_Button_cali_rp_clicked();
    void on_Button_cali_z_clicked();
    void on_Button_cali_yaw_clicked();
    void on_Button_cali_x_clicked();
    void on_Button_cali_y_clicked();
    void on_commandButton_Load_clicked();
    void on_commandButton_Save_clicked();
};
#endif // MAINWINDOW_H
