#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , commNode(nullptr)
{
    ui->setupUi(this);
    QImage img;
    img.load(":/icon/images/logo.png");
    img.scaled(ui->label_logo->width(),ui->label_logo->height());
    ui->label_logo->setPixmap(QPixmap::fromImage(img).scaled(ui->label_logo->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));

    ui->commandButton_Load->setIcon(QIcon(":/icon/images/load.png"));
    ui->commandButton_Load->setIconSize(QSize(24,24));
    ui->commandButton_Save->setIcon(QIcon(":/icon/images/save.png"));
    ui->commandButton_Save->setIconSize(QSize(24,24));

    commNode = rclcomm::getInstance();
    if (!commNode) {
        return; // 防止空指针
    }

    connect(commNode,SIGNAL(emitShowData()),this,SLOT(updateCalcRelInfo()));

    setupGroundTable(5);    //与雷达数量对应
    setupStereoTable(5);
    setupModifyTable();
}

void MainWindow::updateCalcRelInfo()
{
    if (!commNode || !commNode->check_instance) {
        return; // 防止空指针访问
    }

    // 检查各雷达的is_active状态并设置OnLine列
    std::vector<LidarConfig> lidars = {
        {commNode->init_params.lidar_front_top},
        {commNode->init_params.lidar_front_right},
        {commNode->init_params.lidar_front_left},
        {commNode->init_params.lidar_rear_top},
        {commNode->init_params.lidar_rear_center}
    };

    for (int i = 0; i < static_cast<int>(lidars.size()); ++i) 
    {
        QString status = lidars[i].is_active ? "On" : "Off";
        QTableWidgetItem *item = ui->table_lidar_ground->item(i, 1);
        if (item) {
            item->setText(status);
        }
        item = ui->table_lidar_ground->item(i, 2);
        if (item) {
            int plates_num = commNode->check_instance->getPlatesCount(lidars[i].sensor_name);
            item->setText(QString::number(plates_num));
            if("Off" == status) item->setText("0");
        }
    }

    for (int i = 0; i < static_cast<int>(lidars.size()); ++i) 
    {
        QString status = lidars[i].is_active ? "On" : "Off";
        QTableWidgetItem *item_x = ui->table_lidar_stereo->item(i, 2);
        if (item_x) {
            int plates_num = commNode->check_instance->getPlatesCount(lidars[i].sensor_name);
            item_x->setText(QString::number(plates_num));
            if("Off" == status) item_x->setText("0");
        }
        QTableWidgetItem *item_y = ui->table_lidar_stereo->item(i, 3);
        if (item_y) {
            int plates_for_y_num = commNode->check_instance->getPlates_for_y_Count(lidars[i].sensor_name);
            item_y->setText(QString::number(plates_for_y_num));
            if("Off" == status) item_y->setText("0");
        }
    }

    if(commNode->check_instance->ground_cali_done)
        ui->checkBox_model->setCheckState(Qt::Checked);
    else
        ui->checkBox_model->setCheckState(Qt::Unchecked);

    // 软件正常运行指示灯
    static bool isRunning = true;
    if (ui->label_run) {
        if (isRunning) {
            ui->label_run->setStyleSheet("background-color: rgba(20, 255, 208, 1);");
        } else {
            ui->label_run->setStyleSheet("background-color: rgba(96, 193, 0, 1);");
        }
    }
    isRunning = !isRunning;

}

MainWindow::~MainWindow()
{
    // 不要删除单例对象，它会自动管理生命周期
    commNode = nullptr;
    delete ui;
}


void MainWindow::setupGroundTable(int rowCnt)
{
    if (!commNode) {
        return; // 防止空指针访问
    }

    QStringList header{"雷达id", "OnLine", "Plates Num", "Check", "Capture"};
    ui->table_lidar_ground->setColumnCount(header.size());
    ui->table_lidar_ground->setHorizontalHeaderLabels(header);
    ui->table_lidar_ground->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->table_lidar_ground->verticalHeader()->setVisible(false);
    ui->table_lidar_ground->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->table_lidar_ground->setRowCount(rowCnt);

    std::vector<LidarConfig> lidars = {
        {commNode->init_params.lidar_front_top},
        {commNode->init_params.lidar_front_right},
        {commNode->init_params.lidar_front_left},
        {commNode->init_params.lidar_rear_top},
        {commNode->init_params.lidar_rear_center}
    };


    for (int i = 0; i < static_cast<int>(lidars.size()) && i < rowCnt; ++i) {
        ui->table_lidar_ground->setItem(i, 0, new QTableWidgetItem(QString::fromStdString(lidars[i].sensor_name)));
        ui->table_lidar_ground->setItem(i, 1, new QTableWidgetItem(QString("Off")));
        // 可根据实际需求设置 Plates Num
        ui->table_lidar_ground->setItem(i, 2, new QTableWidgetItem(QString("")));

        QCheckBox *checkBox = new QCheckBox(ui->table_lidar_ground);
        checkBox->setProperty("row", i);
        ui->table_lidar_ground->setCellWidget(i, 3, checkBox);
        connect(checkBox, &QCheckBox::stateChanged, this, [this, checkBox]() {
            int row = checkBox->property("row").toInt();
            bool checked = (checkBox->checkState() == Qt::Checked);
            QTableWidgetItem *item = ui->table_lidar_ground->item(row, 0);
            if (item) {
                QString sensor_name = item->text();
                commNode->setGroundCheckStatusByName(sensor_name.toStdString(), checked);
            }
        });

        QPushButton *btn = new QPushButton("详情", ui->table_lidar_ground);
        btn->setProperty("row", i);
        connect(btn, &QPushButton::clicked, this, [this]() {
            int row = sender()->property("row").toInt();
            QTableWidgetItem *item = ui->table_lidar_ground->item(row, 0);
            if (item) {
                QString sensor_name = item->text();
                auto plates = commNode->check_instance->analyzeLidarByName(sensor_name.toStdString());
                ui->textBrowser_ground->setPlainText(QString::fromStdString(plates.describe));
            }
            
        });
        ui->table_lidar_ground->setCellWidget(i, 4, btn);
    }
    // 将fr, fl 第4列的checkBox设置为勾选且不可修改
    if (1) {
        QWidget *widget = ui->table_lidar_ground->cellWidget(FRONT_RIGHT, 3);
        QCheckBox *cb = qobject_cast<QCheckBox *>(widget);
        if (cb) {cb->setChecked(true); cb->setEnabled(false);}  //地面校验时，fr fl两个雷达必须选中
        widget = ui->table_lidar_ground->cellWidget(FRONT_LEFT, 3);
        cb = qobject_cast<QCheckBox *>(widget);
        if (cb) {cb->setChecked(true); cb->setEnabled(false);}  //地面校验时，fr fl两个雷达必须选中
    }
}

void MainWindow::setupStereoTable(int rowCnt)
{
    if (!commNode) {
        return; // 防止空指针访问
    }

    std::vector<LidarConfig> lidars = {
        {commNode->init_params.lidar_front_top},
        {commNode->init_params.lidar_front_right},
        {commNode->init_params.lidar_front_left},
        {commNode->init_params.lidar_rear_top},
        {commNode->init_params.lidar_rear_center}
    };

    QStringList header{"雷达id", "To FR_Lidar", "X Plane Num", "Y Plane Num", "Check", "Capture"};
    ui->table_lidar_stereo->setColumnCount(header.size());
    ui->table_lidar_stereo->setHorizontalHeaderLabels(header);
    ui->table_lidar_stereo->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->table_lidar_stereo->verticalHeader()->setVisible(false);
    ui->table_lidar_stereo->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->table_lidar_stereo->setRowCount(rowCnt);


    for (int i = 0; i < static_cast<int>(lidars.size()) && i < rowCnt; ++i) {
        ui->table_lidar_stereo->setItem(i, 0, new QTableWidgetItem(QString::fromStdString(lidars[i].sensor_name)));
        // 可根据实际需求设置 Plates Num
        ui->table_lidar_stereo->setItem(i, 2, new QTableWidgetItem(QString("0")));
        ui->table_lidar_stereo->setItem(i, 3, new QTableWidgetItem(QString("0")));

        QCheckBox *checkBox = new QCheckBox(ui->table_lidar_stereo);
        checkBox->setProperty("row", i);
        ui->table_lidar_stereo->setCellWidget(i, 4, checkBox);
        connect(checkBox, &QCheckBox::stateChanged, this, [this, checkBox]() {
            int row = checkBox->property("row").toInt();
            bool checked = (checkBox->checkState() == Qt::Checked);
            QTableWidgetItem *item = ui->table_lidar_stereo->item(row, 0);
            if (item) {
                QString sensor_name = item->text();
                commNode->setStereoCheckStatusByName(sensor_name.toStdString(), checked);
            }
        });

        QPushButton *btn = new QPushButton("详情", ui->table_lidar_stereo);
        btn->setProperty("row", i);
        connect(btn, &QPushButton::clicked, this, [this]() {
            int row = sender()->property("row").toInt();
            QTableWidgetItem *item = ui->table_lidar_stereo->item(row, 0);
            if (item) {
                QString sensor_name = item->text();
                auto plates = commNode->check_instance->analyzeLidarByName(sensor_name.toStdString());
                ui->textBrowser_stereo->setPlainText(QString::fromStdString(plates.describe));
            }

            // TODO: 这里写你自己的逻辑
        });
        ui->table_lidar_stereo->setCellWidget(i, 5, btn);
    }
    if (rowCnt > 2) {
        QWidget *widget = ui->table_lidar_stereo->cellWidget(FRONT_RIGHT, 4);
        QCheckBox *cb = qobject_cast<QCheckBox *>(widget);
        if (cb) {cb->setChecked(true); cb->setEnabled(false);}  //地面校验时，fr fl两个雷达必须选中
    }
}

void MainWindow::setupModifyTable()
{
    if (!commNode) {
        return; // 防止空指针访问
    }
    QStringList header;
    header << QString::fromStdString(commNode->init_params.lidar_front_top.sensor_name)
        << QString::fromStdString(commNode->init_params.lidar_front_right.sensor_name)
        << QString::fromStdString(commNode->init_params.lidar_front_left.sensor_name)
        << QString::fromStdString(commNode->init_params.lidar_rear_top.sensor_name)
        << QString::fromStdString(commNode->init_params.lidar_rear_center.sensor_name);

    //设置平移参数微调表格---------------------
    ui->table_trans_modify->setColumnCount(header.size());
    ui->table_trans_modify->setHorizontalHeaderLabels(header);
    ui->table_trans_modify->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->table_trans_modify->verticalHeader()->setVisible(true);

    // 允许编辑表格内容
    ui->table_trans_modify->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::EditKeyPressed);

    // 设置三个行表头
    QStringList rowHeader_trans;
    rowHeader_trans << "X" << "Y" << "Z";
    ui->table_trans_modify->setRowCount(rowHeader_trans.size());
    ui->table_trans_modify->setVerticalHeaderLabels(rowHeader_trans);

    //设置旋转参数微调表格---------------------
    ui->table_rotate_modify->setColumnCount(header.size());
    ui->table_rotate_modify->setHorizontalHeaderLabels(header);
    ui->table_rotate_modify->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->table_rotate_modify->verticalHeader()->setVisible(true);

    // 允许编辑表格内容
    ui->table_rotate_modify->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::EditKeyPressed);

    // 设置三个行表头
    QStringList rowHeader_rotate;
    rowHeader_rotate << "X (Roll)" << "Y (Pitch)" << "Z (Yaw)";
    ui->table_rotate_modify->setRowCount(rowHeader_rotate.size());
    ui->table_rotate_modify->setVerticalHeaderLabels(rowHeader_rotate);

    on_Button_trans_setZero_clicked();
    on_Button_rotate_setZero_clicked();
}

void MainWindow::on_checkBox_model_stateChanged(int arg1)
{
    if (!commNode) {
        return; // 防止空指针访问
    }

    if(arg1 == Qt::Checked) commNode->check_instance->ground_cali_done = true;
    else commNode->check_instance->ground_cali_done = false;
}

void MainWindow::on_checkBox_locker_yaw_stateChanged(int arg1)
{
    if (!commNode) {
        return; // 防止空指针访问
    }

    if(arg1 == Qt::Checked) commNode->check_instance->yaw_cali_done = true;
    else commNode->check_instance->yaw_cali_done = false;
}


void MainWindow::on_Button_ground_clicked()
{
    std::string rel = commNode->check_instance->groundAnalysis();
    ui->textBrowser_ground->setPlainText(QString::fromStdString(rel));
}

void MainWindow::on_Button_stereo_clicked()
{
    std::string rel = commNode->check_instance->stereoAnalysis();
    ui->textBrowser_stereo->setPlainText(QString::fromStdString(rel));
}

void MainWindow::on_tabWidget_currentChanged(int index)
{
    //tabWidget切换
    if(0 == index)  //ground页面
    {
        commNode->switch_ground_wall = true;
    }
    else if(1 == index) //stereo页面
    {
        commNode->switch_ground_wall = false;
    }
}



void MainWindow::on_Button_cali_rp_clicked()
{
    std::string result = commNode->check_instance->groundAutoCali_RP(commNode->init_params);
    ui->textBrowser_ground->append(QString::fromStdString(result));
}


void MainWindow::on_Button_cali_z_clicked()
{
    std::string result = commNode->check_instance->groundAutoCali_Z(commNode->init_params);
    ui->textBrowser_ground->append(QString::fromStdString(result));
}


void MainWindow::on_Button_cali_yaw_clicked()
{
    std::string result = commNode->check_instance->stereoAutoCali_Yaw(commNode->init_params);
    ui->textBrowser_stereo->append(QString::fromStdString(result));
}


void MainWindow::on_Button_cali_x_clicked()
{
    std::string result = commNode->check_instance->stereoAutoCali_X(commNode->init_params);
    ui->textBrowser_stereo->append(QString::fromStdString(result));
}


void MainWindow::on_Button_cali_y_clicked()
{
    std::string result = commNode->check_instance->stereoAutoCali_Y(commNode->init_params);
    ui->textBrowser_stereo->append(QString::fromStdString(result));
}


void MainWindow::on_commandButton_Load_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("选择文件"), "", tr("yaml (*.yaml*)"));
    if (!fileName.isEmpty()) {
        std::cout << "Selected file: " << fileName.toStdString() << std::endl;
        if (commNode) 
        {
            if(!commNode->readCalibrationFromYaml(fileName.toStdString(), commNode->init_params))
            {
                QMessageBox::warning(this, tr("读取失败"), tr("文件读取失败，请检查文件格式或内容。"));
            }
            else
            {
                QMessageBox::information(this, tr("读取成功"), tr("文件读取成功。"));
            }
        }   
    }
}


void MainWindow::on_commandButton_Save_clicked()
{
    QString fileName = QFileDialog::getSaveFileName(this, tr("保存文件"), "", tr("yaml (*.yaml*)"));
    if (!fileName.isEmpty()) {
        std::cout << "Saving to file: " << fileName.toStdString() << std::endl;
        if (commNode) 
        {
            if(!commNode->writeYamlFromParams(fileName.toStdString(), commNode->init_params))
            {
                QMessageBox::warning(this, tr("保存失败"), tr("文件保存失败，请检查路径或权限。"));
            }
            else
            {
                QMessageBox::information(this, tr("保存成功"), tr("文件保存成功。"));
            }
        }   
    }
}

void MainWindow::on_Button_trans_setZero_clicked()
{
    // 将table_trans_modify中所有参数设置为0.0
    int rowCount = ui->table_trans_modify->rowCount();    // 3行 (X, Y, Z)
    int columnCount = ui->table_trans_modify->columnCount(); // 5列 (雷达)
    
    for (int row = 0; row < rowCount; ++row) {
        for (int col = 0; col < columnCount; ++col) {
            QTableWidgetItem *item = ui->table_trans_modify->item(row, col);
            if (item) {
                item->setText("0.0");
            } else {
                // 如果item不存在，创建一个新的item
                item = new QTableWidgetItem("0.0");
                item->setTextAlignment(Qt::AlignCenter);
                ui->table_trans_modify->setItem(row, col, item);
            }
        }
    }
}

void MainWindow::on_Button_trans_modify_clicked()
{
    if (!commNode)
    {
        ui->label_info_trans->setText("Err: 未初始化ROS2节点!");
        return;
    }

    int rowCount = ui->table_trans_modify->rowCount();    // 3行 (X, Y, Z)
    int columnCount = ui->table_trans_modify->columnCount(); // 5列 (雷达)

    for (int col = 0; col < columnCount; ++col) 
    {
        QString sensor_name = ui->table_trans_modify->horizontalHeaderItem(col)->text();
        std::vector<double> values_vec{0.0, 0.0, 0.0};
        for (int row = 0; row < rowCount; ++row) 
        {
            QTableWidgetItem *item = ui->table_trans_modify->item(row, col);
            if (item) 
            {
                bool ok = false;
                values_vec.at(row) = item->text().toDouble(&ok);
                if (!ok) 
                {
                    ui->label_info_trans->setText(QString("第%1行%2列输入错误, 请输入有效的数字！").arg(row + 1).arg(col + 1));
                    return;
                }
            }
        }
        commNode->setAxisModifyByName(sensor_name.toStdString(), values_vec);
    }
    ui->label_info_trans->setText("参数修改成功: " + QDateTime::currentDateTime().toString("HH:mm:ss"));
}

void MainWindow::on_Button_rotate_setZero_clicked()
{
    // 将table_rotate_modify中所有参数设置为0.0
    int rowCount = ui->table_rotate_modify->rowCount();    // 3行 (X, Y, Z)
    int columnCount = ui->table_rotate_modify->columnCount(); // 5列 (雷达)
    
    for (int row = 0; row < rowCount; ++row) {
        for (int col = 0; col < columnCount; ++col) {
            QTableWidgetItem *item = ui->table_rotate_modify->item(row, col);
            if (item) {
                item->setText("0.0");
            } else {
                // 如果item不存在，创建一个新的item
                item = new QTableWidgetItem("0.0");
                item->setTextAlignment(Qt::AlignCenter);
                ui->table_rotate_modify->setItem(row, col, item);
            }
        }
    }
}

void MainWindow::on_Button_rotate_modify_clicked()
{
    if (!commNode)
    {
        ui->label_info_rotate->setText("Err: 未初始化ROS2节点!");
        return;
    }

    int rowCount = ui->table_rotate_modify->rowCount();    // 3行 (X, Y, Z)
    int columnCount = ui->table_rotate_modify->columnCount(); // 5列 (雷达)

    for (int col = 0; col < columnCount; ++col) 
    {
        QString sensor_name = ui->table_rotate_modify->horizontalHeaderItem(col)->text();
        std::vector<double> values_vec{0.0, 0.0, 0.0};
        for (int row = 0; row < rowCount; ++row) 
        {
            QTableWidgetItem *item = ui->table_rotate_modify->item(row, col);
            if (item) 
            {
                bool ok = false;
                values_vec.at(row) = item->text().toDouble(&ok);
                if (!ok) 
                {
                    ui->label_info_rotate->setText(QString("第%1行%2列输入错误, 请输入有效的数字！").arg(row + 1).arg(col + 1));
                    return;
                }
            }
        }
        commNode->setRotateModifyByName(sensor_name.toStdString(), values_vec);
    }
    ui->label_info_rotate->setText("参数修改成功: " + QDateTime::currentDateTime().toString("HH:mm:ss"));
}
